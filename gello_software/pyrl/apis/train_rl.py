import itertools
import os
import os.path as osp
import time
from collections import defaultdict
from datetime import datetime

import numpy as np
from pyrl.env import ReplayMemory, save_eval_statistics
from pyrl.utils.data import GDict, dict_to_str, is_not_null, num_to_str, to_float
from pyrl.utils.math import EveryNSteps
from pyrl.utils.meta import get_logger, get_total_memory, get_world_rank, td_format


class EpisodicStatistics:
    def __init__(self, num_procs, info_keys_mode={}):
        self.num_procs = num_procs
        self.info_keys_mode = {
            "rewards": [True, "sum", "all"],
            "max_single_R": [True, "max", "all"],
            "lens": [True, "sum", "all"],
        }  # if used in print, stats over episode, if print all infos
        self.info_keys_mode.update(info_keys_mode)
        for key, item in self.info_keys_mode.items():
            assert item[1] in ["mean", "min", "max", "sum"]
            assert item[2] in ["mean", "all"]
        self.num_workers = 1
        self.reset_current()
        self.reset_history()

    def push(self, trajs):
        rewards, dones, index, infos = trajs["rewards"], trajs["episode_dones"], trajs.get("worker_indices", None), trajs.get("infos", None)
        dones = dones.reshape(-1)
        self.expand_buffer(index)
        for j in range(len(rewards)):
            i = 0 if index is None else int(index[j])
            self.current[i]["lens"] += 1
            self.current[i]["rewards"] += to_float(rewards[j])
            if "max_single_R" not in self.current[i]:
                self.current[i]["max_single_R"] = -np.inf
            self.current[i]["max_single_R"] = max(self.current[i]["max_single_R"], to_float(rewards[j]))

            if infos is not None:
                for key, manner in self.info_keys_mode.items():
                    if key != "rewards" and key in infos:
                        if manner[1] in ["sum", "mean"]:
                            self.current[i][key] += to_float(infos[key][j])
                        elif manner[1] == "min":
                            if key not in self.current[i]:
                                self.current[i][key] = np.inf
                            self.current[i][key] = np.minimum(self.current[i][key], to_float(infos[key][j]))
                        elif manner[1] == "max":
                            if key not in self.current[i]:
                                self.current[i][key] = -np.inf
                            self.current[i][key] = np.maximum(self.current[i][key], to_float(infos[key][j]))

            if dones[j]:
                for key, value in self.current[i].items():
                    if key not in ["rewards", "max_single_R", "lens"] and self.info_keys_mode[key][1] == "mean":
                        value /= self.current[i]["lens"]
                    self.history[key].append(value)
                self.current[i] = defaultdict(float)

    def expand_buffer(self, index):
        max_index = np.max(index) + 1
        for i in range(max(max_index - self.num_workers, 0)):
            self.current.append(defaultdict(float))

    def reset_history(self):
        self.history = defaultdict(list)

    def reset_current(self):
        self.current = [
            defaultdict(float),
        ]

    def get_sync_stats(self):
        num_ep = GDict(len(self.history["rewards"])).allreduce(op="SUM", wrapper=False)

        history_min, history_max, history_sum = {}, {}, {}
        for key, value in self.history.items():
            value = np.stack(value, axis=0)
            history_min[key] = value.min(0)
            history_max[key] = value.max(0)
            history_sum[key] = value.sum(0)

        history_min = GDict(history_min).allreduce(op="MIN", wrapper=False)
        history_max = GDict(history_max).allreduce(op="MAX", wrapper=False)
        history_sum = GDict(history_sum).allreduce(op="SUM", wrapper=False)
        history_mean = {key: item / num_ep for key, item in history_sum.items()}
        return history_min, history_max, history_mean

    def get_stats_str(self):
        history_min, history_max, history_mean = self.get_sync_stats()
        ret = ""
        for key, item in self.info_keys_mode.items():
            if not (key in history_mean and item[0]) or (isinstance(history_mean[key], np.ndarray) and history_mean[key].size > 1):
                continue
            if len(ret) > 0:
                ret += ", "
            if key == "lens":
                precision = 0
            elif key == "rewards":
                precision = 1
            else:
                precision = 2
            mean_i = num_to_str(history_mean[key], precision=precision)
            min_i = num_to_str(history_min[key], precision=precision)
            max_i = num_to_str(history_max[key], precision=precision)

            ret += f"{key}:{mean_i}"
            if item[2] == "all":
                ret += f"[{min_i}, {max_i}]"
        return ret

    def get_stats(self):
        history_min, history_max, history_mean = self.get_sync_stats()
        ret = {}
        for key in self.info_keys_mode:
            if key in history_mean:
                out_key = key if "/" in key else f"env/{key}"
                ret[f"{out_key}_mean"] = history_mean[key]
                ret[f"{out_key}_min"] = history_min[key]
                ret[f"{out_key}_max"] = history_max[key]
        return ret


def train_rl(
    agent,
    rollout,
    evaluator,
    replay,
    on_policy,
    work_dir,
    expert_replay=None,
    recent_traj_replay=None,
    total_steps=1000000,
    warm_steps=10000,
    resume_steps=0,
    use_policy_to_warm_up=False,
    print_steps=None,
    n_steps=1,
    n_updates=1,
    n_checkpoint=None,
    n_log=1000,
    n_eval=None,
    eval_cfg=None,
    ep_stats_cfg={},
    warm_up_updates=0,
    # warm_up_train_q_only=-1,
    exp_logger_cfg=dict(type="tensorboard"),
    clean_up=False,
    timestamp=None,
):
    world_rank = get_world_rank()
    logger = get_logger()
    agent.set_mode(mode="train")

    import torch
    from pyrl.utils.torch import get_cuda_info
    from pyrl.utils.torch import save_checkpoint
    from pyrl.utils.exp_logger import build_exp_logger

    exp_logs = ReplayMemory(n_updates, None)
    exp_logs.reset()

    if "log_dir" not in exp_logger_cfg:
        exp_logger_cfg["log_dir"] = work_dir
    exp_logger_cfg["work_dir"] = work_dir
    exp_logger_cfg["clean_up"] = clean_up
    exp_logger_cfg["timestamp"] = timestamp

    if world_rank == 0:
        exp_logger = build_exp_logger(exp_logger_cfg)

    if print_steps is None:
        print_steps = n_steps

    if on_policy:
        rollout.full_episode = True

    checkpoint_dir = osp.join(work_dir, "models")
    os.makedirs(checkpoint_dir, exist_ok=True)
    if rollout is not None:
        obs = rollout.reset()
        action = rollout.action_space.sample()
        agent.reset()
        logger.info(f"Rollout observation shape: {GDict(obs).shape}, action shape: {action.shape}!")
        logger.info(f"Rollout observation dtype: {GDict(obs).dtype}, action dtype: {action.dtype}!")
        episode_statistics = EpisodicStatistics(rollout.num_envs, **ep_stats_cfg)
        total_episodes = 0
    else:
        # Batch RL
        all_shape = replay.memory.shape
        logger.info(f"Loaded replay buffer shape: {all_shape}!")

    check_eval = EveryNSteps(n_eval)
    # if is_not_null(n_eval) and (n_checkpoint is None or n_checkpoint > n_eval):
    #     n_checkpoint = n_eval

    check_checkpoint = EveryNSteps(n_checkpoint)
    check_exp_logs = EveryNSteps(n_log)

    total_updates = 0
    print_replay_shape = False

    if warm_steps > 0:
        logger.info(f"Begin {warm_steps} warm-up steps with {'initial policy' if use_policy_to_warm_up else 'random policy'}!")
        # Randomly warm up replay buffer for model-free RL and learned model for mode-based RL
        assert (not on_policy) and (rollout is not None), f"Only support warm-up for off-policy online RL!"
        trajectories = rollout.forward_with_policy(agent if use_policy_to_warm_up else None, warm_steps, replay=replay)
        episode_statistics.push(trajectories)
        logger.info(f"Warm up samples stats: {episode_statistics.get_stats_str()}!")
        rollout.reset()
        agent.reset()
        episode_statistics.reset_current()

        check_eval.check(warm_steps)
        check_checkpoint.check(warm_steps)
        check_exp_logs.check(warm_steps)
        logger.info(f"Finish {warm_steps} warm-up steps!")


        for i in range(warm_up_updates):
            total_updates += 1
            training_infos = agent.update_parameters(replay, updates=total_updates)
            exp_logs.push(training_infos)
    warm_steps = GDict(warm_steps).allreduce(op="SUM", wrapper=False)

    steps = resume_steps + warm_steps
    total_steps += resume_steps  # Run for another total_steps after resume steps
    begin_steps = steps

    grad_steps = 0
    begin_time = datetime.now()
    max_ETA_len = None
    logger.info("Begin training!")

    for iteration_id in itertools.count(1):
        if steps >= total_steps:
            break

        if rollout is not None:
            # For online RL algorithm
            episode_statistics.reset_history()
            if on_policy:
                replay.reset()
                rollout.reset()
                agent.reset()
                episode_statistics.reset_current()

        tb_print = defaultdict(list)
        tb_log = {}
        print_log = {}

        update_time = 0
        time_begin_episode = time.time()
        tmp_steps = 0

        if n_steps > 0:
            # For online RL
            collect_sample_time = 0
            num_episodes = 0
            if recent_traj_replay is not None:
                recent_traj_replay.reset()
            """
            For on-policy algorithm, we will print training infos for every gradient batch.
            For off-policy algorithm, we will print training infos for every n_steps epochs.
            """
            while num_episodes < print_steps and not (on_policy and num_episodes > 0):
                # Collect samples
                start_time = time.time()
                agent.eval()  # For things like batch norm
                trajectories = rollout.forward_with_policy(agent, n_steps, replay)

                if not print_replay_shape:
                    print_replay_shape = True
                    logger.info(f"Replay buffer shape: {replay.memory.shape}.")
                agent.train()

                if trajectories is not None:
                    if recent_traj_replay is not None:
                        recent_traj_replay.push_batch(trajectories)

                    episode_statistics.push(trajectories)
                    n_ep = np.sum(trajectories["episode_dones"].astype(np.int32))
                    num_episodes += n_ep
                    tmp_steps += len(trajectories["rewards"])
                collect_sample_time += time.time() - start_time

                # Train agent
                for i in range(n_updates):
                    total_updates += 1
                    start_time = time.time()
                    extra_args = {} if expert_replay is None else dict(expert_replay=expert_replay)
                    training_infos = agent.update_parameters(replay, updates=total_updates, **extra_args)
                    # torch.cuda.empty_cache()
                    grad_steps += training_infos.get("grad_steps", 1)

                    for key in training_infos:
                        tb_print[key].append(training_infos[key])
                    update_time += time.time() - start_time

                if hasattr(agent, "update_discriminator"):
                    assert recent_traj_replay is not None
                    start_time = time.time()
                    disc_update_applied = agent.update_discriminator(expert_replay, recent_traj_replay, n_ep)
                    if disc_update_applied:
                        recent_traj_replay.reset()
                    update_time += time.time() - start_time
            tb_print = {key: np.mean(tb_print[key]) for key in tb_print}

            ep_stats = episode_statistics.get_stats()

            episode_time, collect_sample_time = GDict([time.time() - time_begin_episode, collect_sample_time]).allreduce(op="MAX", wrapper=False)
            tmp_steps, num_episodes = GDict([tmp_steps, int(num_episodes)]).allreduce(op="SUM", wrapper=False)

            steps += tmp_steps
            total_episodes += num_episodes

            print_log["samples_stats"] = episode_statistics.get_stats_str()
            tb_print.update(dict(episode_time=episode_time, collect_sample_time=collect_sample_time))

            tb_log.update(ep_stats)
            tb_log.update(dict(num_episodes=num_episodes, total_episodes=total_episodes))
        else:
            # For offline RL
            for i in range(n_updates):
                total_updates += 1
                steps += 1

                tmp_time = time.time()
                training_infos = agent.update_parameters(replay, updates=total_updates)
                grad_steps += training_infos.get("grad_steps", 1)
                update_time += time.time() - tmp_time

                for key in training_infos:
                    tb_print[key].append(training_infos[key])
            tb_print = {key: np.mean(tb_print[key]) for key in tb_print}
        tb_log["update_time"] = update_time
        tb_log["total_steps"] = int(total_steps)
        tb_log["total_updates"] = int(total_updates)
        tb_log["total_grad_steps"] = grad_steps
        tb_log["buffer_size"] = len(replay)

        tb_print, tb_log = GDict([tb_print, tb_log]).allreduce(wrapper=False)
        total_memory = GDict(get_total_memory("G", False)).allreduce(op="SUM", wrapper=False)
        tb_print["memory"] = total_memory
        print_log.update(get_cuda_info(device=torch.cuda.current_device(), number_only=False))

        if world_rank != 0:
            continue
        tb_log.update(tb_print)
        # print(tb_log)
        print_log.update(tb_print)
        print_log = {key.split("/")[-1]: val for key, val in print_log.items()}
        # print(print_log)
        # exit(0)

        if check_exp_logs.check(steps):
            # print(GDict(tb_log).shape, GDict(tb_log).type)
            exp_logger.log(tb_log, n_iter=steps, tag_name="train")
            # exit(0)

        percentage = f"{((steps - begin_steps) / (total_steps - begin_steps)) * 100:.0f}%"
        passed_time = td_format(datetime.now() - begin_time)
        ETA = td_format((datetime.now() - begin_time) * max((total_steps - begin_steps) / (steps - begin_steps) - 1, 0))
        if max_ETA_len is None:
            max_ETA_len = len(ETA)

        logger.info(f"{steps}/{total_steps}({percentage}) Passed time:{passed_time} ETA:{ETA} {dict_to_str(print_log)}")

        if check_eval.check(steps) and is_not_null(evaluator):
            standardized_eval_step = check_eval.standard(steps)
            logger.info(f"Begin to evaluate at step: {steps}. " f"The evaluation info will be saved at eval_{standardized_eval_step}")
            eval_dir = osp.join(work_dir, f"eval_{standardized_eval_step}")

            agent.eval()  # For things like batch norm
            agent.set_mode(mode="test")  # For things like obs normalization

            lens, rewards, finishes = evaluator.run(agent, **eval_cfg, work_dir=eval_dir)
            # agent.recover_data_parallel()

            torch.cuda.empty_cache()
            save_eval_statistics(eval_dir, lens, rewards, finishes)
            agent.train()
            agent.set_mode(mode="train")

            eval_dict = dict(mean_length=np.mean(lens), std_length=np.std(lens), mean_reward=np.mean(rewards), std_reward=np.std(rewards))
            exp_logger.log(eval_dict, n_iter=steps, tag_name="test")

        if check_checkpoint.check(steps):
            standardized_ckpt_step = check_checkpoint.standard(steps)
            model_path = osp.join(checkpoint_dir, f"model_{standardized_ckpt_step}.ckpt")
            logger.info(f"Save model at step: {steps}. The model will be saved at {model_path}")
            agent.to_normal()
            save_checkpoint(agent, model_path)
            agent.recover_ddp()

    if n_checkpoint and world_rank == 0:
        model_path = osp.join(checkpoint_dir, f"model_final.ckpt")
        logger.info(f"Save checkpoint at final step {total_steps}. The model will be saved at {model_path}.")
        agent.to_normal()
        save_checkpoint(agent, model_path)
        agent.recover_ddp()
