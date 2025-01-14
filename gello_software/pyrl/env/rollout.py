import numpy as np
from math import ceil
from tqdm import tqdm

from pyrl.utils.data import DictArray, GDict, to_np
from pyrl.utils.meta import get_logger, get_world_size, Timer, empty_context_manager

from .builder import ROLLOUTS

from .env_utils import build_vec_env
from .replay_buffer import ReplayMemory


@ROLLOUTS.register_module()
class Rollout:
    def __init__(self, env_cfg, num_procs=20, with_info=False, save_hidden=False, full_episode=False, seed=None, partial_forward=1, **kwargs):
        if seed is None:
            seed = np.random.randint(0, int(1e9))

        get_logger().info(f"Rollout environments have seed from [{seed}, {seed + num_procs})")
        if env_cfg is not None:
            self.vec_env = build_vec_env(env_cfg, num_procs, seed=seed, **kwargs)
        self._is_closed = False

        self.with_info = with_info
        self.save_hidden = save_hidden
        self.full_episode = full_episode

        self.num_envs = self.vec_env.num_envs
        self.partial_forward = partial_forward if self.num_envs > 1 else 1
        self.rnn_states = GDict(None)

    def __getattr__(self, name):
        return getattr(self.vec_env, name)

    def reset(self, idx=None, *args, **kwargs):
        assert not self._is_closed, "The rollout is closed!"
        if self.partial_forward < 1:
            self.vec_env.wait_dict()
        if idx is not None:
            kwargs = dict(**kwargs, idx=idx)
        self.rnn_states.assign(idx, 0)
        return self.vec_env.reset(*args, **kwargs)

    def _process_ret(self, infos):
        assert (
            isinstance(infos, dict) and len(infos) == 9
        ), f"Output of step_dict should have length 9! The info have type {type(infos)} and size {len(infos)}, keys {list(infos[-1].keys())}!"
        if not self.with_info:
            infos.pop("infos")

    def forward_with_policy(self, pi=None, num: int = 1, replay: ReplayMemory = None):
        assert not self._is_closed, "The rollout is closed!"
        if pi is None:
            ret = self.vec_env.step_random_actions(num)
            # print(GDict(ret).shape)
            self._process_ret(ret)
            if replay is not None:
                if not self.full_episode:
                    replay.push_batch(ret)
                else:
                    replay.cache_trajectories(ret)
                    replay.push_cached_trajectories()
                # print(replay.shape, len(replay))
            return ret

        timer, sync_timer = Timer(), Timer()
        num_forward_envs, step_times = [], []

        import torch
        from pyrl.utils.torch import barrier, build_dist_var

        tqdm_logger = tqdm(total=num, dynamic_ncols=True, disable=True)

        # if replay is not None and replay.memory is not None:
        #     print(replay.memory.shape, replay.memory.type)

        @torch.no_grad()
        def get_actions(idx=None, with_states=False):
            # Get action on gpu
            done_index = self.vec_env.done_idx
            if len(done_index) > 0:
                assert False, "We should already reset the environment!"
                self.reset(idx=done_index)
            idx = idx if idx is not None else slice(None)

            obs = DictArray(self.vec_env.recent_obs).slice(idx, wrapper=False)

            prev_actions = self.vec_env.recent_actions[idx]

            manager = pi.no_sync(mode="actor") if hasattr(pi, "no_sync") else empty_context_manager()
            with manager:
                # print("Get rnn state", self.rnn_states.shape, self.rnn_states.slice(idx).shape, idx)

                actions, next_rnn_states = pi(
                    obs, rnn_states=self.rnn_states.slice(idx, wrapper=False), rnn_mode="with_states", prev_actions=prev_actions
                )

                # import torch

                # print(obs, actions, actions.mean().item(), obs.mean().item())
                # exit(0)

                if self.rnn_states.memory is None and next_rnn_states is not None:
                    self.rnn_states = DictArray(GDict(next_rnn_states).slice(0, wrapper=False), capacity=self.num_envs).copy()
                    self.rnn_states.assign_all(0)

                current_states = self.rnn_states.slice(idx).copy(wrapper=False)
                self.rnn_states.assign(idx, next_rnn_states)
            # print("Set rnn state", self.rnn_states.shape)
            # exit(0)
            # print(GDict(obs).mean(), actions)
            # exit(0)
            return (actions, current_states, next_rnn_states) if with_states else actions

        if self.full_episode:
            # assert replay is not None, "Directly save samples to replay buffer to save memory."
            world_size = get_world_size()
            num_done = build_dist_var("num_done", "int")
            total, finished, ret = 0, 0, None

            sync_timer.since_last_check()

            # print(1, torch.rand(10, device="cuda:0"))
            # print(1, torch.rand(10, device="cpu"))
            step_idx = None
            timer.since_last_check("overhead")
            actions, rnn_states, next_rnn_states = GDict(get_actions(step_idx, with_states=True)).detach().to_numpy(wrapper=False)
            # print("action", actions, self.vec_env.recent_obs)
            # print(2, torch.rand(10, device="cuda:0"))
            # print(2, torch.rand(10, device="cpu"))

            # exit(0)

            # print(0, actions)

            timer.since_last_check("agent")

            while total < num:
                import sapien.core as sapien

                with sapien.ProfilerBlock("Simulation"):
                    timer.since_last_check("overhead")
                    if self.partial_forward == 1:
                        items = self.vec_env.step_dict(actions)
                    else:
                        self.vec_env.step_async(actions, idx=step_idx)
                        step_idx, items = self.vec_env.wait_dict(int(ceil(self.num_envs * self.partial_forward)))
                    timer.since_last_check("sim")
                step_times.append(items["infos"]["step_times"][:, 0])
                with sapien.ProfilerBlock("Process_info"):
                    self._process_ret(items)
                    if step_idx is None:
                        items["worker_indices"] = np.arange(self.num_envs, dtype=np.int32)[:, None]
                        items["is_truncated"] = np.zeros(self.num_envs, dtype=np.bool_)[:, None]
                        if self.rnn_states.memory is not None and self.save_hidden:
                            items["rnn_states"] = rnn_states
                            items["next_rnn_states"] = next_rnn_states
                    else:
                        items["worker_indices"] = step_idx[:, None]
                        items["is_truncated"] = np.zeros(len(step_idx), dtype=np.bool_)[:, None]
                        if self.rnn_states.memory is not None and self.save_hidden:
                            items["rnn_states"] = GDict(rnn_states).slice(step_idx, wrapper=False)
                            items["next_rnn_states"] = GDict(next_rnn_states).slice(step_idx, wrapper=False)
                # print(GDict(items).type)

                # Reset recurrent states before calculating the next actions
                for ii in range(len(items["episode_dones"])):
                    if items["episode_dones"][ii, 0]:
                        self.rnn_states.assign(items["worker_indices"][ii, 0], 0)

                if total < num - 1:
                    # The last one is not useful
                    with sapien.ProfilerBlock("Network"):
                        timer.since_last_check("overhead")
                        get_action_infos = get_actions(step_idx, with_states=True)
                        timer.since_last_check("agent")
                    with sapien.ProfilerBlock("Copy to numpy"):
                        items.update(self.vec_env.obs_next_obs_cpu(step_idx))
                    with sapien.ProfilerBlock("Copy on CPU"):
                        actions, rnn_states, next_rnn_states = GDict(get_action_infos).detach().to_numpy(wrapper=False)
                    timer.since_last_check("copy")
                # if 997 < (total + 1) <= 1002:
                #     print(total + 1, num, actions, items["episode_dones"])

                items = DictArray(items)
                total += len(items)
                tqdm_logger.update(len(items))
                num_forward_envs.append(len(items))

                timer.since_last_check("overhead")
                if replay is not None:
                    finished += replay.cache_trajectories(items, num - finished)
                timer.since_last_check("copy")
                """
                for ii in range(len(items)):
                    items_i = items.slice(ii, wrapper=False)  # Make a safe copy to replay buffer!
                    # print(GDict(items_i).type)
                    # exit(0)

                    i = items_i["worker_indices"][0]
                    trajs[i].append(items_i)

                    if items_i["episode_dones"][0]:
                        unfinished -= len(trajs[i])
                        if len(trajs[i]) + finished > num:
                            trajs[i] = trajs[i][: num - finished]

                        if replay is not None:
                            timer.since_last_check("overhead")
                            # print(DictArray.stack(trajs[i], axis=0).shape)
                            replay.push_batch(DictArray.stack(trajs[i], axis=0, wrapper=False))
                            timer.since_last_check("copy")

                        finished += len(trajs[i])
                        trajs[i] = []
                    """

                if total >= num * 0.8 and sync_timer.since_last_check() >= 1:
                    if num_done.get() >= world_size * 0.5:  # Use the trick in DD-PPO
                        break

            timer.since_last_check("overhead")
            if replay is not None:
                finished += replay.push_cached_trajectories(num - finished)
                assert finished == num, f"#collected samples {finished} need to be equal to num {num}!"
            timer.since_last_check("copy")

            """
            if unfinished > 0:
                for i in range(self.num_envs):
                    # print(i, len(trajs[i]), finished, num, unfinished)
                    if len(trajs[i]) > 0 and finished < num:
                        if len(trajs[i]) + finished > num:
                            trajs[i] = trajs[i][: num - finished]
                        trajs[i][-1]["is_truncated"] = true_array
                        traj_i = DictArray.stack(trajs[i], axis=0).to_two_dims(False)
                        finished += len(trajs[i])

                        if replay is not None:
                            timer.since_last_check("overhead")
                            replay.push_batch(traj_i)
                            timer.since_last_check("copy")
                del trajs
            """

            num_done.add(1)
            barrier()
            del num_done

            sim_time, agent_time, copy_time, overhead_time, overall_time = (
                timer["sim"],
                timer["agent"],
                timer["copy"],
                timer["overhead"],
                timer.since_start(),
            )

            # print(step_times)
            # exit(0)
            total_step_fps = 1 / np.concatenate(step_times)
            max_fps_per_iter = [1.0 / np.max(_) for _ in step_times]
            median_fps_per_iter = [1.0 / np.median(_) for _ in step_times]

            get_logger().info(
                f"Finish with {finished} samples, simulation time/FPS:{sim_time:.2f}/{finished / sim_time:.2f}, agent time/FPS:{agent_time:.2f}/{finished / agent_time:.2f}, overhead time:{overhead_time:.2f}, copy time:{copy_time:.2f}, overall time/FPS:{overall_time:.2f}/{finished / overall_time:.2f}, average num of forward envs: {np.mean(num_forward_envs)}."
            )

            """
            def log_simulation_time(task_name, numbers):
                get_logger().info(f"Simulation Time: {task_name} FPS:{np.mean(numbers):.2f}\u00B1{np.std(numbers):.2f} [{np.min(numbers):.2f}, {np.max(numbers):.2f}]")
            log_simulation_time("step", total_step_fps)
            log_simulation_time("batch", max_fps_per_iter)
            log_simulation_time("batch-median", median_fps_per_iter)
            """

            # with open("/data/github_code/ManiSkill2-dev/info.txt", "w") as f:
            #     f.write(f"Finish with {finished} samples, simulation time/FPS:{sim_time:.2f}/{finished / sim_time:.2f}, agent time/FPS:{agent_time:.2f}/{finished / agent_time:.2f}, overhead time:{other_time:.2f}, copy time:{copy_time:.2f}, overall time/FPS:{overall_time:.2f}/{finished / overall_time:.2f}, average num of forward envs: {np.mean(num_forward_envs)}.")
            # print(d_oh0, d_oh1, d_oh2, d_oh3)
            ret = replay.tail(num) if replay is not None else 0
            # print(ret["rewards"].reshape(-1, 1000).sum(-1))
            # exit(0)
            # print("RNN State", ret["rnn_states"].mean())
            # exit(0)
        else:
            assert num % self.num_envs == 0, f"{self.num_envs} % {num} != 0, some processes are idle, you are wasting memory!"
            ret = []
            for i in range(num // self.num_envs):
                action = to_np(get_actions())
                items = self.vec_env.step_dict(action)
                # print(action)
                # print("====>", action.mean().item())

                if items["episode_dones"].any():
                    worker_indices = items["worker_indices"][items["episode_dones"][:, 0], 0]
                    self.rnn_states.assign(worker_indices, 0)

                self._process_ret(items)
                items = GDict(items).to_numpy().copy(wrapper=False)
                ret.append(items)
            ret = DictArray.concat(ret, axis=0)  # .to_two_dim(wrapper=False)
            # print(GDict(ret).shape)
            if replay is not None:
                replay.push_batch(ret)
        return ret

    def close(self):
        self._is_closed = True
        self.vec_env.close()


@ROLLOUTS.register_module()
class NetworkRollout:
    def __init__(self, model, reward_only=False, use_cost=False, num_samples=1, **kwargs):
        self.reward_only = reward_only
        self.model = model
        self.num_envsum_models = self.model.num_heads
        self.num_envsum_samples = num_samples
        self.is_cost = -1 if use_cost else 1

    def reset(self, **kwargs):
        if hasattr(self.model, "reset"):
            self.model.reset()

    def random_action(self):
        raise NotImplementedError

    def step_states_actions(self, states, actions):
        """
        :param states: [N, ..] n different env states
        :param actions: [N, L, NA] n sequences of actions
        :return: rewards [N, L, 1]
        """
        assert self.reward_only
        batch_size = actions.shape[0]
        len_seq = actions.shape[1]
        assert states.shape[0] == actions.shape[0]
        import torch

        with torch.no_grad():
            device = self.model.device
            current_states = (
                DictArray(states)
                .to_torch(dtype="float32", device=device, non_blocking=True)
                .unsqueeze(1)
                .repeat(self.num_envsum_models, axis=1)
                .repeat(self.num_envsum_samples, axis=0, wrapper=False)
            )
            actions = (
                DictArray(actions).to_torch(dtype="float32", device=device, non_blocking=True).repeat(self.num_envsum_samples, axis=0, wrapper=False)
            )
            assert current_states.ndim == 3
            rewards = []
            # print(len_seq)
            for i in range(len_seq):
                recent_actions = actions[:, i : i + 1].repeat_interleave(self.num_envsum_models, dim=1)
                # print(recent_actions.shape)
                # print(current_states.mean(0).mean(0), recent_actions.mean(0).mean(0))
                # print(current_states, recent_actions)
                # print(current_states.shape, recent_actions.shape)
                next_obs, r, done = self.model(current_states, recent_actions)
                # print('NO', next_obs)
                # exit(0)
                # print(r.mean())
                # exit(0)
                assert r.ndim == 2 and done.ndim == 2
                current_states = next_obs
                rewards.append(r.mean(dim=1).detach())
            rewards = DictArray.stack(rewards, axis=1).to_numpy(wrapper=False)
            rewards[rewards != rewards] = -1e6

            # print(rewards.sum(-1).mean(), rewards.shape)
            # exit(0)

            rewards = rewards.reshape(batch_size, self.num_envsum_samples, len_seq).mean(1)
        return rewards[..., None]


@ROLLOUTS.register_module()
class BanditRollout:
    """
    A numpy-based rollout for bandit problems, which is much faster than the parallel rollout with multiple processes.
    """

    def __init__(self, env_cfg, **kwargs):
        self.logger = get_logger()
        self.vec_env = build_vec_env(env_cfg)
        self.vec_env.reset()
        x, value = self.vec_env.model.get_global_minimum(self.vec_env.model.d)
        self.logger.info(f"{x} {value}!")

    def __getattr__(self, name):
        return getattr(self.vec_env, name)

    def _get_reward(self, x):
        return self.vec_env.step(x)[1]

    def reset(self, **kwargs):
        return np.zeros(1)

    def random_action(self):
        raise NotImplemented

    def step_states_actions(self, states, actions):
        # states: [N, S]
        # actions: [N, 1, NA]
        assert actions.shape[1] == 1 and actions.ndim == 3
        actions = actions[:, 0]
        reward = np.apply_along_axis(self._get_reward, 1, actions) * self.is_cost
        return reward[:, None, None]
