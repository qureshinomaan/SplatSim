import os
import os.path as osp
import shutil

import imageio
import numpy as np
from h5py import File
from pyrl.utils.data import (
    DictArray,
    GDict,
    dict_to_str,
    is_str,
    num_to_str,
    to_np,
    dict_to_str,
    to_item,
)
from pyrl.utils.file import dump, load
from pyrl.utils.meta import get_logger, get_logger_name, get_total_memory, get_meta_info

from .builder import EVALUATIONS
from .env_utils import build_vec_env


def save_eval_statistics(folder, lengths, rewards, finishes, logger=None):
    if logger is None:
        logger = get_logger()
    logger.info(
        f"Num of trails: {len(lengths):.2f}, "
        f"Length: {np.mean(lengths):.2f}\u00B1{np.std(lengths):.2f}, "
        f"Reward: {np.mean(rewards):.2f}\u00B1{np.std(rewards):.2f}, "
        f"Success or Early Stop Rate: {np.mean(finishes):.2f}\u00B1{np.std(finishes):.2f}"
    )
    if folder is not None:
        table = [["length", "reward", "finish"]]
        table += [[num_to_str(__, precision=2) for __ in _] for _ in zip(lengths, rewards, finishes)]
        dump(table, osp.join(folder, "statistics.csv"))


def log_mem_info(logger):
    import torch
    from pyrl.utils.torch import get_cuda_info

    print_dict = {}
    print_dict["memory"] = get_total_memory("G", False)
    print_dict.update(get_cuda_info(device=torch.cuda.current_device(), number_only=False))
    print_info = dict_to_str(print_dict)
    logger.info(f"Resource usage: {print_info}")


@EVALUATIONS.register_module()
class Evaluation:
    def __init__(self, env_cfg=None, num_procs=1, seed=None, **kwargs):
        seed = seed if seed is not None else np.random.randint(int(1e9))
        self.n = num_procs
        self.vec_env = build_vec_env(env_cfg, num_procs, **kwargs, seed=seed)
        # print(self.vec_env.reset())
        # exit(0)
        self.vec_env.reset()
        self.use_hidden_state = kwargs.get("use_hidden_state", None)

        self.num_envs = self.vec_env.num_envs
        self.all_env_indices = np.arange(self.num_envs, dtype=np.int32)
        self.log_every_episode = kwargs.get("log_every_episode", True)
        self.log_every_step = kwargs.get("log_every_step", False)

        self.save_traj = kwargs.get("save_traj", False)
        self.save_video = kwargs.get("save_video", False)
        self.only_save_success_traj = kwargs.get("only_save_success_traj", False)

        self.sample_mode = kwargs.get("sample_mode", "eval")
        self.eval_levels = kwargs.get("eval_levels", None)

        self.video_format = kwargs.get("video_format", "mp4")
        self.video_fps = kwargs.get("fps", 20)

        logger_name = get_logger_name()
        self.logger = get_logger("Evaluation-" + logger_name, with_stream=True)
        self.logger.info(f"Evaluation environments have seed in [{seed}, {seed + num_procs})!")

        if self.eval_levels is not None and is_str(self.eval_levels):
            is_csv = eval_levels.split(".")[-1] == "csv"
            eval_levels = load(self.eval_levels)
            self.eval_levels = eval_levels[0] if is_csv else eval_levels
        if self.eval_levels is not None:
            self.logger.info(f"During evaluation, levels are selected from an existing list with length {len(self.eval_levels)}")

    def reset_pi(self, pi, idx, rnn_states):
        rnn_states.assign(idx, 0)

        """ When we run CEM, we need the level of the rollout env to match the level of test env.  """
        if not hasattr(pi, "reset"):
            return
        reset_kwargs = {}
        if hasattr(self.vec_env.vec_env.single_env, "level"):
            reset_kwargs["level"] = self.vec_env.level
        pi.reset(**reset_kwargs)  # For CEM and PETS-like model-based method.

    def run(self, pi, num=1, work_dir=None, **kwargs):
        eval_levels, h5_file = None, None
        if self.eval_levels is not None:
            num = min(len(self.eval_levels), num)
            random_start = np.random.randint(len(self.eval_levels) - num + 1)
            eval_levels = self.eval_levels[random_start : random_start + num]
        self.logger.info(f"We will evaluate over {num} episodes!")

        if osp.exists(work_dir):
            self.logger.warning(f"We will overwrite this folder {work_dir} during evaluation!")
            shutil.rmtree(work_dir, ignore_errors=True)
        os.makedirs(work_dir, exist_ok=True)

        if self.save_video:
            video_dir = osp.join(work_dir, "videos")
            self.logger.info(f"Save videos to {video_dir}.")
            os.makedirs(video_dir, exist_ok=True)

        if self.save_traj:
            trajectory_path = osp.join(work_dir, "trajectory.h5")
            if osp.exists(trajectory_path):
                self.logger.warning(f"We will overwrite this file {trajectory_path} during evaluation!")
            h5_file = File(trajectory_path, "w")
            self.logger.info(f"Save trajectory at {trajectory_path}.")
            group = h5_file.create_group(f"meta")
            GDict(get_meta_info()).to_hdf5(group)

        import torch

        num_finished, num_start, num_envs = 0, 0, min(self.num_envs, num)
        traj_idx = np.arange(num_envs, dtype=np.int32)
        video_writers, episodes = None, None

        if eval_levels is not None and hasattr(self.vec_env.vec_env.single_env, "level"):
            obs_all = self.vec_env.reset(level=eval_levels[:num_envs], idx=np.arange(num_envs))
        else:
            obs_all = self.vec_env.reset(idx=np.arange(num_envs))
        rnn_states_all, obs_all = GDict(None), DictArray(obs_all).copy()
        self.reset_pi(pi, self.all_env_indices, rnn_states_all)

        if self.save_video:
            video_writers = []
            imgs = self.vec_env.render(mode="rgb_array", idx=np.arange(num_envs))
            for i in range(num_envs):
                video_file = osp.join(video_dir, f"{i}.{self.video_format}")
                video_writers.append(imageio.get_writer(video_file, fps=self.video_fps, quality=5, macro_block_size=4))
        episodes = [[] for i in range(num_envs)]
        num_start = num_envs
        episode_lens, episode_rewards, episode_finishes = (
            np.zeros(num, dtype=np.int32),
            np.zeros(num, dtype=np.float32),
            np.zeros(num, dtype=np.bool_),
        )
        action = self.vec_env.action_space.sample() * 0

        total_steps = 0
        while num_finished < num:
            idx = np.nonzero(traj_idx >= 0)[0]
            total_steps += 1
            # print(total_steps)

            obs = obs_all.slice(idx, wrapper=False)
            if self.use_hidden_state:
                obs = self.vec_env.get_state()
                if idx is not None:
                    obs = obs[idx]
            with torch.no_grad():
                rnn_states = rnn_states_all.slice(idx, wrapper=False)
                with pi.no_sync(mode="actor"):
                    action, rnn_states = pi(obs, mode=self.sample_mode, rnn_states=rnn_states, prev_actions=action, rnn_mode="with_states")
                    action = to_np(action)

                rnn_states_all.assign(idx, rnn_states)

            if self.save_traj:
                env_state = self.vec_env.get_env_state()
            infos = self.vec_env.step_dict(action, idx=idx, restart=False)

            if self.save_traj:
                next_env_state = self.vec_env.get_env_state()
                for key in next_env_state:
                    env_state["next_" + key] = next_env_state[key]
                infos.update(env_state)

            infos = GDict(infos).to_array().to_two_dims()
            episode_dones = infos["episode_dones"]
            obs_all.assign(idx, infos["next_obs"])

            if self.log_every_step and self.num_envs == 1:
                reward, done, info, episode_done = GDict([infos["rewards"], infos["dones"], infos["infos"], infos["episode_dones"]]).item(
                    wrapper=False
                )
                assert isinstance(info, dict)
                info_str = dict_to_str({key.split("/")[-1]: val for key, val in info.items()})
                self.logger.info(
                    f"Episode {traj_idx[0]}, Step {episode_lens[traj_idx[0]]}: Reward: {reward:.3f}, Early Stop or Finish: {done}, Info: {info_str}"
                )
            if self.save_video:
                imgs = self.vec_env.render(mode="rgb_array", idx=idx)
                for j, i in enumerate(idx):
                    video_writers[i].append_data(imgs[j])
            reset_idx = []
            reset_levels = []
            for j, i in enumerate(idx):
                episodes[i].append(GDict(infos).slice(j, wrapper=False))
                episode_lens[traj_idx[i]] += 1
                episode_rewards[traj_idx[i]] += to_item(infos["rewards"][j])
                if to_item(episode_dones[j]):
                    num_finished += 1
                    if self.save_video:
                        video_writers[i].close()

                    episodes_i = GDict.stack(episodes[i], 0)
                    episodes[i] = []

                    reward = episodes_i["rewards"].sum()
                    done = to_item(infos["dones"][j])
                    episode_finishes[traj_idx[i]] = done

                    if self.log_every_episode:
                        self.logger.info(
                            f"Episode {traj_idx[i]} ends: Length {episode_lens[traj_idx[i]]}, Reward: {reward}, Early Stop or Finish: {done}!"
                        )
                        log_mem_info(self.logger)

                    if self.save_traj and (not self.only_save_success_traj or done):
                        group = h5_file.create_group(f"traj_{traj_idx[i]}")
                        GDict(episodes_i.memory).to_hdf5(group)

                    if num_start < num:
                        traj_idx[i] = num_start
                        reset_idx.append(i)
                        if eval_levels is not None:
                            reset_levels.append(eval_levels[num_start])
                        num_start += 1
                    else:
                        traj_idx[i] = -1
            reset_idx = np.array(reset_idx, dtype=np.int32)
            if len(reset_idx) > 0:
                if eval_levels is not None:
                    reset_levels = np.array(reset_levels, dtype=np.int64)
                    obs = self.vec_env.reset(level=reset_levels, idx=reset_idx)
                else:
                    obs = self.vec_env.reset(idx=reset_idx)
                obs_all.assign(reset_idx, obs)
                self.reset_pi(pi, reset_idx, rnn_states_all)

                if self.save_video:
                    imgs = self.vec_env.render(mode="rgb_array", idx=reset_idx)
                    for j, i in enumerate(reset_idx):
                        video_file = osp.join(video_dir, f"{traj_idx[i]}.{self.video_format}")
                        video_writers[i] = imageio.get_writer(video_file, fps=self.video_fps, quality=5, macro_block_size=8)
        if h5_file is not None:
            h5_file.close()
        return episode_lens, episode_rewards, episode_finishes

    def close(self):
        self.vec_env.close()
