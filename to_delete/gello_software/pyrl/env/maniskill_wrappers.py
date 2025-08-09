import cv2, numpy as np

from collections import deque, defaultdict
from gym.core import ObservationWrapper

from pyrl.utils.data import DictArray, GDict, deepcopy, is_num, to_np, to_array
from pyrl.utils.meta import Registry, build_from_cfg

from .wrappers import ExtendedWrapper
from .observation_process import (
    pcd_base,
    pcd_uniform_downsample,
    pcd_voxel_downsample,
)

WRAPPERS = Registry("wrappers of gym environments")


def depth_to_pcd(intrinsic, depth):
    if depth.ndim == 3:
        depth = depth.squeeze()
    z = depth
    v, u = np.indices(z.shape)
    uv1 = np.stack([u + 0.5, v + 0.5, np.ones_like(z)], axis=-1)
    return uv1 @ np.linalg.inv(intrinsic).T * z[..., np.newaxis]


class ManiSkillBatchWrapper(ExtendedWrapper):
    """
    Apply GPU-based pointcloud downsampling over observation of vectorized environments
    It should have the same api as vec_env
    """

    def __init__(self, vec_env, device, downsample="maniskill"):
        super(ManiSkillBatchWrapper, self).__init__(vec_env)

        self.downsample, self.device = downsample, device
        all_env_indices, example_env = np.arange(self.num_envs), self.env.single_env
        self.VISUAL_OBS_MODES, self.PCD_OBS_MODES, self.obs_mode = example_env.VISUAL_OBS_MODES, example_env.PCD_OBS_MODES, example_env.obs_mode
        self.is_discrete, self.reward_scale, self.is_cost = example_env.is_discrete, example_env.reward_scale, example_env.is_cost

        if self.obs_mode in self.VISUAL_OBS_MODES:
            # Avid using torch in state-based MPC
            if hasattr(example_env, "seg_labels"):
                self.seg_labels = self.get_attr("seg_labels", idx=all_env_indices)
                self.num_valid_seg_labels = self.get_attr("num_valid_seg_labels", idx=all_env_indices)
            if hasattr(example_env, "seg_indices"):
                self.seg_indices = example_env.seg_indices
        from .env_utils import convert_observation_to_space

        self.observation_space = convert_observation_to_space(self.env.reset(idx=all_env_indices))

    def reset(self, idx=None, *args, **kwargs):
        obs = self.env.reset(idx=idx, *args, **kwargs)
        if self.obs_mode in self.VISUAL_OBS_MODES:
            self.seg_labels[idx] = self.get_attr("seg_labels", idx=idx)
            self.num_valid_seg_labels[idx] = self.get_attr("num_valid_seg_labels", idx=idx)
        return self.observation(obs)

    def observation(self, obs):
        if self.obs_mode not in self.VISUAL_OBS_MODES:
            return obs

        assert isinstance(obs, dict), f"Visual observation should have type {type(obs)}!"

        import torch
        from pyrl.utils.data import to_torch

        with torch.no_grad():
            obs_ret = obs.copy()  # shallow copy of a dict
            for key in ["Color", "Position", "Segmentation"]:
                if key in obs_ret:
                    obs_ret[key] = to_torch(obs_ret[key], device=self.device)
            if self.obs_mode in self.PCD_OBS_MODES:
                # Fuse pointclouds from different images to one pointcloud!
                camera_matrices = to_torch(obs_ret.pop("camera_matrices"), device=self.device, dtype="float32")
                obs_ret["xyz"] = (
                    torch.einsum(
                        "bkij, bknmj->bknmi",
                        camera_matrices[..., :3, :3],
                        obs_ret["Position"][..., :3],
                    )
                    + camera_matrices[..., None, None, :3, 3]
                ).reshape(self.num_envs, -1, 3)
                for key in ["Color", "Segmentation"]:
                    if key in obs_ret:
                        obs_ret[key] = obs_ret[key].view(self.num_envs, -1, 4)

            if self.obs_mode in self.VISUAL_OBS_MODES:
                if "Color" in obs_ret:
                    obs_ret["rgb"] = obs_ret["Color"][..., :3]

                if self.obs_mode in ["depth", "rgbd"]:
                    obs_ret["depth"] = obs_ret["Position"][..., 3:]

                if "Segmentation" in obs_ret and hasattr(self, "seg_labels") and hasattr(self, "seg_indices"):
                    seg = obs_ret["Segmentation"]
                    final_seg = []
                    for i, seg_idx in enumerate(self.seg_indices):
                        max_num = self.num_valid_seg_labels[:, i].max()
                        if seg.ndim == 5:
                            seg_label = to_torch(self.seg_labels[..., None, None, None, i, :max_num], device=self.device)
                        else:
                            seg_label = to_torch(self.seg_labels[..., None, i, :max_num], device=self.device)
                        final_seg.append((seg[..., seg_idx : seg_idx + 1] == seg_label).any(-1))
                    obs_ret["seg"] = torch.stack(final_seg, dim=-1)
                for key in ["Color", "Position", "Segmentation", "camera_matrices"]:
                    obs_ret.pop(key, None)

            if self.obs_mode in self.PCD_OBS_MODES:
                from pyrl.utils.cpp_ops.ops_3d.pcd_process import downsample_pcd

                if self.downsample == "maniskill":
                    index, mask = downsample_pcd(
                        obs_ret["xyz"], mode="maniskill", min_z=1e-4, num=1200, num_min=50, num_fg=800, seg=obs_ret["seg"]
                    )  # [B, M]
                else:
                    index, mask = downsample_pcd(obs_ret["xyz"], mode="uniform", min_z=1e-4, num=1200)  # [B, M]
                target_size = list(index.shape)
                for key in ["rgb", "seg", "xyz"]:
                    if key in obs_ret:
                        index_key = index[..., None].expand(target_size + [int(obs_ret[key].shape[-1])])
                        obs_ret[key] = torch.gather(obs_ret[key], dim=1, index=index_key)
            if self.obs_mode in self.IMAGE_OBS_MODES:
                for key in ["rgb", "depth"]:
                    obs_ret[key] = obs_ret[key].transpose(-3, -1)  # [..., H, W, C] -> [..., C, H, W]
        return obs_ret

    def step(self, actions, idx=None):
        obs, rewards, dones, infos = self.env.step(actions, idx)
        return self.observation(obs), rewards, dones, infos

    def render(self, mode="rgb_array", idx=None):
        img = self.env.render(mode, idx=idx)
        img = to_np(img[:, 0, ..., :3])
        if img.dtype in [np.float32, np.float64]:
            img = np.clip(img, a_min=0, a_max=1) * 255
        img = img.astype(np.uint8)
        return img


class ManiSkillObsWrapper(ExtendedWrapper):
    def __init__(self, env, stack_frame=1, n_points=1200, min_pts=50, max_size=84, fg_pts=500):
        """
        Stack k last frames for point clouds or rgbd and remap the rendering configs
        """
        super(ManiSkillObsWrapper, self).__init__(env)
        self.stack_frame = stack_frame
        self.buffered_data = {}
        self.n_points, self.min_pts, self.fg_pts = n_points, min_pts, fg_pts
        self.max_size = max_size
        self.IMAGE_OBS_MODE = ["rgb", "rgbd", "depth"]
        self.PCD_OBS_MODE = ["pointcloud"]

    def get_state(self):
        return self.env.get_state(True)

    def observation(self, observation):
        if self.obs_mode == "state":
            return np.concatenate([observation, self.prev_action]) if self.add_prev_action else observation
        else:
            state = observation["state"] if "state" in observation else observation["agent"]
            target_info = observation.pop("target_info", None)
            if target_info is not None:
                state = np.concatenate([state, target_info])
            assert len(observation) == 2, f"{observation.keys()}"
            observation = observation[self.obs_mode]

            obs = {}
            from pyrl.utils.data import float_to_int, as_dtype, GDict

            if "rgb" in observation:
                observation["rgb"] = float_to_int(observation["rgb"], dtype="uint8")

            if self.obs_mode in self.IMAGE_OBS_MODE:
                for key in observation:
                    obs[key] = observation[key].transpose(2, 0, 1)

            elif self.obs_mode in self.PCD_OBS_MODE:
                obs = pcd_base(observation, self.n_points, self.min_pts)
                for key in obs:
                    obs[key] = obs[key].transpose(1, 0)
            obs["agent"] = state
            return obs

    def step(self, action):
        next_obs, reward, done, info = super(ManiSkillObsWrapper, self).step(action)
        return self.observation(next_obs), reward, done, info

    def reset(self, *args, **kwargs):
        # Change the ManiSkill level to seed as the standard interface in gym
        return self.observation(self.env.reset(*args, **kwargs))

    def get_obs(self):
        return self.observation(self.env.get_obs())

    def set_state(self, *args, **kwargs):
        return self.observation(self.env.set_state(*args, **kwargs))

    def render(self, mode="human", *args, **kwargs):
        if mode == "human":
            self.env.render(mode, *args, **kwargs)
            return

        if mode in ["rgb_array", "color_image"]:
            img = self.env.render(mode="color_image", *args, **kwargs)
        else:
            img = self.env.render(mode=mode, *args, **kwargs)
        if isinstance(img, dict):
            if "world" in img:
                img = img["world"]
            elif "main" in img:
                img = img["main"]
            else:
                print(img.keys())
                exit(0)
        if isinstance(img, dict):
            img = img["rgb"]
        if img.ndim == 4:
            assert img.shape[0] == 1
            img = img[0]
        if img.dtype in [np.float32, np.float64]:
            img = np.clip(img, a_min=0, a_max=1) * 255
        img = img[..., :3]
        img = img.astype(np.uint8)
        return img
