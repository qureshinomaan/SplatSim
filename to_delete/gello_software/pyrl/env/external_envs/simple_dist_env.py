import gym, numpy as np, os
from gym import spaces, register
from gym.core import Env
from gym.wrappers import TimeLimit
from collections import defaultdict


class DistEnv(Env):
    def __init__(self, image_size=20, n=2, obs_mode="state", max_depth=2, pad=2, ego_mode=False, box_size=1, max_dist=5, min_dist=2):
        # Num of
        if isinstance(image_size, list):
            image_size = image_size[0]
        self.img_size = image_size
        self.n = n
        self.obs_mode = obs_mode
        self.max_depth = max_depth
        self.ego_mode = ego_mode
        self.pad = pad
        self.box_size = box_size
        self.max_dist = max_dist
        self.min_dist = min_dist
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,))
        self.intrinsic = np.array([[20, 0, (self.img_size - 1) / 2.0], [0, 20, (self.img_size - 1) / 2.0], [0, 0, 1]])
        self.np_random = np.random.RandomState()
        self._step = 0

    @property
    def inv_intrinsic(self):
        intrinsic = self.intrinsic
        fx, fy, S, cx, cy = intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 1], intrinsic[0, 2], intrinsic[1, 2]
        inv_intrinsic = np.array([[1 / fx, -S / (fx * fy), (S * cy - cx * fy) / (fx * fy)], [0, 1 / fy, -cy / fy], [0, 0, 1]])
        return inv_intrinsic

    def get_xyz(self, depth=None):
        if depth is None:
            uv1 = np.array(
                [
                    [self.source[1] + 0.5, self.source[0] + 0.5, 1],
                    [self.target[1] + 0.5, self.target[0] + 0.5, 1],
                ]
            )
            depth = np.array([self.source_depth, self.target_depth])
        else:
            v, u = np.indices(depth.shape)
            uv1 = np.stack([u + 0.5, v + 0.5, np.ones_like(depth)], axis=-1)

        xyz = uv1 @ self.inv_intrinsic.T * depth[..., None]
        return xyz

    def reset(self):
        while True:
            source = self.np_random.randint(self.img_size - self.pad, size=2) + self.pad
            target = self.np_random.randint(self.img_size - self.pad, size=2) + self.pad
            self.source = source
            self.target = target
            self.source_depth, self.target_depth = self.np_random.rand(2) * self.max_depth
            self.source_xyz, self.target_xyz = self.get_xyz()
            if self.min_dist < np.linalg.norm(self.source_xyz - self.target_xyz) < self.max_dist:
                break
        self._step = 0
        # print(self.source_xyz, self.target_xyz, self.source, self.target, self.source_depth, self.target_depth)
        return self.get_obs()

    def seed(self, seed):
        self.action_space.seed(seed)
        self.np_random.seed(seed)

    def get_obs(self, obs_mode=None):
        obs_mode = self.obs_mode if obs_mode is None else obs_mode
        if self.obs_mode == "rgb":
            rgb = np.zeros([3, self.img_size, self.img_size], dtype=np.uint8)
            rgb[:, self.source[0], self.source[1]] = [255, 0, 0]
            rgb[:, self.target[0], self.target[1]] = [0, 0, 255]
            return {"rgb": rgb}
        elif self.obs_mode in ["rgbd", "pointcloud", "xyz-img"]:
            rgb = np.zeros([3, self.img_size, self.img_size], dtype=np.uint8)
            depth = np.zeros([1, self.img_size, self.img_size], dtype=np.float32)

            for i in range(-(self.box_size // 2), self.box_size // 2 + 1):
                for j in range(-(self.box_size // 2), self.box_size // 2 + 1):
                    if 0 <= self.source[0] + i < self.img_size and 0 <= self.source[1] + j < self.img_size:
                        rgb[:, self.source[0] + i, self.source[1] + j] = [255, 0, 0]
                        depth[:, self.source[0] + i, self.source[1] + j] = self.source_depth
                    if 0 <= self.target[0] + i < self.img_size and 0 <= self.target[1] + j < self.img_size:
                        rgb[:, self.target[0] + i, self.target[1] + j] = [0, 0, 255]
                        depth[:, self.target[0] + i, self.target[1] + j] = self.target_depth
                    # print(self.source_depth, self.target_depth, i, j, self.source, self.target)
            if self.obs_mode == "rgbd":
                return {"rgb": rgb, "depth": np.float32(depth / self.max_depth)}
            elif self.obs_mode == "xyz-img":
                return {"rgb": rgb, "xyz": self.get_xyz(depth[0]).transpose(2, 0, 1)}
            else:
                # print(depth.shape, depth.min(), depth.max())

                """
                sign = (depth[0] > 0).reshape(-1)
                assert np.sum(sign) >= 1

                xyz = self.get_xyz(depth[0]).reshape(-1, 3)
                rgb = rgb.reshape(3, -1).T

                xyz = xyz[sign, :]
                rgb = rgb[sign, :]
                if rgb[0, 0] == 255:
                    source_idx = 0
                else:
                    source_idx = 1
                target_idx = 1 - source_idx
                if np.sum(sign) == 1:
                    xyz = np.concatenate([xyz, xyz], axis=0)
                    rgb = np.concatenate(
                        [rgb, (np.array([0, 0, 255]) if source_idx == 0 else np.array([255, 0]))[None]],
                        axis=0,
                    )

                assert rgb.shape[0] == 2, f"{rgb.shape, np.sum(sign)}"
                """
                source_idx, target_idx = 0, 1
                xyz = self.get_xyz()
                if self.ego_mode:
                    xyz[target_idx] -= xyz[source_idx]
                    return {"xyz": np.float32(xyz).T, "rgb": np.array([[255, 0, 0], [0, 0, 255]], np.uint8).T}
                else:
                    return {"xyz": np.float32(xyz).T, "rgb": np.array([[255, 0, 0], [0, 0, 255]], np.uint8).T}
        elif self.obs_mode == "state":
            return np.float32(np.concatenate([self.source, self.target]))

    def render(self):
        return self.get_obs("rgb")

    def set_obs_mode(self, obs_mode):
        self.obs_mode = obs_mode

    def step(self, action):
        assert self._step == 0
        action = action * self.max_dist
        r = np.linalg.norm(self.source_xyz + action - self.target_xyz)
        xyz = (self.source_xyz + action) @ self.intrinsic.T
        self.source_depth = max(xyz[-1], 1e-3)
        self.source = np.int32(xyz[:2] / xyz[-1])
        self.source = np.clip(self.source, 0, self.img_size - 1)
        return self.get_obs(), -r, True, {}


register(
    id="reacher3d_easy-v0",
    entry_point="pyrl.env.external_envs.simple_dist_env:DistEnv",
    kwargs=dict(),
)


# register(
#     id="Dist3dSingle-v0",
#     entry_point="pyrl.env.external_envs.simple_dist_env:DistEnvSingle",
#     kwargs=dict(),
# )


if __name__ == "__main__":
    pass
