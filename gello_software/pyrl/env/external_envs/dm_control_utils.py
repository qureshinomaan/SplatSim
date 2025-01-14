import gym, numpy as np, os
from gym import spaces, register
from gym.core import Env
from gym.wrappers import TimeLimit
from collections import defaultdict
import dm_control


REPRESENTED_ENVS = [
    #   Easy ones
    "dmc_ball_in_cup_catch-v0",
    "dmc_cartpole_swingup-v0",
    "dmc_reacher_easy-v0",
    "dmc_finger_spin-v0",
    "dmc_walker_walk-v0",
    "dmc_cheetah_run-v0",
    #   Harder ones
    "dmc_quadruped_run-v0",
    "dmc_acrobot_swingup-v0",
    "dmc_finger_turn_hard-v0",
    "dmc_hopper_hop-v0",
    "dmc_reacher_hard-v0",
    "dmc_walker_run-v0",
    "dmc_humanoid_stand-v0",
    "dmc_humanoid_walk-v0",
    "dmc_humanoid_run-v0",
    #   Vert hard
    "dmc_dog_walk-v0",
    "dmc_dog_run-v0",
]


DEFAULT_ACTION_REPEAT = defaultdict(lambda: 4)
DEFAULT_ACTION_REPEAT.update({"humanoid": 2, "dog": 2, "walker": 2, "finger": 2, "cartpole": 4, "reacher3d": 1})  # The same setting as DrQ & SVEA

DEFAULT_DEPTH_FILTER = defaultdict(lambda: 5)
DEFAULT_DEPTH_FILTER.update({"acrobot": 10, "dog": 10, "humanoid": 8, "reacher3d": 20})

DEFAULT_GROUND_EPS = defaultdict(lambda: 8e-3)
DEFAULT_GROUND_EPS.update(
    {
        "acrobot": 0.01,
        "dog": 0.01,
        "humanoid": 0.02,
        "cartpole": 0.01,
        "acrobot": 0.02,
        "dog": 0.02,
        "reacher3d": 0.1,
    }
)


DEFAULT_NUM_BODY = {
    "ball_in_cup": 128,
    "cartpole": 256,
    "reacher": 256,
    "finger": 384,
    "walker": 384,
    "cheetah": 256,
    "quadruped": 384,
    "acrobot": 128,
    "hopper": 256,
    "humanoid": 384,
    "dog": 384,
    "reacher3d": 128,
}


def build_dmc_env(
    domain,
    task,
    obs_mode="state",
    image_size=(84, 84),
    camera_id=None,
    episode_length=1000,
    frame_skip=None,
    max_depth=None,
    n_points=None,
    num_ground=None,
    ground_eps=None,
    is_distraction=False,
    **kwargs,
):
    if obs_mode != "state":
        assert kwargs.get("visualize_reward", False) == False, "cannot use visualize reward when learning from pixels"
    if not is_distraction:
        from dm_control import suite
    else:
        from distracting_control import suite

    if frame_skip is None:
        frame_skip = DEFAULT_ACTION_REPEAT[domain]
    if max_depth is None:
        max_depth = DEFAULT_DEPTH_FILTER[domain]
    if ground_eps is None:
        ground_eps = DEFAULT_GROUND_EPS[domain]
    if n_points is None:
        if num_ground is None:
            n_points = int(DEFAULT_NUM_BODY.get(domain, 384) * 4 / 3)
            num_ground = int(n_points // 4)
        else:
            if num_ground is None:
                num_ground = 0
            n_points = int(DEFAULT_NUM_BODY.get(domain, 384)) + num_ground

    if num_ground is None:
        num_ground = 0

    print(n_points)

    if camera_id is None:
        camera_id = 2 if domain == "quadruped" else 0
    env = suite.load(domain, task, **kwargs)
    env = DMCEnv(
        env,
        obs_mode=obs_mode,
        image_size=image_size,
        n_points=n_points,
        frame_skip=frame_skip,
        camera_id=camera_id,
        max_depth=max_depth,
        num_ground=num_ground,
        ground_eps=ground_eps,
    )
    env.domain = domain
    env.task = task

    max_episode_steps = (episode_length + frame_skip - 1) // frame_skip
    return TimeLimit(env, max_episode_steps=max_episode_steps)


def register_dmc_envs():
    from dm_control import suite

    for domain, task in suite.ALL_TASKS:
        env_id = "dmc_%s_%s-v0" % (domain, task)
        try:
            register(
                id=env_id,
                entry_point="pyrl.env.external_envs.dm_control_utils:build_dmc_env",
                kwargs=dict(domain=domain, task=task),
            )
        except:
            pass

        env_id = "distract_dmc_%s_%s-v0" % (domain, task)
        try:
            register(
                id=env_id,
                entry_point="pyrl.env.external_envs.dm_control_utils:build_dmc_env",
                kwargs=dict(domain=domain, task=task, is_distraction=True),
            )
        except:
            pass


def _flatten_obs(obs):
    obs_pieces = []
    for v in obs.values():
        flat = np.array([v]) if np.isscalar(v) else v.ravel()
        obs_pieces.append(flat)
    return np.concatenate(obs_pieces, axis=0).astype(np.float32)


class DMCEnv(Env):
    def __init__(
        self,
        env,
        obs_mode="state",
        image_size=(84, 84),  # height, width
        frame_skip=4,
        max_depth=5,
        n_points=512,
        num_ground=100,
        ground_eps=8e-3,
        camera_id=0,
        z_to_world=True,
        fix_base_z=None,
        use_done=False,
    ):
        super(DMCEnv, self).__init__()
        # assert camera_id >= 0, "camera_id must be >= 0"
        assert obs_mode in ["xyz", "pointcloud", "xyz-img", "xyz-rgb-img", "rgb", "depth", "state", "rgbd"]
        self.env, self.obs_mode, self.frame_skip = env, obs_mode, frame_skip
        self.image_size, self.n_points, self.camera_id, self.max_depth = image_size, n_points, camera_id, max_depth

        if hasattr(env, "action_spec"):
            self.min_action, self.max_action = np.float32(env.action_spec().minimum), np.float32(env.action_spec().maximum)
        else:
            action_space = self.env.action_space
            self.min_action = np.float32(action_space.low)
            self.max_action = np.float32(action_space.high)

        self.action_space = spaces.Box(-np.ones_like(self.min_action), np.ones_like(self.min_action), dtype=np.float32)
        self.image_size = np.array(self.image_size)
        # self.channels_first = channels_first
        self.num_ground = num_ground
        self.ground_eps = ground_eps
        self.z_to_world = z_to_world
        self.fix_base_z = fix_base_z
        self.use_done = use_done
        self._internal_step = 0

    def __getattr__(self, name):
        return getattr(self.env, name)

    def set_obs_mode(self, obs_mode):
        self.obs_mode = obs_mode

    @property
    def np_random(self):
        if hasattr(self.env, "task"):
            return self.env.task._random
        elif hasattr(self.env, "np_random"):
            return self.env.np_random
        else:
            from IPython import embed

            print("Let's find random generator")
            embed()

    @property
    def physics(self):
        if hasattr(self.env, "physics"):
            return self.env.physics
        elif hasattr(self.env, "sim"):
            return self.env.sim
        else:
            from IPython import embed

            print("Please find mujoco simulator")
            embed()

    @property
    def intrinsic(self):
        """
        K = [[fx, S, cx], [0, fy, cy], [0, 0, 1]]
        """
        fov = self.physics.model.cam_fovy[self.camera_id]
        # Focal transformation matrix (3x4).
        # https://github.com/openai/mujoco-py/issues/271
        focal_scaling = 0.5 * self.image_size[1] / np.tan(fov * np.pi / 360)
        focal = np.diag([focal_scaling, focal_scaling, 1.0])
        # Image matrix (3x3).
        image = np.eye(3)
        image[:2, 2] = (self.image_size - 1) / 2.0
        return image @ focal

    @property
    def inv_intrinsic(self):
        intrinsic = self.intrinsic
        fx, fy, S, cx, cy = intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 1], intrinsic[0, 2], intrinsic[1, 2]
        inv_intrinsic = np.array([[1 / fx, -S / (fx * fy), (S * cy - cx * fy) / (fx * fy)], [0, 1 / fy, -cy / fy], [0, 0, 1]])
        return inv_intrinsic

    def get_cam_pose(self):
        cam_pos = self.physics.data.cam_xpos[self.camera_id]
        cam_to_body_rot = np.array(self.physics.model.cam_mat0[self.camera_id]).reshape(3, 3)
        body_to_world_rot = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        cam_to_world_rot = cam_to_body_rot @ body_to_world_rot
        return cam_pos, cam_to_world_rot

    @property
    def extrinsic(self):
        """
        The extrinsic matrix is a transformation matrix from the world coordinate system to the camera coordinate system.
        """
        pos, rot = self.get_cam_pose()
        # Translation matrix (4x4). Rotation matrix (4x4).
        translation, rotation = np.eye(4), np.eye(4)
        translation[:3, 3] = -pos
        rotation[:3, :3] = rot.T
        return rotation @ translation

    @property
    def inv_extrinsic(self):
        ret = np.eye(4)
        pos, rot = self.get_cam_pose()
        ret[:3, 3] = pos
        ret[:3, :3] = rot
        return ret

    def get_xyz(self, depth, world_frame=False):
        """
        Modified from https://github.com/mattcorsaro1/mj_pc/blob/main/mj_point_clouds.py
        """
        v, u = np.indices(depth.shape)
        uv1 = np.stack([u + 0.5, v + 0.5, np.ones_like(depth)], axis=-1)
        xyz = uv1 @ self.inv_intrinsic.T * depth[..., None]
        if world_frame:
            from pyrl.utils.data import to_gc, to_nc

            return to_nc(to_gc(xyz) @ self.inv_extrinsic.T)
        else:
            return xyz

    def _get_vis_obs(self, with_depth=False):
        if type(self) == DMCEnv:
            from dm_control.mujoco.engine import Camera

            camera = Camera(physics=self.physics, width=self.image_size[0], height=self.image_size[1], camera_id=self.camera_id)
            rgb = camera.render(depth=False)

            if with_depth:
                depth = camera.render(depth=True)
                sign = depth <= self.max_depth
            else:
                depth = sign = None
            camera._scene.free()
        else:
            kwargs = dict(mode="offscreen", width=self.image_size[0], height=self.image_size[1], camera_name=self.camera_name, device_id=-1)
            if self.obs_mode == "rgb":
                rgb, depth, sign = self.physics.render(**kwargs, depth=False), None, None
            else:
                rgb, depth = self.physics.render(**kwargs, depth=True)

                if depth is not None:
                    # The depth from mujoco is raw depth
                    extent = self.physics.model.stat.extent
                    near = self.physics.model.vis.map.znear * extent
                    far = self.physics.model.vis.map.zfar * extent
                    depth = near / (1 - depth * (1 - near / far))
                    sign = depth <= self.max_depth

        return rgb, depth, sign

    def get_obs(self, time_step=None):
        # print(_flatten_obs(time_step.observation))
        if self.obs_mode == "state":
            obs = time_step if isinstance(time_step, np.ndarray) else _flatten_obs(time_step.observation)
        else:
            with_depth = self.obs_mode in ["depth", "rgbd", "pointcloud", "xyz-img", "xyz-rgb-img", "raw_pcd"]
            rgb, depth, sign = self._get_vis_obs(with_depth=with_depth)
            camera_xyz, camera_rot = self.get_cam_pose()

            if self.obs_mode in ["xyz", "pointcloud", "xyz-img", "xyz-rgb-img"]:
                obs = {}
                if type(self) != DMCEnv:
                    depth = np.flip(depth, axis=0)
                    sign = np.flip(sign, axis=0)
                    rgb = np.flip(rgb, axis=0)
                xyz = self.get_xyz(depth, world_frame=False) @ camera_rot.T
                if self.z_to_world:
                    xyz[..., -1] += camera_xyz[-1]

                if self.obs_mode == "pointcloud":
                    assert not np.isnan(depth).any(), "Depth contains nan values!"

                    if self.n_points == -1:
                        xyz, rgb, sign = xyz.reshape(-1, 3), rgb.reshape(-1, 3), sign.reshape(-1)
                        base_line_z = xyz[sign][..., -1].min() if self.fix_base_z is None else self.fix_base_z
                        ground_mask = xyz[..., -1] <= base_line_z + self.ground_eps  # Non ground part is foreground

                        obs["filter_seg"] = ((~ground_mask) & sign)[..., None]
                        obs["filter_mask"] = sign
                    else:
                        xyz, rgb = xyz[sign], rgb[sign]  # Remove points outside the max depth
                        if xyz.shape[0] == 0:
                            xyz = np.zeros([self.n_points, 3], dtype=np.float32)
                            rgb = np.zeros([self.n_points, 3], dtype=np.uint8)
                        else:
                            base_line_z = xyz[..., -1].min() if self.fix_base_z is None else self.fix_base_z
                            ground_mask = xyz[..., -1] <= base_line_z + self.ground_eps

                            if self.num_ground > -1 and self.n_points > -1:
                                ground_mask_index, body_mask_index = np.where(ground_mask)[0], np.where(~ground_mask)[0]

                                from pyrl.utils.data import sample_and_pad

                                # from pyrl.utils.visualization import plot_show_image

                                # plot_show_image(rgb, show=True)

                                # plot_show_image(depth, show=True)

                                # print(self.n_points, self.num_ground, len(ground_mask_index), len(body_mask_index))

                                body_selected = sample_and_pad(len(body_mask_index), self.n_points - self.num_ground, self.np_random, pad=True)
                                ground_selected = sample_and_pad(len(ground_mask_index), self.num_ground, self.np_random, pad=True)

                                if len(body_mask_index) > 0 and len(ground_mask_index) > 0:
                                    index = np.concatenate([body_mask_index[body_selected], ground_mask_index[ground_selected]])
                                    xyz, rgb = xyz[index], rgb[index]
                                else:
                                    body_selected = (
                                        body_mask_index[body_selected]
                                        if len(body_mask_index) > 0
                                        else np.zeros(self.n_points - self.num_ground, dtype=body_mask_index.dtype)
                                    )
                                    ground_selected = (
                                        ground_mask_index[ground_selected]
                                        if len(ground_mask_index) > 0
                                        else np.zeros(self.num_ground, dtype=ground_mask_index.dtype)
                                    )
                                    index = np.concatenate([body_selected, ground_selected])
                                    xyz, rgb = xyz[index], rgb[index]
                                    if len(body_mask_index) == 0:
                                        xyz[: self.n_points - self.num_ground] = 0
                                        rgb[: self.n_points - self.num_ground] = 0
                                    if len(ground_mask_index) == 0:
                                        xyz[self.n_points - self.num_ground :] = 0
                                        rgb[self.n_points - self.num_ground :] = 0

                            elif self.n_points > -1:
                                len_xyz = len(xyz)
                                if len_xyz < self.n_points:
                                    index = np.arange(len(xyz))
                                    index = np.concatenate([index] * ((self.n_points + len(index) - 1) // len(index)))
                                else:
                                    index = self.np_random.permutation(xyz.shape[0])
                                index = index[: self.n_points]

                                xyz, rgb = xyz[index], rgb[index]
                                obs["filter_seg"] = ~ground_mask[index, None]

                                if len_xyz < self.n_points:
                                    obs["filter_mask"] = np.zeros(self.n_points, dtype=np.bool_)
                                    obs["filter_mask"][: len(xyz)] = True
                                else:
                                    obs["filter_mask"] = np.ones(self.n_points, dtype=np.bool_)
                else:
                    # xyz image
                    xyz[~sign] = 0
                    # AdroitEnv is a special case

                    if type(self) != DMCEnv:
                        from .adroit_utils import AdroitEnv

                        rgb = np.flip(rgb, axis=0)
                    else:
                        xyz = np.flip(xyz, axis=0)

                obs["xyz"] = xyz
                if self.obs_mode in ["xyz-rgb-img", "pointcloud"]:
                    obs["rgb"] = rgb
            else:
                obs = {}

                if type(self) != DMCEnv:
                    from .adroit_utils import AdroitEnv

                    if isinstance(self, AdroitEnv):
                        rgb = np.flip(rgb, axis=0)
                        if depth is not None:
                            depth = np.flip(depth, axis=0)
                            sign = np.flip(sign, axis=0)

                if "rgb" in self.obs_mode:
                    obs["rgb"] = rgb
                if "d" in self.obs_mode:
                    depth[~sign] = 0  # Filter pixels are too far away
                    depth = depth / self.max_depth
                    obs["depth"] = np.float16(depth)[..., None]

            ret = {}
            for k, v in obs.items():
                if v.ndim == 3:
                    ret[k] = v.transpose(2, 0, 1)
                elif v.ndim == 2:
                    ret[k] = v.transpose(1, 0)
                else:
                    assert v.ndim == 1
                    ret[k] = v
            obs = ret
        return obs

    def seed(self, seed):
        self.np_random.seed(seed)
        self.action_space.seed(seed)

    def step(self, action):
        action = np.clip((action + 1) * 0.5, a_min=0, a_max=1)
        action = self.max_action * action + self.min_action * (1 - action)
        reward = 0
        self._internal_step += 1

        for i in range(self.frame_skip):
            time_step = self.env.step(action)
            is_not_dm_control = isinstance(time_step, (tuple, list)) and type(time_step).__name__ != "TimeStep"
            if is_not_dm_control:
                reward += time_step[1]
                done = time_step[2]
            else:
                reward += time_step.reward or 0
                done = time_step.last()
            if done and i != self.frame_skip - 1:
                raise RuntimeError("DMControl env terminated early")

        obs = self.get_obs(time_step)
        if done:
            is_not_dm_control = isinstance(time_step, (tuple, list)) and type(time_step).__name__ != "TimeStep"
            if is_not_dm_control:
                info = time_step[-1]
                if isinstance(info, dict) and info.get("TimeLimit.truncated", False):
                    done = False
            else:
                if time_step.discount > 0.9:
                    done = False
        return obs, reward, done, {}

    def reset(self):
        self._internal_step = 0
        time_step = self.env.reset()
        return self.get_obs(time_step)

    def get_obs_dict(self):
        return self.env.physics.render(height=self.height, width=self.width, camera_id=self.camera_id)

    def render(self, mode="rgb_array", image_size=None, camera_id=0):
        assert mode == "rgb_array", "only support rgb_array mode, given %s" % mode

        if isinstance(image_size, int):
            height, width = image_size, image_size
        elif isinstance(image_size, (list, tuple)):
            height, width = image_size
        else:
            height, width = self.image_size
        camera_id = camera_id or self.camera_id

        if mode in ["rgb", "rgb_array"]:
            return self.env.physics.render(height=height, width=width, camera_id=camera_id, depth=False)
        elif mode in ["depth"]:
            return self.env.physics.render(height=height, width=width, camera_id=camera_id, depth=True)
        else:
            raise NotImplementedError(f"Only rgb_array mode is supported during rendering. Given mode {mode}")

    def get_state(self):
        return self.env.physics.get_state().flatten()

    def set_state(self, state):
        self.env.physics.set_state(state)


if __name__ == "__main__":
    env = gym.make("dmc_cheetah_run-v0", obs_mode="pointcloud")
    obs = env.reset()
    exit(0)

    from pyrl.utils.data import GDict

    for obs_mode in ["rgb", "rgbd", "depth", "pointcloud", "xyzimage"]:
        env = gym.make("dmc_cheetah_run-v0", image_size=[64, 64], obs_mode=obs_mode)
        obs = env.reset()
        print("obs_mode", obs_mode, "channels_first", GDict(obs).shape)
        print(env)

        num, done = 0, False
        while not done:
            obs, reward, done, _ = env.step(env.action_space.sample())
            num += 1
        print(num)
        env.close()
