import numpy as np
from gym.core import Wrapper
from gym.spaces import Discrete, Box

from pyrl.utils.data import DictArray, GDict, deepcopy, is_num, to_np, to_array
from pyrl.utils.meta import Registry, build_from_cfg


WRAPPERS = Registry("wrappers of gym environments")


def depth_to_pcd(intrinsic, depth):
    if depth.ndim == 3:
        depth = depth.squeeze()
    z = depth
    v, u = np.indices(z.shape)
    uv1 = np.stack([u + 0.5, v + 0.5, np.ones_like(z)], axis=-1)
    return uv1 @ np.linalg.inv(intrinsic).T * z[..., np.newaxis]


class ExtendedWrapper(Wrapper):
    def __getattr__(self, name):
        # gym standard do not support name with '_'
        return getattr(self.env, name)


class BufferAugmentedEnv(ExtendedWrapper):
    """
    For multi-process environments.
    Use a buffer to transfer data from sub-process to main process!
    """

    def __init__(self, env, buffers):
        super(BufferAugmentedEnv, self).__init__(env)
        self.reset_buffer = GDict(buffers[0])
        self.step_buffer = GDict(buffers[:4])
        if len(buffers) == 5:
            self.vis_img_buffer = GDict(buffers[4])

    def reset(self, *args, **kwargs):
        self.reset_buffer.assign_all(self.env.reset(*args, **kwargs))

    def step(self, *args, **kwargs):
        alls = self.env.step(*args, **kwargs)
        self.step_buffer.assign_all(alls)

    def render(self, *args, **kwargs):
        ret = self.env.render(*args, **kwargs)
        if ret is not None:
            assert self.vis_img_buffer is not None, "You need to provide vis_img_buffer!"

            self.vis_img_buffer.assign_all(ret)


class ExtendedEnv(ExtendedWrapper):
    """
    Extended api for all environments, which should be also supported by VectorEnv.

    Supported extra attributes:
    1. is_discrete, is_cost, reward_scale

    Function changes:
    1. step: reward multiplied by a scale, convert all f64 to to_f32
    2. reset: convert all f64 to to_f32

    Supported extra functions:
    2. step_random_actions
    3. step states_actions
    """

    def __init__(self, env, reward_scale, use_cost):
        super(ExtendedEnv, self).__init__(env)
        assert reward_scale > 0, "Reward scale should be positive!"
        self.is_discrete = isinstance(env.action_space, Discrete)
        self.is_cost = -1 if use_cost else 1
        self.reward_scale = reward_scale * self.is_cost

    def _process_action(self, action):
        if self.is_discrete:
            if is_num(action):
                action = int(action)
            else:
                assert action.size == 1, f"Dim of discrete action should be 1, but we get {len(action)}"
                action = int(action.reshape(-1)[0])
        return action

    def reset(self, *args, **kwargs):
        kwargs = dict(kwargs)
        obs = self.env.reset(*args, **kwargs)
        return GDict(obs).f64_to_f32(wrapper=False)

    def step(self, action, *args, **kwargs):
        import time

        st = time.time()

        action = self._process_action(action)
        obs, reward, done, info = self.env.step(action, *args, **kwargs)
        if isinstance(info, dict) and "TimeLimit.truncated" not in info:
            info["TimeLimit.truncated"] = False
        obs, info = GDict([obs, info]).f64_to_f32(wrapper=False)
        info["step_times"] = time.time() - st
        return obs, np.float32(reward * self.reward_scale), np.bool_(done), info

    # The following three functions are available for VectorEnv too!
    def step_random_actions(self, num):
        from .env_utils import true_done

        ret = None
        obs = GDict(self.reset()).copy(wrapper=False)
        prev_actions = None
        for i in range(num):
            actions = to_array(to_np(self.action_space.sample()))
            if prev_actions is None:
                prev_actions = actions * 0
            next_obs, rewards, dones, infos = self.step(actions)
            next_obs = GDict(next_obs).copy(wrapper=False)
            info_i = dict(
                obs=obs,
                next_obs=next_obs,
                actions=actions.copy(),
                prev_actions=prev_actions.copy(),
                rewards=rewards,
                dones=true_done(dones, infos),
                infos=GDict(infos).copy(wrapper=False),
                episode_dones=dones,
            )
            info_i = GDict(info_i).to_array(wrapper=False)
            obs = GDict(next_obs).copy(wrapper=False)

            prev_actions[:] = actions

            if ret is None:
                ret = DictArray(info_i, capacity=num)
            ret.assign(i, info_i)
            if dones:
                obs = GDict(self.reset()).copy(wrapper=False)
                prev_actions = prev_actions * 0
                actions = actions * 0
        # print(i, obs, next_obs, actions)
        return ret.to_two_dims(wrapper=False)

    def step_states_actions(self, states=None, actions=None):
        """
        For CEM only
        states: [N, NS]
        actions: [N, L, NA]
        return [N, L, 1]
        """
        assert actions.ndim == 3
        rewards = np.zeros_like(actions[..., :1], dtype=np.float32)
        for i in range(len(actions)):
            if hasattr(self, "set_state") and states is not None:
                self.set_state(states[i])
            for j in range(len(actions[i])):
                rewards[i, j] = self.step(actions[i, j])[1]
        return rewards

    def get_env_state(self):
        ret = {}
        if hasattr(self.env, "get_state"):
            ret["env_states"] = self.env.get_state()
        if hasattr(self.env, "level"):
            ret["env_levels"] = self.env.level
        return ret

    def seed(self, seed=None):
        # print(seed)
        self.env.seed(seed)
        self.env.action_space.seed(seed)


class MujocoWrapper(ExtendedWrapper):
    def get_state(self):
        if hasattr(self.env, "goal"):
            return np.concatenate([self.env.sim.get_state().flatten(), self.env.goal], axis=-1)
        else:
            return self.env.sim.get_state().flatten()

    def set_state(self, state):
        if hasattr(self.env, "goal"):
            sim_state_len = self.env.sim.get_state().flatten().shape[0]
            self.env.sim.set_state(self.env.sim.get_state().from_flattened(state[:sim_state_len], self.env.sim))
            self.env.goal = state[sim_state_len:]
        else:
            self.env.sim.set_state(self.env.sim.get_state().from_flattened(state, self.env.sim))

    def get_obs(self):
        return self.env.unwrapped._get_obs()


class PendulumWrapper(ExtendedWrapper):
    def get_state(self):
        return np.array(self.env.state)

    def set_state(self, state):
        self.env.state = deepcopy(state)

    def get_obs(self):
        return self.env._get_obs()


@WRAPPERS.register_module()
class FixedInitWrapper(ExtendedWrapper):
    def __init__(self, env, init_state, level=None, *args, **kwargs):
        super(FixedInitWrapper, self).__init__(env)
        self.init_state = np.array(init_state)
        self.level = level

    def reset(self, *args, **kwargs):
        if self.level is not None:
            # For ManiSkill
            self.env.reset(level=self.level)
        else:
            self.env.reset()
        self.set_state(self.init_state)
        return self.env.get_obs()

class RenderInfoWrapper(ExtendedWrapper):
    def step(self, action):
        obs, rew, done, info = super().step(action)
        info["reward"] = rew
        self._info_for_render = info
        return obs, rew, done, info

    def reset(self, **kwargs):
        obs = super().reset(**kwargs)
        # self._info_for_render = self.env.get_info()
        self._info_for_render = {}
        return obs

    def render(self, mode, **kwargs):
        from pyrl.utils.image.misc import put_info_on_image

        if mode == "rgb_array" or mode == "cameras":
            img = super().render(mode=mode, **kwargs)
            return put_info_on_image(img, self._info_for_render, extras=None, overlay=True)
        else:
            return super().render(mode=mode, **kwargs)
        
        
@WRAPPERS.register_module()
class FrameStackWrapper(ExtendedWrapper):
    def __init__(self, env, num_frames: int, **kwargs) -> None:
        super().__init__(env)
        self.num_frames = num_frames
        self.obs_mode = getattr(self.env, "obs_mode", "state")
        self.frames = []
        self.pos_encoding = np.eye(num_frames, dtype=np.uint8)

    def observation(self):
        if self.obs_mode == "pointcloud":
            num_points = self.frames[0]["xyz"].shape[-1]
            pos_encoding = np.repeat(self.pos_encoding, num_points, axis=-1)
            obs = GDict.concat(self.frames, axis=-1, wrapper=False)

            obs["pos_encoding"] = pos_encoding
            return obs
        else:
            return GDict.concat(self.frames, axis=-3, wrapper=False)

    def step(self, actions, idx=None):
        next_obs, rewards, dones, infos = self.env.step(actions)
        self.frames = self.frames[1:] + [next_obs]
        return self.observation(), rewards, dones, infos

    def reset(self, **kwargs):
        obs = self.env.reset(**kwargs)
        self.frames = [obs] * self.num_frames
        return self.observation()


@WRAPPERS.register_module()
class MuJoCoVisual(ExtendedWrapper):
    def __init__(self, env, img_size=(84, 84), action_repeat=2, **kwargs):
        self.env, self.img_size, self.action_repeat = env, img_size, action_repeat
        self.obs_mode = "rgb"
        self.action_space = env.action_space

    def _get_obs(self):
        rgb = self.env.render("rgb_array", width=self.img_size[0], height=self.img_size[1])
        rgb = rgb.transpose(2, 0, 1)
        return {"rgb": rgb}

    def reset(self, *args, **kwargs):
        self.env.reset(*args, **kwargs)
        return self._get_obs()

    def step(self, action):
        reward = 0
        for i in range(self.action_repeat):
            _, reward_i, done, info = self.env.step(action)
            reward += reward_i
        return self._get_obs(), reward, done, info

    def seed(self, seed):
        self.env.seed(seed)
        self.action_space.seed(seed)


def build_wrapper(cfg, default_args=None):
    return build_from_cfg(cfg, WRAPPERS, default_args)
