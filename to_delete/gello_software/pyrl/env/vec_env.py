import numpy as np
from copy import deepcopy
from gym.core import Env

from pyrl.utils.data import DictArray, GDict, SharedDictArray, split_list_of_parameters, concat, is_num, index_to_slice, to_torch
from pyrl.utils.meta import Worker, get_world_rank
from pyrl.utils.math import split_num
from .action_space_utils import stack_action_space
from .env_utils import build_env, convert_observation_to_space
from .wrappers import ExtendedEnv, ExtendedWrapper

"""
@property
def running_idx(self):
    return [i for i, env in enumerate(self.workers) if env.is_running]

@property
def ready_idx(self):
    return [i for i, env in enumerate(self.workers) if env.is_ready]

@property
def idle_idx(self):
    return [i for i, env in enumerate(self.workers) if env.is_idle]

"""


def create_buffer_for_env(env, num_envs=1, shared_np=True):
    assert isinstance(env, ExtendedEnv)
    obs = env.reset()
    item = [obs, np.float32(1.0), True, GDict(env.step(env.action_space.sample())[-1]).to_array().float(), env.render("rgb_array")]
    buffer = DictArray(GDict(item).to_array(), capacity=num_envs)
    if shared_np:
        buffer = SharedDictArray(buffer)
    return buffer


class UnifiedVectorEnvAPI(ExtendedWrapper):
    """
    This wrapper is necessary for all environments.
    Returned data may be a buffer, please not in-place modify them!

    Copy_to_cpu:
        1. We use a extra buffer to store this data.
        2. Use stream "copy" to copy it.
        3. Sychronize before push to the buffer.
    """

    def __init__(self, vec_env):
        super(UnifiedVectorEnvAPI, self).__init__(vec_env)
        assert isinstance(vec_env, VectorEnvBase), f"Please use correct type of environments {type(vec_env)}!"
        self.vec_env, self.num_envs, self.action_space = vec_env, vec_env.num_envs, vec_env.action_space
        self.all_env_indices = np.arange(self.num_envs, dtype=np.int32)

        self.single_env = self.vec_env.single_env
        self.is_discrete, self.reward_scale, self.is_cost = self.single_env.is_discrete, self.single_env.reward_scale, self.single_env.is_cost

        self.recent_actions = self.vec_env.action_space.sample() * 0
        self.prev_actions = self.recent_actions.copy()
        self.recent_obs = DictArray(self.vec_env.reset(idx=self.all_env_indices)).copy()
        self.episode_dones = np.zeros([self.num_envs, 1], dtype=np.bool_)

        self.dirty = False

        if self.server_based_env:
            # For step_dict only: avoid GPU mallocing which will block everything
            self.cpu_step_obs_buffer = self.recent_obs.to_torch(device="cpu", use_copy=True).pin_memory()
            self.cpu_step_next_obs_buffer = self.recent_obs.to_torch(device="cpu", use_copy=True).pin_memory()

        self.reseed()

    @property
    def server_based_env(self):
        return isinstance(self.vec_env, (ServerBasedVectorEnv, ServerBasedVectorSingleEnv))

    def obs_next_obs_cpu(self, idx=None):
        if not self.server_based_env:
            return {}

        if idx is None:
            obs = self.cpu_step_obs_buffer
            next_obs = self.cpu_step_next_obs_buffer
        else:
            idx = slice(0, len(idx))
            obs = self.cpu_step_obs_buffer.slice(idx)
            next_obs = self.cpu_step_next_obs_buffer.slice(idx)
        from pyrl.utils.torch import get_stream

        get_stream("copy_to_cpu", False).synchronize()
        return {"obs": obs.to_numpy(wrapper=False), "next_obs": next_obs.to_numpy(wrapper=False)}

    @property
    def done_idx(self):
        return np.nonzero(self.episode_dones)[0]

    def _process_idx(self, idx):
        if idx is None:
            slice_idx = slice(None)
            idx = self.all_env_indices
        else:
            slice_idx = index_to_slice(idx)
        self.vec_env._assert_id(idx)
        return idx, slice_idx

    def reset(self, idx=None, *args, **kwargs):
        self.dirty = False

        idx, slice_idx = self._process_idx(idx)
        args, kwargs = list(args), dict(kwargs)
        if len(args) > 0:
            for i, arg_i in enumerate(args):
                if not hasattr(arg_i, "__len__"):
                    args[i] = np.array([arg_i for j in idx])
                assert len(arg_i) == len(idx), f"Len of value {len(arg_i)} is not {len(idx)}!"
        if len(kwargs) > 0:
            for key, value in kwargs.items():
                if not hasattr(value, "__len__"):
                    kwargs[key] = np.array([value for i in idx])
                assert len(value) == len(idx), f"Len of value {len(value)} is not {len(idx)}!"
        obs = self.vec_env.reset(idx=idx, *args, **kwargs)

        self.recent_actions[slice_idx] *= 0
        self.prev_actions[slice_idx] *= 0
        self.recent_obs.assign(slice_idx, obs, non_blocking=True)
        self.episode_dones[slice_idx] = False
        return obs

    def _update_actions(self, actions, idx):
        self.prev_actions[idx] = self.recent_actions[idx]
        self.recent_actions[idx] = actions

    def step_async(self, actions, idx=None):
        idx, slice_idx = self._process_idx(idx)

        assert not self.dirty, "You need to reset environment after doing step_states_actions!"
        assert len(actions) == len(idx), f"shape of actions: {actions.shape}, #idx: {len(idx)}"

        self._update_actions(actions, slice_idx)
        self.vec_env.step_async(actions, idx=idx)

    def step(self, actions, idx=None):
        idx, slice_idx = self._process_idx(idx)

        assert not self.dirty, "You need to reset environment after doing step_states_actions!"
        assert len(actions) == len(idx), f"shape of actions: {actions.shape}, #idx: {len(idx)}"

        self._update_actions(actions, slice_idx)
        infos = self.vec_env.step(actions, idx=idx)

        self.recent_obs.assign(slice_idx, infos[0])
        self.episode_dones[slice_idx] = infos[2]
        return DictArray(infos).copy(wrapper=False)

    def render(self, mode="rgb_array", idx=None):
        return self.vec_env.render(mode, self._process_idx(idx)[0])

    def step_states_actions(self, *args, **kwargs):
        self.dirty = True
        self.vec_env._assert_id(self.all_env_indices)
        return self.vec_env.step_states_actions(*args, **kwargs)

    def step_random_actions(self, num):
        self.dirty = True
        self.vec_env._assert_id(self.all_env_indices)
        return self.vec_env.step_random_actions(num)

    def get_attr(self, name, idx=None):
        return self.vec_env.get_attr(name, self._process_idx(idx)[0])

    def call(self, name, idx=None, *args, **kwargs):
        return self.vec_env.call(name, self._process_idx(idx)[0], *args, **kwargs)

    def get_state(self, idx=None):
        return GDict(self.call("get_state", idx=idx)).copy(wrapper=False)

    def get_obs(self, idx=None):
        return GDict(self.call("get_obs", idx=idx)).copy(wrapper=False)

    def set_state(self, state, idx=None):
        return self.call("set_state", state=state, idx=idx)

    def seed(self, seed):
        return self.call("seed", seed=seed)

    def reseed(self):
        return self.vec_env.reseed()

    def get_env_state(self, idx=None):
        return self.call("get_env_state", idx=idx)

    # def random_action(self):
    #     return self.action_space.sample()

    def _step_res_to_dict(self, infos, obs, idx, slice_idx):
        from .env_utils import true_done
        from pyrl.utils.torch import get_stream

        next_obs, reward, done, info = GDict(infos).copy(wrapper=False)
        if self.server_based_env:
            with get_stream("copy_to_cpu"):
                self.cpu_step_obs_buffer.assign(slice(0, len(idx)), obs, non_blocking=True)
                self.cpu_step_next_obs_buffer.assign(slice(0, len(idx)), next_obs, non_blocking=True)

        return dict(
            obs=obs,
            next_obs=next_obs,
            prev_actions=self.prev_actions[slice_idx].copy(),
            actions=self.recent_actions[slice_idx].copy(),
            rewards=reward,
            dones=true_done(done, info),
            episode_dones=done,
            infos=info,
            worker_indices=idx[:, None],
        )

    # Special functions
    def step_dict(self, actions, idx=None, restart=True):
        idx, slice_idx = self._process_idx(idx)
        obs = self.recent_obs.slice(slice_idx).copy(wrapper=False)
        infos = self.step(actions, idx=idx)

        dones = infos[2]
        ret = self._step_res_to_dict(infos, obs, idx, slice_idx)
        if np.any(dones) and restart:
            self.reset(idx=idx[np.where(dones[..., 0])[0]])
        return ret

    def wait_dict(self, num=None, restart=True):
        idx, infos = self.vec_env.wait(num=num)
        idx, slice_idx = self._process_idx(idx)
        obs = self.recent_obs.slice(slice_idx).copy(wrapper=False)

        dones = infos[2]
        ret = self._step_res_to_dict(infos, obs, idx, slice_idx)
        if np.any(dones) and restart:
            self.reset(idx=idx[np.where(dones[..., 0])[0]])
        else:
            self.recent_obs.assign(slice_idx, infos[0])
        return idx, ret

    def __getattr__(self, name, idx=None):
        return self.vec_env.get_attr(name, self._process_idx(idx)[0])

    def close(self):
        self.vec_env.close()


class VectorEnvBase(Env):
    SHARED_NP_BUFFER: bool

    def __init__(self, env_cfgs=None, wait_num=None, timeout=None, **kwargs):
        super(VectorEnvBase, self).__init__()
        single_env_cfg = deepcopy(env_cfgs[0])
        if isinstance(self, (ServerBasedVectorEnv, ServerBasedVectorSingleEnv)):
            single_env_cfg.pop("renderer", None)
            single_env_cfg.pop("render_client_kwargs", None)
            single_env_cfg["use_client"] = True

        self.env_cfgs, self.single_env, self.num_envs = env_cfgs, build_env(single_env_cfg), len(env_cfgs)

        assert wait_num is None and timeout is None, "We do not support partial env step now!"
        self.timeout = int(1e9) if timeout is None else timeout
        self.wait_num = len(env_cfgs) if wait_num is None and env_cfgs is not None else wait_num
        self.workers, self.buffers = None, None
        self.action_space = stack_action_space(self.single_env.action_space, self.num_envs)
        self.all_env_indices = np.arange(self.num_envs)
        self.vec_base_seed = None

        if self.SHARED_NP_BUFFER is not None:
            self.buffers = create_buffer_for_env(self.single_env, self.num_envs, self.SHARED_NP_BUFFER)
            buffers = self.buffers.memory
            self.reset_buffer = DictArray(buffers[0])
            self.step_buffer = DictArray(buffers[:4])
            self.vis_img_buffer = DictArray(buffers[4])

    def __getattr__(self, name, idx=None):
        if not hasattr(self.single_env, name):
            return None
        else:
            return self.get_attr(name, idx)

    def _init_obs_space(self):
        single_obs = self.single_env.reset()
        obs = DictArray(single_obs, capacity=1).repeat(self.num_envs, wrapper=False)
        self.observation_space = convert_observation_to_space(obs)

    def _process_idx(self, idx):
        if idx is None:
            slice_idx = slice(None)
            idx = self.all_env_indices
        else:
            slice_idx = index_to_slice(idx)
        self._assert_id(idx)
        return idx, slice_idx

    def _assert_id(self, idx=None):
        raise NotImplementedError

    def reset(self, idx=None, *args, **kwargs):
        raise NotImplementedError

    def step(self, actions, idx=None):
        raise NotImplementedError

    def render(self, mode, idx=None):
        raise NotImplementedError

    def step_states_actions(self, states, actions):
        raise NotImplementedError

    def step_random_actions(self, num):
        raise NotImplementedError

    def get_attr(self, name, idx=None):
        raise NotImplementedError

    def call(self, name, idx=None, *args, **kwargs):
        raise NotImplementedError

    def get_obs(self, idx=None):
        return self.call("get_obs", idx)

    def get_state(self, idx=None):
        return self.call("get_state", idx)

    def set_state(self, state, idx=None):
        return self.call("set_state", state=state, idx=idx)

    def get_env_state(self, idx=None):
        return self.call("get_env_state", idx=idx)

    def seed(self, seed, idx=None):
        raise NotImplementedError

    def reseed(self):
        self.seed(self.vec_base_seed)

    def __str__(self):
        return "<{}{}>".format(type(self).__name__, self.single_env)

    def close(self):
        if self.workers is not None:
            for worker in self.workers:
                worker.close()
        if self.buffers is not None:
            del self.buffers


class SingleEnv2VecEnv(VectorEnvBase):
    """
    Build vectorized api for single environment!
    """

    SHARED_NP_BUFFER = None

    def __init__(self, env_cfgs, seed=None, **kwargs):
        assert len(env_cfgs) == 1
        super(SingleEnv2VecEnv, self).__init__(env_cfgs, **kwargs)
        self.vec_base_seed = np.random.randint(int(1e9)) if seed is None else seed
        if isinstance(self, ServerBasedVectorSingleEnv):
            self._env = build_env(env_cfgs[0])
        else:
            self._env = self.single_env
        self._init_obs_space()

    def _assert_id(self, idx):
        return True

    def _unsqueeze(self, item):
        if item is not None:
            return GDict(item).to_array().unsqueeze(axis=0, wrapper=False)

    def reset(self, idx=None, *args, **kwargs):
        args = list(args)
        kwargs = dict(kwargs)
        args, kwargs = GDict([args, kwargs]).slice(0, wrapper=False)
        ret = self._unsqueeze(self._env.reset(*args, **kwargs))
        return ret

    def step(self, actions, idx=None):
        ret = self._env.step(actions[0])
        ret = self._unsqueeze(ret)
        return ret

    def render(self, mode, idx=None):
        return self._unsqueeze(self._env.render(mode))

    def step_states_actions(self, *args, **kwargs):
        return self._env.step_states_actions(*args, **kwargs)

    def step_random_actions(self, num):
        ret = self._env.step_random_actions(num)
        ret["worker_indices"] = np.zeros(ret["dones"].shape, dtype=np.int32)
        return GDict(ret).to_two_dims(wrapper=False)

    def get_attr(self, name, idx=None):
        return self._unsqueeze(getattr(self._env, name))

    def call(self, name, idx=None, *args, **kwargs):
        args, kwargs = GDict(list(args)).squeeze(0, False), GDict(dict(kwargs)).squeeze(0, False)
        ret = getattr(self._env, name)(*args, **kwargs)
        ret = GDict(ret).to_array()
        return self._unsqueeze(ret)

    def seed(self, seed):
        if seed is not None:
            self.vec_base_seed = seed
            self._env.seed(self.vec_base_seed)
            self.action_space.seed(self.vec_base_seed)


class VectorEnv(VectorEnvBase):
    """
    Always use shared memory and requires the environment have type BufferAugmentedEnv
    """

    SHARED_NP_BUFFER = True

    def __init__(self, env_cfgs, seed=None, **kwargs):
        super(VectorEnv, self).__init__(env_cfgs=env_cfgs, **kwargs)
        self.vec_base_seed = np.random.randint(int(1e9)) if seed is None else seed
        self.workers = [Worker(build_env, i, self.vec_base_seed + i, True, self.buffers.get_infos(), cfg=cfg) for i, cfg in enumerate(env_cfgs)]
        self.running_idx = []
        self._init_obs_space()
        # self.seed(self.vec_base_seed)

    def _assert_id(self, idx):
        for i in idx:
            assert self.workers[i].is_idle, f"Cannot interact with environment {i} which is stepping now."

    def remove_running_idx(self, idx=None):
        if idx is None:
            self.running_idx = []
        else:
            list_idx = list(idx)
            self.running_idx = [_ for _ in self.running_idx if _ not in list_idx]

    def reset(self, idx=None, *args, **kwargs):
        idx, slice_idx = self._process_idx(idx)
        args, kwargs = list(args), dict(kwargs)
        all_kwargs = GDict([args, kwargs])
        for i in range(len(idx)):
            args_i, kwargs_i = all_kwargs.slice(i, wrapper=False)
            self.workers[idx[i]].call("reset", *args_i, **kwargs_i)
        [self.workers[i].wait() for i in idx]
        self.remove_running_idx(idx)
        return self.reset_buffer.slice(slice_idx, wrapper=False)

    def step_async(self, actions, idx=None):
        for i in range(len(idx)):
            self.workers[idx[i]].call("step", action=actions[i])
        self.running_idx += list(idx)

    def wait(self, num=None):
        num = num or len(self.running_idx)
        # print(num, self.running_idx)
        num = min(num, len(self.running_idx))
        ret_idx = []
        while len(ret_idx) < num:
            unfinished = []
            for i in self.running_idx:
                ret = self.workers[i].wait_async()
                if ret is None:
                    unfinished.append(i)
                else:
                    ret_idx.append(i)
            self.running_idx = unfinished
        ret_idx = np.array(sorted(ret_idx), np.int32)
        ret_idx, slice_idx = self._process_idx(ret_idx)
        return ret_idx, self.step_buffer.slice(slice_idx, wrapper=False)

    def step(self, actions, idx=None):
        idx, slice_idx = self._process_idx(idx)
        self.step_async(actions, idx)
        slice_idx = slice(None) if len(idx) == len(self.workers) else idx
        [self.workers[i].wait() for i in idx]
        self.remove_running_idx(idx)

        # print(self.step_buffer[-1])
        # exit(0)
        return self.step_buffer.slice(slice_idx, wrapper=False)

    def render(self, mode="rgb_array", idx=None):
        idx, slice_idx = self._process_idx(idx)
        for i in idx:
            self.workers[i].call("render", mode=mode)
        [self.workers[i].wait() for i in idx]
        return self.vis_img_buffer.slice(slice_idx, wrapper=False)

    def step_random_actions(self, num):
        # For replay buffer warmup of the RL agent
        n, num_per_env = split_num(num, self.num_envs)
        self._assert_id(list(range(n)))

        shared_mem_value = [bool(self.workers[i].shared_memory.value) for i in range(n)]
        for i, num_i in enumerate(num_per_env):
            self.workers[i].set_shared_memory(False)
            self.workers[i].call("step_random_actions", num=num_i)
        ret = []
        for i in range(n):
            ret_i = self.workers[i].wait()
            ret_i["worker_indices"] = np.ones(ret_i["dones"].shape, dtype=np.int32) * i
            ret.append(ret_i)
            self.workers[i].set_shared_memory(shared_mem_value[i])
        return DictArray.concat(ret, axis=0, wrapper=False)

    def step_states_actions(self, states, actions):
        """
        Return shape: [N, LEN, 1]
        """
        # For MPC
        paras = split_list_of_parameters(self.num_envs, states=states, actions=actions)
        n = len(paras)
        self._assert_id(list(range(n)))

        shared_mem_value = [bool(self.workers[i].shared_memory.value) for i in range(n)]
        for i in range(n):
            args_i, kwargs_i = paras[i]
            self.workers[i].set_shared_memory(False)
            self.workers[i].call("step_states_actions", *args_i, **kwargs_i)
        ret = []
        for i in range(n):
            ret.append(self.workers[i].wait())
            self.workers[i].set_shared_memory(shared_mem_value[i])
        return concat(ret, axis=0)

    def get_attr(self, name, idx=None):
        shared_mem_value = [bool(self.workers[i].shared_memory.value) for i in idx]
        for i in idx:
            self.workers[i].set_shared_memory(False)
            self.workers[i].get_attr(name)
        ret = []
        for i, mem_flag in zip(idx, shared_mem_value):
            ret.append(self.workers[i].wait())
            self.workers[i].set_shared_memory(mem_flag)
        ret = GDict(ret).to_array()
        return GDict.stack(ret, axis=0, wrapper=False)

    def call(self, name, idx=None, *args, **kwargs):
        args, kwargs = GDict(list(args)), GDict(dict(kwargs))

        shared_mem_value = [bool(self.workers[i].shared_memory.value) for i in idx]
        for j, i in enumerate(idx):
            self.workers[i].set_shared_memory(False)
            self.workers[i].call(name, *args.slice(j, 0, False), **kwargs.slice(j, 0, False))

        ret = []
        for i, mem_flag in zip(idx, shared_mem_value):
            ret.append(self.workers[i].wait())
            self.workers[i].set_shared_memory(mem_flag)
        ret = GDict(ret).to_array()
        return None if ret[0] is None else GDict.stack(ret, axis=0, wrapper=False)

    def seed(self, base_seed):
        if base_seed is not None:
            self.vec_base_seed = base_seed
            [self.workers[i].call("seed", self.vec_base_seed + i) for i in range(self.num_envs)]
            [self.workers[i].wait() for i in range(self.num_envs)]
            self.action_space.seed(self.vec_base_seed)


class ServerBasedVectorEnv(VectorEnv):
    """
    Always use shared memory and requires the environment have type BufferAugmentedEnv
    """

    SHARED_NP_BUFFER = True

    def __init__(self, env_cfgs, server_port, seed=None, *args, **kwargs):
        # For maniskill2 only
        from sapien.core import RenderServer

        self.obs_mode = env_cfgs[0].get("obs_mode")
        assert self.obs_mode not in [
            "none",
            "state",
            "state_dict",
        ], "If you want to use state representation, please use normal env instead of server based env"

        self.server = RenderServer(device=env_cfgs[0].get("device", "cuda:0"))
        server_port = f"localhost:{server_port + get_world_rank()}"
        self.server.start(server_port)

        for i, cfg_i in enumerate(env_cfgs):
            cfg_i["renderer"] = "render_client"
            cfg_i["render_client_kwargs"] = {
                "address": server_port,
                "process_index": i,
            }
            cfg_i["use_client"] = True
            env_cfgs[i] = cfg_i
        super(ServerBasedVectorEnv, self).__init__(env_cfgs=env_cfgs, seed=seed, *args, **kwargs)
        super(ServerBasedVectorEnv, self).reset(idx=np.arange(len(env_cfgs)))
        # self.seed(seed)

        self.texture_name = self.single_env.SUPPORTED_TEXTURE_NAMES[self.single_env.obs_mode]
        self.texture_map = {name: i for i, name in enumerate(self.texture_name)}

        self.camera_names = [camera.get_name() for camera in self.single_env.unwrapped._scene.get_cameras() if camera.get_name() != "render_camera"]
        self.render_buffer = self.server.auto_allocate_torch_tensors(self.texture_name)

        buffers = self.torch_buffer()
        self.reset_buffer = DictArray(buffers[0])
        self.reset_cpu_buffer = self.reset_buffer.cpu()

        self.step_buffer = DictArray(buffers[:4])
        self.vis_img_buffer = DictArray(buffers[4])

        self.in_rendering = []

    def torch_buffer(self):
        from pyrl.utils.data import to_torch

        buffers = GDict(self.buffers)
        if self.obs_mode == "rgbd":
            for name in self.camera_names:
                item = buffers[0]["image"][name]
                for key in ["rgb", "depth"]:
                    if key in item:
                        item[key] = to_torch(item[key], device=self.render_buffer[0].device)
        elif self.obs_mode == "pointcloud":
            # print(buffers.shape)
            # exit(0)
            for name in self.camera_names:
                item = buffers[0]["pointcloud"][name]
                for key in ["rgb", "xyz"]:
                    if key in item:
                        item[key] = to_torch(item[key], device=self.render_buffer[0].device)
        else:
            raise NotImplementedError
        return buffers.memory

    def visual_obs_post_process(self, idx):
        idx, slice_idx = self._process_idx(idx)
        obs_buffer = self.reset_buffer
        import torch

        with torch.no_grad():
            if self.obs_mode == "pointcloud":
                rgb = torch.clamp(self.render_buffer[self.texture_map["Color"]][slice_idx, ..., :3] * 255, 0, 255).type(torch.uint8)
                xyz = self.render_buffer[self.texture_map["Position"]].clone()
                xyz[..., -1] = 0

                for i, name in enumerate(self.camera_names):
                    camera_extrinsic = to_torch(
                        obs_buffer["pointcloud"][name]["camera_extrinsic"], device=self.render_buffer[0].device, non_blocking=True
                    )
                    # tmp = GDict([obs_buffer["pointcloud"][name]["xyz"], xyz, camera_extrinsic])
                    # print(tmp.shape, tmp.dtype, tmp.device)
                    # exit(0)
                    obs_buffer["pointcloud"][name]["xyz"][slice_idx] = torch.einsum("bijk,blk->bijl", xyz[:, i], camera_extrinsic)[..., :3]
                    obs_buffer["pointcloud"][name]["rgb"][slice_idx] = rgb[:, i]
            elif self.obs_mode == "rgbd":
                rgb = torch.clamp(self.render_buffer[self.texture_map["Color"]][slice_idx, ..., :3] * 255, 0, 255).type(torch.uint8)
                depth = -self.render_buffer[self.texture_map["Position"]][slice_idx, ..., 2:3]
                for i, name in enumerate(self.camera_names):
                    obs_buffer["image"][name]["rgb"][slice_idx] = rgb[:, i]
                    obs_buffer["image"][name]["depth"][slice_idx] = depth[:, i]
            else:
                raise NotImplementedError

    def reset(self, idx=None, *args, **kwargs):
        idx, slice_idx = self._process_idx(idx)
        super(ServerBasedVectorEnv, self).reset(idx=idx, *args, **kwargs)
        self.server.wait_scenes(idx)
        self.visual_obs_post_process(idx)
        return self.reset_buffer.slice(slice_idx, wrapper=False)

    def step(self, actions=None, idx=None, *args, **kwargs):
        idx, slice_idx = self._process_idx(idx)
        super(ServerBasedVectorEnv, self).step(actions=actions, idx=idx, *args, **kwargs)
        from pyrl.utils.sapien import get_profiler_block

        with get_profiler_block("Wait scenes"):
            self.server.wait_scenes(idx)
        self.visual_obs_post_process(idx)
        return self.step_buffer.slice(slice_idx, wrapper=False)

    def wait(self, num=None):
        pending_envs = len(self.running_idx) + len(self.in_rendering)
        num = num or pending_envs
        num = min(num, pending_envs)
        ret_idx = []
        while len(ret_idx) < num:  # Two-stage busy waiting
            unfinished = []
            for i in self.running_idx:
                simulation_finished = self.workers[i].wait_async()
                if simulation_finished:
                    self.in_rendering.append(i)
                else:
                    unfinished.append(i)
            self.running_idx = unfinished

            unfinished = []
            for i in self.in_rendering:
                render_finished = self.server.wait_scenes([i], timeout=0)
                if render_finished:
                    ret_idx.append(i)
                else:
                    unfinished.append(i)
            self.in_rendering = unfinished
        ret_idx = np.array(sorted(ret_idx), np.int32)
        ret_idx, slice_idx = self._process_idx(ret_idx)
        self.visual_obs_post_process(ret_idx)
        return ret_idx, self.step_buffer.slice(slice_idx, wrapper=False)

    def render(self):
        raise NotImplementedError("Not supported!")


class ServerBasedVectorSingleEnv(SingleEnv2VecEnv):
    """
    Always use shared memory and requires the environment have type BufferAugmentedEnv
    Also, it is designed for ManiSkill2
    """

    SHARED_NP_BUFFER = True

    def __init__(self, env_cfgs, server_port, seed=None, *args, **kwargs):
        # For maniskill2 only
        from sapien.core import RenderServer

        self.obs_mode = env_cfgs[0].get("obs_mode")
        assert self.obs_mode not in [
            "none",
            "state",
            "state_dict",
        ], "If you want to use state representation, please use normal env instead of server based env"

        device = env_cfgs[0].get("device", "cuda:0")
        self.server = RenderServer(device=device)
        server_port = f"localhost:{server_port + get_world_rank()}"
        self.server.start(server_port)

        for i, cfg_i in enumerate(env_cfgs):
            cfg_i["renderer"] = "render_client"
            cfg_i["render_client_kwargs"] = {
                "address": server_port,
                "process_index": i,
            }
            cfg_i["use_client"] = True
            env_cfgs[i] = cfg_i
        super(ServerBasedVectorSingleEnv, self).__init__(seed=seed, env_cfgs=env_cfgs, *args, **kwargs)
        super(ServerBasedVectorSingleEnv, self).reset(idx=np.arange(len(env_cfgs)))
        # self._env.seed(seed)

        self.texture_name = self.single_env.SUPPORTED_TEXTURE_NAMES[self.single_env.obs_mode]
        self.texture_map = {name: i for i, name in enumerate(self.texture_name)}
        self.camera_names = [camera.get_name() for camera in self.single_env.unwrapped._scene.get_cameras() if camera.get_name() != "render_camera"]

        self.render_buffer = self.server.auto_allocate_torch_tensors(self.texture_name)
        self.torch_obs_buffer = DictArray(self.torch_buffer())

    def torch_buffer(self):
        from pyrl.utils.data import to_torch

        ret = {}
        if self.obs_mode == "rgbd":
            for name in self.camera_names:
                obs_buffer = self.reset_buffer["image"][name]
                ret[name] = {}
                for key in ["rgb", "depth"]:
                    if key in obs_buffer:
                        ret[name][key] = to_torch(obs_buffer[key], device=self.render_buffer[0].device)
        else:
            raise NotImplementedError
            # for name in self.camera_names:
            #     buffers[0].pop("pointcloud")
            #     item = buffers[0]["pointcloud"] = {}
            #     for key in ["rgb", "depth"]:
            #         if key in item:
            #             item[key] = to_torch(item[key], device=self.render_buffer[0].device)
        return {"image": ret}

    def visual_obs_post_process(self, idx):
        idx, slice_idx = self._process_idx(idx)
        import torch

        obs_buffer = self.torch_obs_buffer
        if self.obs_mode == "pointcloud":
            raise NotImplementedError
        elif self.obs_mode == "rgbd":
            rgb = torch.clamp(self.render_buffer[self.texture_map["Color"]][slice_idx, ..., :3] * 255, 0, 255).type(torch.uint8)
            depth = -self.render_buffer[self.texture_map["Position"]][slice_idx, ..., slice(2, 3)]

            for i, name in enumerate(self.camera_names):
                # print(obs_buffer["image"][name]["rgb"].shape, rgb.shape)
                obs_buffer["image"][name]["rgb"][slice_idx].copy_(rgb[:, i], non_blocking=True)
                obs_buffer["image"][name]["depth"][slice_idx].copy_(depth[:, i], non_blocking=True)

    def reset(self, idx=None, *args, **kwargs):
        idx, slice_idx = self._process_idx(idx)
        obs = super(ServerBasedVectorSingleEnv, self).reset(idx=idx, *args, **kwargs)
        self.server.wait_scenes(idx)
        self.visual_obs_post_process(idx)
        images_and_states = self.torch_obs_buffer.slice(slice_idx)
        images_and_states.update(obs)
        return images_and_states.memory

    def step(self, actions=None, idx=None, *args, **kwargs):
        pass

        idx, slice_idx = self._process_idx(idx)
        next_obs, rewards, dones, infos = super(ServerBasedVectorSingleEnv, self).step(actions=actions, idx=idx, *args, **kwargs)

        self.server.wait_scenes(idx)
        self.visual_obs_post_process(idx)

        images_and_states = self.torch_obs_buffer.slice(slice_idx)
        images_and_states.update(next_obs)
        next_obs = images_and_states.memory

        return next_obs, rewards, dones, infos

    def wait(self, *args, **kwargs):
        raise NotImplementedError("Not supported!")

    def render(self):
        raise NotImplementedError("Not supported!")


class SapienThreadEnv(VectorEnvBase):
    SHARED_NP_BUFFER = False

    # This vectorized env is designed for maniskill
    def __init__(self, env_cfgs, seed=None, **kwargs):
        self._check_cfgs(env_cfgs)
        super(SapienThreadEnv, self).__init__(env_cfgs, **kwargs)
        self.workers = []
        for i, cfg in enumerate(env_cfgs):
            cfg["buffers"] = self.buffers.slice(i, wrapper=False)
            self.workers.append(build_env(cfg))

        base_seed = np.random.randint(int(1e9)) if seed is None else seed
        [env.seed(base_seed + i) for i, env in enumerate(self.workers)]

        # For step async in sapien
        self._num_finished = 0
        self._env_indices = np.arange(self.num_envs)
        # -1: idle, 0: simulation, 1: rendering, 2: done (need to be reset to idle after get all information)
        self._env_stages = np.ones(self.num_envs, dtype=np.int32) * -1
        self._env_flags = [None for i in range(self.num_envs)]
        self._init_obs_space()

    @classmethod
    def _check_cfgs(self, env_cfgs):
        sign = True
        for cfg in env_cfgs:
            sign = sign and (cfg.get("with_torch", False) and cfg.get("with_cpp", False))
        if not sign:
            from pyrl.utils.meta import get_logger

            logger = get_logger()
            logger.warning("You need to use torch and cpp extension, otherwise the speed is not fast enough!")

    def _assert_id(self, idx):
        for i in idx:
            assert self._env_stages[i] == -1, f"Cannot interact with environment {i} which is stepping now."

    def reset(self, level=None, idx=None):
        idx, slice_idx = self._process_idx(idx)
        self._env_stages[idx] = -1

        for i in range(len(idx)):
            self.workers[idx[i]].reset_no_render(level if is_num(level) or level is None else level[i])
        for i in range(len(idx)):
            self.workers[idx[i]].get_obs(sync=False)
        for i in range(len(idx)):
            self.workers[idx[i]].image_wait(mode="o")
        return self.reset_buffer.slice(slice_idx, wrapper=False)

    def step(self, actions, idx=None, rew_only=False):
        idx, slice_idx = self._process_idx(idx)
        import sapien

        wait_num = len(idx)

        with sapien.core.ProfilerBlock("step_async"):
            self._num_finished = 0
            self._env_stages[idx] = 0
            for i in range(len(idx)):
                self._env_flags[idx[i]] = self.workers[idx[i]].step_async(actions[i])

        with sapien.core.ProfilerBlock("call render-async"):
            render_jobs = []
            while self._num_finished < wait_num:
                for i in range(self.num_envs):
                    if self._env_stages[i] == 0 and self._env_flags[i].ready():
                        self._env_stages[i] += 1
                        self._num_finished += 1
                        if not rew_only:
                            self.workers[i].call_renderer_async(mode="o")
                            render_jobs.append(i)

        for i in render_jobs:
            self.workers[i].get_obs(sync=False)
        with sapien.core.ProfilerBlock("wait for render"):
            for i in render_jobs:
                self.workers[i].image_wait(mode="o")
            self._env_stages[idx] = -1

        if rew_only:
            return self.step_buffer[1][idx]

        infos = self.step_buffer.slice(slice_idx, wrapper=False)
        infos[-1] = self.single_env.deserialize_info(infos[-1])
        return infos

    def render(self, mode="rgb_array", idx=None):
        idx, slice_idx = self._process_idx(idx)
        if mode == "human":
            assert len(self.workers) == 1, "Human rendering only allows num_envs = 1!"
            return self.workers[0].render(mode)
        assert mode == "rgb_array", "We only support rgb_array mode for multiple environments!"
        [self.workers[i].call_renderer_async(mode="v") for i in idx]
        [self.workers[i].image_wait(mode="v") for i in idx]
        return self.vis_img_buffer.slice(slice_idx, wrapper=False)

    def step_random_actions(self, num):
        # For replay buffer warmup of the RL agent
        from .env_utils import true_done

        obs = self.reset(idx=np.arange(self.num_envs))
        num = int(num)
        ret = []
        while num > 0:
            num_i = min(num, self.num_envs)
            actions = self.action_space.sample()[:num_i]
            idx = np.arange(num_i, dtype=np.int32)
            next_obs, rewards, dones, infos = self.step(actions, idx=idx)
            ret_i = dict(
                obs=obs,
                next_obs=next_obs,
                actions=actions,
                rewards=rewards,
                dones=true_done(dones, infos),
                infos=infos,
                episode_dones=dones,
                worker_indices=idx,
            )
            ret.append(GDict(ret_i).to_array().copy(wrapper=False))
            obs = GDict(next_obs).copy(wrapper=False)
            num -= num_i
            if np.any(dones):
                self.reset(idx=np.where(dones[..., 0])[0])
        return DictArray.concat(ret, axis=0).to_two_dims(wrapper=False)

    def step_states_actions(self, states=None, actions=None):
        """
        Return shape: [N, LEN, 1]
        """
        # For MPC
        rewards = np.zeros_like(actions[..., :1], dtype=np.float32)
        for i in range(0, len(actions), self.num_envs):
            num_i = min(len(actions) - i, self.num_envs)
            if hasattr(self, "set_state") and states is not None:
                for j in range(num_i):
                    self.workers[j].set_state(states[i + j])

            for j in range(len(actions[i])):
                rewards[i : i + num_i, j] = self.step(actions[i : i + num_i, j], idx=np.arange(num_i), rew_only=True)
        return rewards

    def get_attr(self, name, idx=None):
        ret = GDict([getattr(self.workers[i], name) for i in idx]).to_array()
        return GDict.stack(ret, 0, wrapper=False)

    def call(self, name, idx=None, *args, **kwargs):
        args, kwargs = GDict(list(args)), GDict(dict(kwargs))
        ret = [getattr(self.workers[i], name)(*args.slice(i, 0, False), **kwargs.slice(i, 0, False)) for i in idx]
        ret = GDict(ret).to_array()
        return None if ret[0] is None else GDict.stack(ret, axis=0, wrapper=False)
