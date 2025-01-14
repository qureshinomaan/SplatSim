import pathlib
from collections import deque

import gym
import numpy as np

from mani_skill_learn.env import get_env_info
from mani_skill_learn.env.observation_process import process_mani_skill_base
from mani_skill_learn.methods.builder import build_brl
from mani_skill_learn.utils.data import to_np, unsqueeze
from mani_skill_learn.utils.meta import Config
from mani_skill_learn.utils.torch import load_checkpoint


class ObsProcess:
    # modified from SapienRLWrapper
    def __init__(self, env, obs_mode, stack_frame=1):
        """
        Stack k last frames for point clouds or rgbd
        """
        self.env = env
        self.obs_mode = obs_mode
        self.stack_frame = stack_frame
        self.buffered_data = {}

    def _update_buffer(self, obs):
        for key in obs:
            if key not in self.buffered_data:
                self.buffered_data[key] = deque([obs[key]] * self.stack_frame, maxlen=self.stack_frame)
            else:
                self.buffered_data[key].append(obs[key])

    def _get_buffer_content(self):
        axis = 0 if self.obs_mode == 'pointcloud' else -1
        return {key: np.concatenate(self.buffered_data[key], axis=axis) for key in self.buffered_data}

    def process_observation(self, observation):
        if self.obs_mode == "state":
            return observation
        observation = process_mani_skill_base(observation, self.env)
        visual_data = observation[self.obs_mode]
        self._update_buffer(visual_data)
        visual_data = self._get_buffer_content()
        state = observation['agent']
        # Convert dict of array to list of array with sorted key
        ret = {}
        ret[self.obs_mode] = visual_data
        ret['state'] = state
        return ret


class BasePolicy(object):
    def __init__(self, opts=None):
        self.obs_mode = 'pointcloud'

    def act(self, observation):
        raise NotImplementedError()

    def reset(self):  # if you use an RNN-based policy, you need to implement this function
        pass


class UserPolicy(BasePolicy):
    def __init__(self, env_name):
        super().__init__()
        self.env = gym.make(env_name)
        self.obs_mode = 'pointcloud'  # remember to set this!
        self.env.set_env_mode(obs_mode=self.obs_mode)
        self.stack_frame = 1

        cfg_path = str(pathlib.Path('./configs/bc/mani_skill_point_cloud_transformer.py').resolve())
        cfg = Config.fromfile(cfg_path)
        cfg.env_cfg['env_name'] = env_name
        obs_shape, action_shape, action_space = get_env_info(cfg.env_cfg)
        cfg.agent['obs_shape'] = obs_shape
        cfg.agent['action_shape'] = action_shape
        cfg.agent['action_space'] = action_space

        self.agent = build_brl(cfg.agent)
        load_checkpoint(self.agent,
            str(pathlib.Path('./example_mani_skill_data/OpenCabinetDrawer_1045_link_0-v0_PN_Transformer.ckpt').resolve()),
            map_location='cpu'
        )
        self.agent.to('cuda')  # dataparallel not done here
        self.agent.eval()

        self.obsprocess = ObsProcess(self.env, self.obs_mode, self.stack_frame)

    def act(self, observation):
        ##### Replace with your code
        observation = self.obsprocess.process_observation(observation)
        return to_np(self.agent(unsqueeze(observation, axis=0), mode='eval'))[0]
