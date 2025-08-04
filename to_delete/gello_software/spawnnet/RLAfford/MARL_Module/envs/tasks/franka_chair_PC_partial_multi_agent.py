from isaacgym.torch_utils import *
import numpy as np
from tasks.franka_chair_PC_partial import TwoFrankaChairPCPartial
import matplotlib.pyplot as plt
from pointnet2_ops import pointnet2_utils
from tqdm import tqdm
from MARL_Module.envs.utils.time_counter import TimeCounter
import os

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class TwoFrankaChairPCPartialMultiAgent(TwoFrankaChairPCPartial) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[0, 1], is_multi_agent=False, log_dir=None):

        self.num_agents = 2
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index, is_multi_agent, log_dir)

        self.num_hand_obs = (self.franka_num_dofs-2)*3 + 13
        self.num_shared_obs = 25 + 5 # 5 for pointcloud
        self.num_obs = self.num_hand_obs + self.num_shared_obs
        self.task_meta["obs_dim"] = self.num_obs
        agent_obs_shape = (self.num_envs, self.num_agents, self.pointCloudDownsampleNum, self.num_obs)
        self.agent_obs_buf = torch.zeros(agent_obs_shape, device=self.device)

    def _refresh_observation(self) :
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        if self.cfg["env"]["driveMode"] == "ik" :
            self.gym.refresh_jacobian_tensors(self.sim)

        point_clouds = self.compute_point_cloud_state(pc=self.sampled_chair_pc)

        point_clouds[:, :, :3] -= self.initial_root_states[:, 2, :3].view(self.num_envs, 1, 3)
        self.obs_buf[:, :, :-5] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
        self.obs_buf[:, :, -5:].copy_(point_clouds)

        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(
                [point_clouds[0, :, :3], self.contact_buffer_list0[0].all()[:, :3]],
                [point_clouds[0, :, 3], torch.ones((self.contact_buffer_list0[0].top,)) * 0.5]
            )
    
    def translate_observation(self, observation) :

        ret = torch.cat(
            (
                torch.cat((
                    observation[:, :, :self.num_hand_obs],
                    observation[:, :, self.num_hand_obs*2:]
                ), dim=2).reshape(self.num_envs, 1, self.pointCloudDownsampleNum, -1),
                torch.cat((
                    observation[:, :, self.num_hand_obs:self.num_hand_obs*2],
                    observation[:, :, self.num_hand_obs*2:]
                ), dim=2).reshape(self.num_envs, 1, self.pointCloudDownsampleNum, -1)
            ), dim=1
        )

        return ret

    def translate_action(self, action) :

        action = torch.stack(action, dim=1)
        
        torch.transpose(action, 0, 1)

        return torch.cat(
            (
                action[:, 0, :],
                action[:, 1, :]
            ), dim=1
        )

    def reset(self, to_reset="all") :

        obs, rew, done, info = super().reset(to_reset)

        self.agent_obs_buf.copy_(self.translate_observation(obs))
        
        return self.agent_obs_buf, self.agent_obs_buf, None

    def step(self, action) :

        obs, rew, done, info = super().step(self.translate_action(action))

        self.agent_obs_buf.copy_(self.translate_observation(obs))

        return self.agent_obs_buf, self.agent_obs_buf, rew.view(-1, 1, 1).repeat_interleave(self.num_agents, 1), done.view(-1, 1).repeat_interleave(self.num_agents, 1), info, None

