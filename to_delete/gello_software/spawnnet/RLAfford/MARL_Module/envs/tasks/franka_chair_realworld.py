import json
import os
from time import time
from isaacgym import gymutil, gymtorch, gymapi
from isaacgym.torch_utils import *
import numpy as np
from random import shuffle, randint
import yaml
from tasks.franka_chair import TwoFrankaChair
from MARL_Module.envs.utils.contact_buffer import ContactBuffer
from tqdm import tqdm
from MARL_Module.envs.utils.time_counter import TimeCounter, TimeCounterSesion

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class TwoFrankaChairRealWorld(TwoFrankaChair) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, log_dir=log_dir)

        self.cur_step = 0
        self.max_episode_length
        self.pose_buffer = torch.zeros((2, self.max_episode_length, 7+2), device=self.device)  # 7 for position and orientation, 2 for gripper
    
    def _franka0_pose(self) :

        initial_franka_pose_0 = gymapi.Transform()
        initial_franka_pose_0.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        initial_franka_pose_0.p = gymapi.Vec3(0.3, 0.4, 0.05)

        return initial_franka_pose_0

    def _franka1_pose(self) :

        initial_franka_pose_1 = gymapi.Transform()
        initial_franka_pose_1.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        initial_franka_pose_1.p = gymapi.Vec3(0.3, -0.4, 0.05)

        return initial_franka_pose_1
    
    def _cam_pose(self) :

        cam_pos = gymapi.Vec3(6, 0.0, 6.0)
        cam_target = gymapi.Vec3(0, 0.0, 0.1)

        return cam_pos, cam_target
    
    def _get_reward_done(self) :

        success = torch.zeros((self.env_num, ), device=self.device).long()

        self.rew_buf = torch.zeros((self.num_envs), device=self.device)

        chair_pos = self.chair_root_tensor[:, :3]
        hand_chair_dist_rew0 = -torch.sqrt(((self.hand_tip_pos0-chair_pos)**2).sum(dim=1))
        hand_chair_dist_rew1 = -torch.sqrt(((self.hand_tip_pos1-chair_pos)**2).sum(dim=1))
        hand_chair_dist_rew = torch.min(hand_chair_dist_rew0, hand_chair_dist_rew1)
        chair_dist_rew = -self.chair_root_tensor[:, 0] - 0.2

        chair_axis = quat_axis(self.chair_root_tensor[:, 3:7], 2)
        up_axis = torch.tensor([[0, 0, 1]], device=self.device)

        upright_rew = (chair_axis*up_axis).sum(dim=1) - 1.0

        # self._draw_line(chair_pos[0], chair_pos[0] + up_axis[0])

        self.rew_buf += hand_chair_dist_rew*0.2 + chair_dist_rew + upright_rew

        # self._draw_line(chair_pos[0], chair_pos[0]-torch.tensor([1,0,0.0], device=self.device))

        success = (self.chair_root_tensor[:, 0] < -0.5) & (upright_rew > -0.5)

        time_out = (self.progress_buf >= self.max_episode_length)
        self.reset_buf = (self.reset_buf | time_out)
        self.success_buf = self.success_buf | success
        self.success = self.success_buf & time_out

        old_coef = 1.0 - time_out*0.1
        new_coef = time_out*0.1

        self.success_rate = self.success_rate*old_coef + success*new_coef

        return self.rew_buf, self.reset_buf
    
    def step(self, actions) :

        ret = super().step(actions)

        if not self.train_mode :

            self.pose_buffer[0, self.cur_step, :7] = self.hand_rigid_body_tensor0[0, :7]
            self.pose_buffer[1, self.cur_step, :7] = self.hand_rigid_body_tensor1[0, :7]
            self.pose_buffer[:, self.cur_step, 7:] = self.franka_dof_tensor[0, :, -2:, 0]
            self.cur_step += 1
            if self.success_buf[0] :
                print("successed")
            if self.cur_step >= self.max_episode_length :
                self.save_eff_pose(os.path.join("./logs/", "pose", self.exp_name))
                exit()
        
        return ret
    
    def save_eff_pose(self, path) :

        if not os.path.exists(path) :
            print("folder: {} not exist, creating new folder.".format(path))
            os.mkdir(path)

        torch.save(self.pose_buffer, os.path.join(path, "pose.pt"))