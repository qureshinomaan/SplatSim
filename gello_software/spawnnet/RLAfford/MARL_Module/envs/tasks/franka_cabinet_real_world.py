import json
import os
from time import time
from isaacgym import gymutil, gymtorch, gymapi
from isaacgym.torch_utils import *
import numpy as np
from random import shuffle, randint
import yaml
from tasks.franka_cabinet import OneFrankaCabinet
from MARL_Module.envs.utils.contact_buffer import ContactBuffer
from tqdm import tqdm

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class OneFrankaCabinetRealWorld(OneFrankaCabinet) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, log_dir=log_dir)

        self.cur_step = 0
        self.max_episode_length
        self.pose_buffer = torch.zeros((self.max_episode_length, 7+2), device=self.device)  # 7 for position and orientation, 2 for gripper
    
    def _franka_init_pose(self) :

        initial_franka_pose = gymapi.Transform()

        # in realtime experiment, we hard-coded the position of robot and object.
        initial_franka_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        initial_franka_pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
        return initial_franka_pose
    
    def _obj_init_pose(self, min_dict, max_dict) :

        cabinet_start_pose = gymapi.Transform()
        cabinet_start_pose.p = gymapi.Vec3(-0.7, 0.0, 0.0)
        cabinet_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        
        return cabinet_start_pose
        
    def _cam_pose(self) :

        cam_pos = gymapi.Vec3(6, 0.0, 6.0)
        cam_target = gymapi.Vec3(0, 0.0, 0.1)

        return cam_pos, cam_target
    
    def _get_reward_done(self) :

        door_pos = self.cabinet_door_rigid_body_tensor[:, :3]
        door_rot = self.cabinet_door_rigid_body_tensor[:, 3:7]
        hand_rot = self.hand_rigid_body_tensor[..., 3:7]
        hand_down_dir = quat_axis(hand_rot, 2)
        hand_grip_dir = quat_axis(hand_rot, 1)
        hand_sep_dir = quat_axis(hand_rot, 0)
        handle_pos = quat_apply(door_rot, self.cabinet_handle_pos_tensor) + door_pos
        handle_x = quat_axis(door_rot, 0) * self.cabinet_open_dir_tensor.view(-1, 1)
        handle_z = quat_axis(door_rot, 1)

        cabinet_door_relative_x = -handle_x

        if self.exp_name == "franka_cabinet_state_open_handle_custom" :
            franka_rfinger_pos = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index][:, 0:3] + hand_down_dir*0.05
            franka_lfinger_pos = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index][:, 0:3] + hand_down_dir*0.05
        else :
            franka_rfinger_pos = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index][:, 0:3] + hand_down_dir*0.075
            franka_lfinger_pos = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index][:, 0:3] + hand_down_dir*0.075

        if self.cfg["task"]["useDrawer"] == False :  # 

            # distance from hand to the handle
            d = torch.norm(self.hand_tip_pos - handle_pos, p=2, dim=-1)
            xy_dist = torch.norm(self.hand_tip_pos[:, :2] - handle_pos[:, :2], p=2, dim=-1)
            # reward for reaching the handle
            dist_reward = 1.0 / (1.0 + d**2)
            dist_reward *= dist_reward
            dist_reward = torch.where(d <= 0.12, dist_reward*2, dist_reward)
            # dist_reward = -torch.log(d+0.01)
            # dist_reward = torch.exp(-10 * d)

            dot1 = (hand_down_dir * handle_z).sum(dim=-1)
            dot2 = (hand_down_dir * handle_x).sum(dim=-1) * 0


            # reward for matching the orientation of the hand to the drawer (fingers wrapped)
            rot_reward = 0.5 * (torch.sign(dot1)*dot1**2 + torch.sign(dot2)*dot2**2)


            # bonus if two fingers are at different sides of the handle
            around_handle = torch.zeros_like(rot_reward).long()
            around_handle = torch.where((franka_rfinger_pos[:, 1] > handle_pos[:, 1]) & (franka_lfinger_pos[:, 1] < handle_pos[:, 1]),
                                                around_handle+1, around_handle)
            stage_2 = ((d<=0.12) & (xy_dist<=0.03) & around_handle)
            proj_rfinger = ((franka_rfinger_pos - handle_pos) * (-handle_z)).sum(dim=-1)
            proj_lfinger = ((franka_lfinger_pos - handle_pos) * (-handle_z)).sum(dim=-1)
            gripped = (stage_2 & (proj_rfinger>0) & (proj_lfinger<0))
            grip_reward = (- 0.005 * (self.eff_act[:, -3] + self.eff_act[:, -2]))

            self.stage = stage_2

            # regularization on the actions (summed for each environment)
            action_penalty = torch.sum((self.pos_act[:, :7]-self.franka_dof_tensor[:, :7, 0])**2, dim=-1)

            diff_from_success = (self.success_dof_states.view(self.cabinet_num, -1) - self.cabinet_dof_tensor_spec[:, :, 0]).view(-1)
            success = (diff_from_success < 0.01)

            # how far the cabinet has been opened out
            infront = (((self.hand_tip_pos-door_pos)*cabinet_door_relative_x).sum(dim=1) > 0)

            open_reward = self.cabinet_dof_tensor[:, 0]*10*(d<0.2)*infront
        
            self.rew_buf = 1.5 * dist_reward + 0.5 * rot_reward \
                + 1 * open_reward \
                + 0.1 * grip_reward - 0.75
        # open Drawer
        else:
            # distance from hand to the handle
            d = torch.norm(self.hand_tip_pos - handle_pos, p=2, dim=-1)
            xy_dist = torch.norm(self.hand_tip_pos[:, :2] - handle_pos[:, :2], p=2, dim=-1)
            # reward for reaching the handle
            dist_reward = 1.0 / (1.0 + d**2)
            dist_reward *= dist_reward
            dist_reward = torch.where(d <= 0.12, dist_reward*2, dist_reward)

            dot1 = (hand_grip_dir * handle_z).sum(dim=-1)
            dot2 = (-hand_sep_dir * handle_x).sum(dim=-1)

            # reward for matching the orientation of the hand to the drawer (fingers wrapped)
            rot_reward = 0.5 * (torch.sign(dot1)*dot1**2 + torch.sign(dot2)*dot2**2)

            # bonus if two fingers are at different sides of the handle
            around_handle_reward = torch.zeros_like(rot_reward)
            around_handle_reward = torch.where(franka_lfinger_pos[:, 1] > handle_pos[:, 1],
                                    torch.where(franka_rfinger_pos[:, 1] < handle_pos[:, 1],
                                                around_handle_reward + 0.5, around_handle_reward), around_handle_reward)

            # reward for distance of each finger from the handle
            finger_dist_reward = torch.zeros_like(rot_reward)
            stage_2 = (d<=0.12) 

            lfinger_dist = torch.norm(franka_lfinger_pos[:, 1] - handle_pos[:, 1], p=2, dim=-1)
            rfinger_dist = torch.norm(franka_rfinger_pos[:, 1] - handle_pos[:, 1], p=2, dim=-1)
            lrfinger_dist = torch.norm(franka_rfinger_pos - franka_lfinger_pos, p=2, dim=-1)
            self.stage = stage_2

            finger_dist_reward = torch.where(d<=0.2,
                                            torch.where(franka_lfinger_pos[:, 1] > handle_pos[:, 1],
                                                torch.where(franka_rfinger_pos[:, 1] < handle_pos[:, 1],
                                                    (0.1 - lfinger_dist - rfinger_dist)*10, finger_dist_reward), finger_dist_reward), finger_dist_reward)
            
            finger_dist_reward = torch.where(d<=0.2,
                                            torch.where(franka_rfinger_pos[:, 1] > handle_pos[:, 1],
                                                torch.where(franka_lfinger_pos[:, 1] < handle_pos[:, 1],
                                                    (0.1 - lfinger_dist - rfinger_dist)*10, finger_dist_reward), finger_dist_reward), finger_dist_reward)

            # regularization on the actions (summed for each environment)
            action_penalty = torch.sum((self.pos_act[:, :7]-self.franka_dof_tensor[:, :7, 0])**2, dim=-1)

            diff_from_success = (self.success_dof_states.view(self.cabinet_num, -1) - self.cabinet_dof_tensor_spec[:, :, 0]).view(-1)
            success = (diff_from_success < 0.01)
            
            # how far the cabinet has been opened out
            open_reward = self.cabinet_dof_tensor[:, 0]*10*(d<0.2)*(self.hand_tip_pos[:, 0]>door_pos[:, 0])
            self.rew_buf = 1.0 * dist_reward + 0.5 * rot_reward \
                + 2.0 * open_reward \
                + 0 * finger_dist_reward - 0.75

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

            self.pose_buffer[self.cur_step, :7] = self.hand_rigid_body_tensor[0, :7]
            self.pose_buffer[self.cur_step, 7:] = self.franka_dof_tensor[0, -2:, 0]
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