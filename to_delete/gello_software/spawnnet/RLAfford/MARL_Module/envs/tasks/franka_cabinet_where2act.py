from cgitb import reset
from cmath import pi
from copy import copy, deepcopy
import enum
import os
import math
from re import L, S
from isaacgym import gymutil, gymtorch, gymapi
from isaacgym.torch_utils import *
import numpy as np
from random import shuffle
from torch import device, nonzero, rand
import torch.optim as optim
import torch.nn.functional as F
from gym.spaces.box import Box
from trimesh import PointCloud
from tasks.hand_base.base_task import BaseTask
from MARL_Module.envs.utils.contact_buffer import ContactBuffer
from algorithms.utils.model_3d import PointNet2Feature
from tasks.franka_cabinet import OneFrankaCabinet
from tasks.franka_cabinet_PC_partial import OneFrankaCabinetPCPartial
from open3d import io
from tqdm import tqdm

import ipdb
import time
import sys
import random
from Collision_Predictor_Module.CollisionPredictor.code.train_with_RL import CollisionPredictor
from Collision_Predictor_Module.where2act.code.pc_to_actionscore import where2act_net
from MARL_Module.envs.utils.gpu_mem_track import MemTracker


class OneFrankaCabinetPCWhere2act(OneFrankaCabinetPCPartial) :

	def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

		self.pointCloudDownsampleNum = cfg["env"]["pointDownsampleNum"]
		self.gpu_tracker = MemTracker()
		
		super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index, is_multi_agent, log_dir=log_dir)

		self.task_meta["mask_dim"] = 3
		self.task_meta['need_update'] = True
		self.num_feature = cfg["env"]["pointFeatureDim"]
		self.CP_iter = cfg['cp']['CP_iter']
		self.obs_buf = torch.zeros((self.num_envs, self.pointCloudDownsampleNum, self.num_obs), device=self.device, dtype=torch.float)

		self.CollisionPredictor = CollisionPredictor(self.cfg, self.log_dir)
		self.depth_bar = self.cfg["env"]["depth_bar"]
		self.raw_map = torch.zeros((self.env_num, self.pointCloudDownsampleNum, 4), device=self.device)
		self.where2act = where2act_net(self.device)

	def quat_apply(self, a, b):
		shape = b.shape
		a = a.reshape(-1, 4)  # 4
		a_expand = a.expand(shape[0], 4)
		b = b.reshape(-1, 3)  # num_buffer*3
		xyz = a_expand[:, :3]   # 3
		t = xyz.cross(b, dim=-1) * 2
		return (b + a_expand[:, 3:] * t + xyz.cross(t, dim=-1)).view(shape)

	def get_map(self, raw_point_clouds, raw_mask):

		top_tensor = torch.tensor([x.top for x in self.contact_buffer_list], device=self.device)
		map_dis_bar = self.cfg['env']['map_dis_bar']
		# ipdb.set_trace()
		buffer_size = (self.contact_buffer_list[-1]).buffer.shape[0]
		buffer = torch.zeros((self.num_envs, buffer_size, 3)).to(self.device)
		for i in range(self.num_envs):
			buffer_id = i // self.env_per_cabinet
			buffer_to_door = self.contact_buffer_list[buffer_id].buffer[:, 0:3]
			door_pos = self.cabinet_door_rigid_body_tensor[i, 0:3]
			door_rot = self.cabinet_door_rigid_body_tensor[i, 3:7]
			buffer_to_env = door_pos + self.quat_apply(door_rot.view(1, -1), buffer_to_door).view(buffer_to_door.shape[0], -1)
			buffer[i] = buffer_to_env

		pc_square = torch.mul(raw_point_clouds,raw_point_clouds)  # env * pc * 3
		buffer_square = buffer*buffer  # env * bf * 3
		pc_buffer_dot = raw_point_clouds@(buffer.permute((0, 2, 1)).to(self.device))  # env*pc*bf
		# ipdb.set_trace()
		dis_2 = pc_square.sum(dim=2).reshape(self.num_envs, pc_square.shape[1], 1)\
				+buffer_square.sum(dim=2).reshape(self.num_envs, 1, buffer_square.shape[1])\
				-2*pc_buffer_dot
		# if within a ball of radius=map_dis_bar
		if_eff = dis_2<map_dis_bar**2

		# for all the empty buffer we ignore them 
		for i in range(self.num_envs):
			if_eff[i, :, top_tensor[i // self.env_per_cabinet]:] = False

		tot = if_eff.sum(dim=2)
		# tot = torch.log(tot+1)
		tot_scale = tot/(tot.max()+1e-8)  # env*pc
		tot_scale = tot_scale * raw_mask

		heat_map = torch.cat((raw_point_clouds, tot_scale.reshape(tot_scale.shape[0], tot_scale.shape[1], 1)), dim=2)  # env*pc*5
		# ipdb.set_trace()
		return heat_map

	def save(self, path, iteration) :

		super().save(path, iteration)
		torch.save(self.map, os.path.join(path, "map_{}.pt".format(iteration)))
	
	def load(self, path, iteration) :

		self.CollisionPredictor.load_checkpoint(os.path.join(path, "CP_{}.pt".format(iteration)))

	def _data_argumentation(self, pcd):
		pcd[:, :, :3] *= torch.rand((pcd.shape[0], 1, 3), device=self.device)*0.3 + 0.85
		return pcd

	def update(self, iter) :
		pass
	
	def _get_max_point(self, map) :

		env_max = map[:, :, 3].max(dim=-1)[0]
		weight = torch.where(map[:, :, 3] > env_max.view(self.env_num, 1)-0.1, 1, 0)
		weight_reshaped = weight.view(self.env_num, -1, 1)
		mean = (map[:, :, :3]*weight_reshaped).mean(dim=1)
		return mean

	def _get_reward_done(self) :

		rew, res = super()._get_reward_done()

		d = torch.norm(self.hand_tip_pos - self._get_max_point(self.map), p=2, dim=-1)
		dist_reward = 1.0 / (1.0 + d**2)
		dist_reward *= dist_reward
		dist_reward = torch.where(d <= 0.1, dist_reward*2, dist_reward)
		rew += dist_reward * self.cfg['cp']['max_point_reward']

		return rew, res

	def _refresh_observation(self) :
		self.gym.render_all_camera_sensors(self.sim)
		self.gym.start_access_image_tensors(self.sim)
		self.gym.refresh_actor_root_state_tensor(self.sim)
		self.gym.refresh_dof_state_tensor(self.sim)
		self.gym.refresh_rigid_body_state_tensor(self.sim)
		if self.cfg["env"]["driveMode"] == "ik" :
			self.gym.refresh_jacobian_tensors(self.sim)

		# create new map using information from other envs
		point_clouds = self.compute_point_cloud_state()

		if self.pointCloudVisualizer != None :
			self._refresh_pointcloud_visualizer(point_clouds[0, :, :3], self.raw_map[0, :, 3])

		with torch.no_grad() :
			self.map = self.where2act.predict_affordance(point_clouds[:, :, :3])
			self.map = self.map.to(self.device)

		point_clouds[:, :, :3] -= self.franka_root_tensor[:, :3].view(-1, 1, 3)
		self.obs_buf[:, :, :5].copy_(point_clouds)
		self.obs_buf[:, :, 5].copy_(self.map[:, :, 3] * point_clouds[:, :, 3])
		if self.cfg["cp"]["max_point_observation"] :
			self.obs_buf[:, :, 6:] = self._get_base_observation(self._get_max_point(self.map)).view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
		else :
			self.obs_buf[:, :, 6:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)