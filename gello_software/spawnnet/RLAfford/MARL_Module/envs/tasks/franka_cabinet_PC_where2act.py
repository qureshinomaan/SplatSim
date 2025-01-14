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

        self.cabinet_mask_dim = 5
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index, is_multi_agent, log_dir=log_dir)

        self.num_obs += 1
        self.task_meta["mask_dim"] = 3
        self.task_meta["obs_dim"] = self.num_obs
        self.task_meta['need_update'] = True
        self.num_feature = cfg["env"]["pointFeatureDim"]
        self.obs_buf = torch.zeros((self.num_envs, self.pointCloudDownsampleNum, self.num_obs), device=self.device, dtype=torch.float)

        self.where2act = where2act_net(self.device, cfg["env"]["where2actCheckpoint"])
    
    def _refresh_observation(self) :
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        if self.cfg["env"]["driveMode"] == "ik" :
            self.gym.refresh_jacobian_tensors(self.sim)

        point_clouds = self.compute_point_cloud_state(pc=self.sampled_cabinet_pc)

        point_clouds[:, :, :3] -= self.franka_root_tensor[:, :3].view(-1, 1, 3)

        # Call the predictor of where2act
        where2act_map = self.where2act.predict_affordance(point_clouds[:, :, :3])

        self.obs_buf[:, :, :5].copy_(point_clouds)
        self.obs_buf[:, :, 5].copy_(where2act_map * point_clouds[:, :, 3])
        self.obs_buf[:, :, 6:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)

        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(
                [point_clouds[0, :, :3]],
                [where2act_map[0]]
            )