import json
import os
from this import d
from time import time
from isaacgym import gymutil, gymtorch, gymapi
from isaacgym.torch_utils import *
import numpy as np
from random import shuffle, randint
import yaml
from tasks.hand_base.base_task import BaseTask
from MARL_Module.envs.utils.contact_buffer import ContactBuffer
from tqdm import tqdm

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class TwoFrankaChair(BaseTask) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.cfg = cfg
        self.sim_params = sim_params
        self.physics_engine = physics_engine
        self.agent_index = agent_index
        self.is_multi_agent = is_multi_agent
        self.log_dir = log_dir
        self.up_axis = 'z'
        self.cfg["device_type"] = device_type
        self.cfg["device_id"] = device_id
        self.cfg["headless"] = headless
        self.device_type = device_type
        self.device_id = device_id
        self.headless = headless
        self.device = "cpu"
        self.use_handle = False
        if self.device_type == "cuda" or self.device_type == "GPU":
            self.device = "cuda" + ":" + str(self.device_id)
        self.max_episode_length = self.cfg["env"]["maxEpisodeLength"]
        
        self.env_num_train = cfg["env"]["numTrain"]
        self.env_num_val = cfg["env"]["numVal"]
        self.env_num = self.env_num_train + self.env_num_val
        self.asset_root = cfg["env"]["asset"]["assetRoot"]
        self.chair_num_train = cfg["env"]["asset"]["assetNumTrain"]
        self.chair_num_val = cfg["env"]["asset"]["assetNumVal"]
        self.chair_num = self.chair_num_train+self.chair_num_val
        chair_train_list_len = len(cfg["env"]["asset"]["trainAssets"])
        chair_val_list_len = len(cfg["env"]["asset"]["testAssets"])
        self.chair_train_name_list = []
        self.chair_val_name_list = []
        self.exp_name = cfg['env']["env_name"]
        print("Simulator: number of chairs", self.chair_num)
        print("Simulator: number of environments", self.env_num)
        if self.chair_num_train :
            assert(self.env_num_train % self.chair_num_train == 0)
        if self.chair_num_val :
            assert(self.env_num_val % self.chair_num_val == 0)
        assert(self.env_num_train*self.chair_num_val == self.env_num_val*self.chair_num_train)
        assert(self.chair_num_train <= chair_train_list_len)    # the number of used length must less than real length
        assert(self.chair_num_val <= chair_val_list_len)    # the number of used length must less than real length
        assert(self.env_num % self.chair_num == 0)    # each chair should have equal number envs
        self.env_per_chair = self.env_num // self.chair_num
        self.task_meta = {
            "training_env_num": self.env_num_train,
            "valitating_env_num": self.env_num_val,
            "need_update": True,
            "max_episode_length": self.max_episode_length,
            "obs_dim": cfg["env"]["numObservations"]
        }
        for name in cfg["env"]["asset"]["trainAssets"] :
            self.chair_train_name_list.append(cfg["env"]["asset"]["trainAssets"][name]["name"])
        for name in cfg["env"]["asset"]["testAssets"] :
            self.chair_val_name_list.append(cfg["env"]["asset"]["testAssets"][name]["name"])
        
        if "useTaskId" in self.cfg["task"] and self.cfg["task"]["useTaskId"]:
            self.task_meta["training_env_num"] += self.task_meta["valitating_env_num"]
            self.task_meta["valitating_env_num"] = 0
            self.chair_train_name_list += self.chair_val_name_list
            self.chair_val_name_list = []

        self.env_ptr_list = []
        self.obj_loaded = False
        self.franka_loaded = False

        self.use_handle = cfg["task"]["useHandle"]
        self.use_stage = cfg["task"]["useStage"]
        self.use_slider = cfg["task"]["useSlider"]

        if "useTaskId" in self.cfg["task"] and self.cfg["task"]["useTaskId"]:
            self.cfg["env"]["numObservations"] += self.chair_num

        super().__init__(cfg=self.cfg, enable_camera_sensors=cfg["env"]["enableCameraSensors"])

        # acquire tensors
        self.root_tensor = gymtorch.wrap_tensor(self.gym.acquire_actor_root_state_tensor(self.sim))
        self.dof_state_tensor = gymtorch.wrap_tensor(self.gym.acquire_dof_state_tensor(self.sim))
        self.rigid_body_tensor = gymtorch.wrap_tensor(self.gym.acquire_rigid_body_state_tensor(self.sim))
        if self.cfg["env"]["driveMode"] == "ik" :    # inverse kinetic needs jacobian tensor, other drive mode don't need
            self.jacobian_tensor = gymtorch.wrap_tensor(self.gym.acquire_jacobian_tensor(self.sim, "franka"))
        if not hasattr(self, 'num_agents') :
            self.num_agents = 1

        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)

        self.root_tensor = self.root_tensor.view(self.num_envs, -1, 13)
        self.dof_state_tensor = self.dof_state_tensor.view(self.num_envs, -1, 2)
        self.rigid_body_tensor = self.rigid_body_tensor.view(self.num_envs, -1, 13)
        self.actions = torch.zeros((self.num_envs, self.num_actions*self.num_agents), device=self.device)

        self.initial_dof_states = self.dof_state_tensor.clone()
        self.initial_root_states = self.root_tensor.clone()
        self.initial_rigid_body_states = self.rigid_body_tensor.clone()

        # precise slices of tensors
        env_ptr = self.env_ptr_list[0]
        franka_actor_0 = self.franka_actor_list[0]
        franka_actor_1 = self.franka_actor_list[1]
        chair_actor = self.chair_actor_list[0]
        self.hand_rigid_body_index = [
            self.gym.find_actor_rigid_body_index(
                env_ptr,
                franka_actor_0,
                "panda_hand",
                gymapi.DOMAIN_ENV
            ),
            self.gym.find_actor_rigid_body_index(
                env_ptr,
                franka_actor_1,
                "panda_hand",
                gymapi.DOMAIN_ENV
            )
        ]
        self.hand_lfinger_rigid_body_index = [
            self.gym.find_actor_rigid_body_index(
                env_ptr,
                franka_actor_0,
                "panda_leftfinger",
                gymapi.DOMAIN_ENV
            ),
            self.gym.find_actor_rigid_body_index(
                env_ptr,
                franka_actor_1,
                "panda_leftfinger",
                gymapi.DOMAIN_ENV
            )
        ]
        self.hand_rfinger_rigid_body_index = [
            self.gym.find_actor_rigid_body_index(
                env_ptr,
                franka_actor_0,
                "panda_rightfinger",
                gymapi.DOMAIN_ENV
            ),
            self.gym.find_actor_rigid_body_index(
                env_ptr,
                franka_actor_1,
                "panda_rightfinger",
                gymapi.DOMAIN_ENV
            )
        ]
        self.hand_rigid_body_tensor0 = self.rigid_body_tensor[:, self.hand_rigid_body_index[0], :]
        self.hand_rigid_body_tensor1 = self.rigid_body_tensor[:, self.hand_rigid_body_index[1], :]
        self.franka_dof_tensor = self.dof_state_tensor.view(self.num_envs, 2, -1, 2)
        self.franka_root_tensor = self.root_tensor[:, :2, :]
        self.chair_root_tensor = self.root_tensor[:, 2, :]

        self.dof_dim = self.franka_num_dofs*2
        self.pos_act = torch.zeros((self.num_envs, self.dof_dim), device=self.device)
        self.eff_act = torch.zeros((self.num_envs, self.dof_dim), device=self.device)

        # init collision buffer
        self.contact_buffer_size = cfg["env"]["contactBufferSize"]
        self.contact_moving_threshold = cfg["env"]["contactMovingThreshold"]
        self.map_dis_bar = cfg["env"]["map_dis_bar"]
        self.action_speed_scale = cfg["env"]["actionSpeedScale"]
        self.contact_buffer_list0 = []
        self.contact_buffer_list1 = []
        
        # assert(self.contact_buffer_size >= self.env_per_chair)
        for i in range(self.chair_num) :
            self.contact_buffer_list0.append(ContactBuffer(self.contact_buffer_size, 8, device=self.device))
            self.contact_buffer_list1.append(ContactBuffer(self.contact_buffer_size, 8, device=self.device))
        
        # params of randomization
        self.chair_reset_position_noise = cfg["env"]["reset"]["chair"]["resetPositionNoise"]
        self.chair_reset_rotation_noise = cfg["env"]["reset"]["chair"]["resetRotationNoise"]
        self.franka_reset_position_noise = cfg["env"]["reset"]["franka"]["resetPositionNoise"]
        self.franka_reset_rotation_noise = cfg["env"]["reset"]["franka"]["resetRotationNoise"]
        self.franka_reset_dof_pos_interval = cfg["env"]["reset"]["franka"]["resetDofPosRandomInterval"]
        self.franka_reset_dof_vel_interval = cfg["env"]["reset"]["franka"]["resetDofVelRandomInterval"]

        # params for success rate
        self.success = torch.zeros((self.env_num,), device=self.device)
        self.success_rate = torch.zeros((self.env_num,), device=self.device)
        self.success_buf = torch.zeros((self.env_num,), device=self.device).long()

        self.average_reward = None

        # flags for switching between training and evaluation mode
        self.train_mode = True

    def create_sim(self):
        self.dt = self.sim_params.dt
        self.up_axis_idx = self.set_sim_params_up_axis(self.sim_params, self.up_axis)

        self.sim = super().create_sim(self.device_id, self.graphics_device_id, self.physics_engine, self.sim_params)
        self._create_ground_plane()
        self._place_agents(self.cfg["env"]["numTrain"]+self.cfg["env"]["numVal"], self.cfg["env"]["envSpacing"])
    
    def _franka0_pose(self) :

        initial_franka_pose_0 = gymapi.Transform()
        initial_franka_pose_0.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        initial_franka_pose_0.p = gymapi.Vec3(0.5, 0.4, 0.05)

        return initial_franka_pose_0
    
    def _franka1_pose(self) :

        initial_franka_pose_1 = gymapi.Transform()
        initial_franka_pose_1.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        initial_franka_pose_1.p = gymapi.Vec3(0.5, -0.4, 0.05)

        return initial_franka_pose_1

    def _load_franka(self, env_ptr, env_id):

        if self.franka_loaded == False :

            self.franka_actor_list = []

            asset_root = self.asset_root
            asset_file = "franka_description/robots/franka_panda_longer.urdf"
            if self.use_slider :
                asset_file = "franka_description/robots/franka_panda_slider_longer.urdf"
            asset_options = gymapi.AssetOptions()
            asset_options.fix_base_link = True
            asset_options.disable_gravity = True
            # Switch Meshes from Z-up left-handed system to Y-up Right-handed coordinate system.
            asset_options.flip_visual_attachments = True
            asset_options.armature = 0.01
            self.franka_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)

            self.franka_left_finger_pc = torch.load(os.path.join(self.asset_root, "franka_description/point_cloud/left_finger_long"), map_location=self.device)
            self.franka_right_finger_pc = torch.load(os.path.join(self.asset_root, "franka_description/point_cloud/right_finger_long"), map_location=self.device)    
            self.franka_left_finger_pc = self.franka_left_finger_pc.view(1, -1, 3).repeat_interleave(self.env_num, dim=0)
            self.franka_right_finger_pc = self.franka_right_finger_pc.view(1, -1, 3).repeat_interleave(self.env_num, dim=0)

            self.franka_loaded = True

        franka_dof_max_torque, franka_dof_lower_limits, franka_dof_upper_limits = self._get_dof_property(self.franka_asset)
        self.franka_dof_max_torque_tensor = torch.tensor(franka_dof_max_torque, device=self.device)
        self.franka_dof_mean_limits_tensor = torch.tensor((franka_dof_lower_limits + franka_dof_upper_limits)/2, device=self.device)
        self.franka_dof_limits_range_tensor = torch.tensor((franka_dof_upper_limits - franka_dof_lower_limits)/2, device=self.device)
        self.franka_dof_lower_limits_tensor = torch.tensor(franka_dof_lower_limits, device=self.device)
        self.franka_dof_upper_limits_tensor = torch.tensor(franka_dof_upper_limits, device=self.device)

        dof_props = self.gym.get_asset_dof_properties(self.franka_asset)

        # use position drive for all dofs
        if self.cfg["env"]["driveMode"] in ["pos", "ik"]:
            dof_props["driveMode"][:-2].fill(gymapi.DOF_MODE_POS)
            dof_props["stiffness"][:-2].fill(400.0)
            dof_props["damping"][:-2].fill(40.0)
        else:       # osc
            dof_props["driveMode"][:-2].fill(gymapi.DOF_MODE_EFFORT)
            dof_props["stiffness"][:-2].fill(0.0)
            dof_props["damping"][:-2].fill(0.0)
        # grippers
        dof_props["driveMode"][-2:].fill(gymapi.DOF_MODE_EFFORT)
        dof_props["stiffness"][-2:].fill(0.0)
        dof_props["damping"][-2:].fill(0.0)

        # root pose
        initial_franka_pose_0 = self._franka0_pose()
        initial_franka_pose_1 = self._franka1_pose()

        # set start dof
        self.franka_num_dofs = self.gym.get_asset_dof_count(self.franka_asset)
        default_dof_pos = np.zeros(self.franka_num_dofs, dtype=np.float32)
        default_dof_pos[:-2] = (franka_dof_lower_limits + franka_dof_upper_limits)[:-2] * 0.3
        # grippers open
        default_dof_pos[-2:] = franka_dof_upper_limits[-2:]
        franka_dof_state = np.zeros_like(franka_dof_max_torque, gymapi.DofState.dtype)
        franka_dof_state["pos"] = default_dof_pos

        franka_actor_0 = self.gym.create_actor(
            env_ptr,
            self.franka_asset, 
            initial_franka_pose_0,
            "franka0",
            env_id,
            2,
            0
        )
        
        franka_actor_1 = self.gym.create_actor(
            env_ptr,
            self.franka_asset, 
            initial_franka_pose_1,
            "franka1",
            env_id,
            2,
            0
        )
        
        self.gym.set_actor_dof_properties(env_ptr, franka_actor_0, dof_props)
        self.gym.set_actor_dof_properties(env_ptr, franka_actor_1, dof_props)
        self.gym.set_actor_dof_states(env_ptr, franka_actor_0, franka_dof_state, gymapi.STATE_ALL)
        self.gym.set_actor_dof_states(env_ptr, franka_actor_1, franka_dof_state, gymapi.STATE_ALL)
        self.franka_actor_list.append(franka_actor_0)
        self.franka_actor_list.append(franka_actor_1)

    def _get_dof_property(self, asset) :
        dof_props = self.gym.get_asset_dof_properties(asset)
        dof_num = self.gym.get_asset_dof_count(asset)
        dof_lower_limits = []
        dof_upper_limits = []
        dof_max_torque = []
        for i in range(dof_num) :
            dof_max_torque.append(dof_props['effort'][i])
            dof_lower_limits.append(dof_props['lower'][i])
            dof_upper_limits.append(dof_props['upper'][i])
        dof_max_torque = np.array(dof_max_torque)
        dof_lower_limits = np.array(dof_lower_limits)
        dof_upper_limits = np.array(dof_upper_limits)
        return dof_max_torque, dof_lower_limits, dof_upper_limits
    
    def _load_obj_asset(self) :

        self.chair_asset_name_list = []
        self.chair_asset_list = []
        self.chair_pose_list = []
        self.chair_actor_list = []
        self.chair_pc = []

        train_len = len(self.cfg["env"]["asset"]["trainAssets"].items())
        val_len = len(self.cfg["env"]["asset"]["testAssets"].items())
        train_len = min(train_len, self.chair_num_train)
        val_len = min(val_len, self.chair_num_val)
        total_len = train_len + val_len
        used_len = min(total_len, self.chair_num)

        random_asset = self.cfg["env"]["asset"]["randomAsset"]
        select_train_asset = [i for i in range(train_len)]
        select_val_asset = [i for i in range(val_len)]
        if random_asset :        # if we need random asset list from given dataset, we shuffle the list to be read
            shuffle(select_train_asset)
            shuffle(select_val_asset)
        select_train_asset = select_train_asset[:train_len]
        select_val_asset = select_val_asset[:val_len]

        self.chair_min_tensor = torch.zeros((self.chair_num, 3), device=self.device)
        self.chair_max_tensor = torch.zeros((self.chair_num, 3), device=self.device)

        with tqdm(total=used_len) as pbar:
            pbar.set_description('Loading chair assets:')
            cur = 0

            asset_list = []

            # prepare the assets to be used
            for id, (name, val) in enumerate(self.cfg["env"]["asset"]["trainAssets"].items()) :
                if id in select_train_asset :
                    asset_list.append((id, (name, val)))
            for id, (name, val) in enumerate(self.cfg["env"]["asset"]["testAssets"].items()) :
                if id in select_val_asset :
                    asset_list.append((id, (name, val)))

            for id, (name, val) in asset_list :
            
                self.chair_asset_name_list.append(name)

                asset_options = gymapi.AssetOptions()
                asset_options.fix_base_link = False
                asset_options.disable_gravity = False
                asset_options.collapse_fixed_joints = True
                asset_options.use_mesh_materials = True
                asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX
                asset_options.override_com = True
                asset_options.override_inertia = True
                asset_options.vhacd_enabled = True
                asset_options.vhacd_params = gymapi.VhacdParams()
                asset_options.vhacd_params.resolution = 512

                chair_asset = self.gym.load_asset(self.sim, self.asset_root, val["path"], asset_options)
                self.chair_asset_list.append(chair_asset)

                # print(os.path.join(self.asset_root, val["boundingBox"]))
                with open(os.path.join(self.asset_root, val["boundingBox"]), "r") as f :
                    chair_bounding_box = json.load(f)
                    min_dict = chair_bounding_box["min"]
                    max_dict = chair_bounding_box["max"]

                self.chair_min_tensor[id, :] = torch.tensor([min_dict[0], min_dict[2], min_dict[1]], device=self.device)
                self.chair_max_tensor[id, :] = torch.tensor([max_dict[0], max_dict[2], max_dict[1]], device=self.device)
                chair_start_pose = gymapi.Transform()
                chair_start_pose.p = gymapi.Vec3(-max_dict[2], 0.0, -min_dict[1]+0.01)
                chair_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
                self.chair_pose_list.append(chair_start_pose)

                dataset_path = self.cfg["env"]["asset"]["datasetPath"]

                self.chair_pc.append(torch.load(os.path.join(self.asset_root, dataset_path, name, "point_clouds", "pointcloud_tensor"), map_location=self.device))
                
                pbar.update(1)
                cur += 1
            
        self.chair_pc = torch.stack(self.chair_pc).float()

    def _load_obj(self, env_ptr, env_id):

        if self.obj_loaded == False :

            self._load_obj_asset()

            self.obj_loaded = True

        chair_type = env_id // self.env_per_chair
        subenv_id = env_id % self.env_per_chair
        obj_actor = self.gym.create_actor(
            env_ptr,
            self.chair_asset_list[chair_type],
            self.chair_pose_list[chair_type],
            "chair{}-{}".format(chair_type, subenv_id),
            env_id,
            1,
            0
        )
        self.chair_actor_list.append(obj_actor)

    def _place_agents(self, env_num, spacing):

        print("Simulator: creating agents")

        lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        upper = gymapi.Vec3(spacing, spacing, spacing)
        self.space_middle = torch.zeros((env_num, 3), device=self.device)
        self.space_range = torch.zeros((env_num, 3), device=self.device)
        self.space_middle[:, 0] = self.space_middle[:, 1] = 0
        self.space_middle[:, 2] = spacing/2
        self.space_range[:, 0] = self.space_range[:, 1] = spacing
        self.space_middle[:, 2] = spacing/2
        num_per_row = int(np.sqrt(env_num))

        with tqdm(total=env_num) as pbar:
            pbar.set_description('Enumerating envs:')
            for env_id in range(env_num) :
                env_ptr = self.gym.create_env(self.sim, lower, upper, num_per_row)
                self.env_ptr_list.append(env_ptr)
                self._load_franka(env_ptr, env_id)
                self._load_obj(env_ptr, env_id)
                pbar.update(1)
        
    def _create_ground_plane(self) :
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        plane_params.static_friction = 0.1
        plane_params.dynamic_friction = 0.1
        self.gym.add_ground(self.sim, plane_params)
    
    def _get_reward_done(self) :

        success = torch.zeros((self.env_num, ), device=self.device).long()

        self.rew_buf = torch.zeros((self.num_envs), device=self.device)

        chair_pos = self.chair_root_tensor[:, :3]
        hand_chair_dist_rew0 = -torch.sqrt(((self.hand_tip_pos0-chair_pos)**2).sum(dim=1))
        hand_chair_dist_rew1 = -torch.sqrt(((self.hand_tip_pos1-chair_pos)**2).sum(dim=1))
        hand_chair_dist_rew = torch.max(hand_chair_dist_rew0, hand_chair_dist_rew1)
        chair_dist_rew = -self.chair_root_tensor[:, 0] - 0.2

        chair_axis = quat_axis(self.chair_root_tensor[:, 3:7], 2)
        up_axis = torch.tensor([[0, 0, 1]], device=self.device)

        upright_rew = (chair_axis*up_axis).sum(dim=1) - 1.0

        self.rew_buf += hand_chair_dist_rew*0.2 + chair_dist_rew + upright_rew

        success = (self.chair_root_tensor[:, 0] < -0.5) & (upright_rew > -0.5)

        time_out = (self.progress_buf >= self.max_episode_length)
        self.reset_buf = (self.reset_buf | time_out)
        self.success_buf = self.success_buf | success
        self.success = self.success_buf & time_out

        old_coef = 1.0 - time_out*0.1
        new_coef = time_out*0.1

        self.success_rate = self.success_rate*old_coef + success*new_coef

        return self.rew_buf, self.reset_buf
    
    def _get_base_observation(self, suggested_gt=None) :

        self.hand_tip_rot0 = self.hand_rigid_body_tensor0[:, 3:7]
        hand_down_dir0 = quat_axis(self.hand_tip_rot0, 2)
        self.hand_tip_pos0 = self.hand_rigid_body_tensor0[:, 0:3] + hand_down_dir0 * 0.130
        self.hand_tip_rot1 = self.hand_rigid_body_tensor1[:, 3:7]
        hand_down_dir1 = quat_axis(self.hand_tip_rot1, 2)
        self.hand_tip_pos1 = self.hand_rigid_body_tensor1[:, 0:3] + hand_down_dir1 * 0.130
        
        dim = 105       # 93 + 12
        joints = self.franka_num_dofs - 2
        if "useTaskId" in self.cfg["task"] and self.cfg["task"]["useTaskId"]:
            dim += self.chair_num
        state = torch.zeros((self.num_envs, dim), device=self.device)

        # joint dof value arm0
        state[:,:joints].copy_((2 * (self.franka_dof_tensor[:, 0, :joints, 0]-self.franka_dof_lower_limits_tensor[:joints])/(self.franka_dof_upper_limits_tensor[:joints] - self.franka_dof_lower_limits_tensor[:joints])) - 1)
        # joint dof velocity arm0
        state[:,joints:joints*2].copy_(self.franka_dof_tensor[:, 0, :joints, 1])
        # hand arm0
        state[:,joints*2:joints*2+13].copy_(relative_pose(self.franka_root_tensor[:, 0, :], self.hand_rigid_body_tensor0[:, :]).view(self.env_num, -1))
        # actions arm0
        state[:,joints*2+13:joints*3+13].copy_(self.actions[:, :joints])

        # joint dof value arm1
        state[:,joints*3+13:joints*4+13].copy_((2 * (self.franka_dof_tensor[:, 1, :joints, 0]-self.franka_dof_lower_limits_tensor[:joints])/(self.franka_dof_upper_limits_tensor[:joints] - self.franka_dof_lower_limits_tensor[:joints])) - 1)
        # joint dof velocity arm1
        state[:,joints*4+13:joints*5+13].copy_(self.franka_dof_tensor[:, 1, :joints, 1])
        # hand arm1
        state[:,joints*5+13:joints*5+26].copy_(relative_pose(self.franka_root_tensor[:, 1, :], self.hand_rigid_body_tensor1[:, :]).view(self.env_num, -1))
        # actions arm1
        state[:,joints*5+26:joints*6+26].copy_(self.actions[:, self.franka_num_dofs:self.franka_num_dofs+joints])

        # chair root
        state[:,joints*6+26:joints*6+39].copy_(self.chair_root_tensor)

        gt_begin = 93

        if suggested_gt != None:
            state[:,gt_begin:gt_begin+3].copy_(self.franka_root_tensor[:, 0, 0:3] - suggested_gt[:, 0])
            state[:,gt_begin+3:gt_begin+6].copy_(suggested_gt[:, 0] - self.hand_tip_pos0)
            state[:,gt_begin+6:gt_begin+9].copy_(self.franka_root_tensor[:, 1, 0:3] - suggested_gt[:, 1])
            state[:,gt_begin+9:gt_begin+12].copy_(suggested_gt[: ,1] - self.hand_tip_pos1)

        if "useTaskId" in self.cfg["task"] and self.cfg["task"]["useTaskId"]:
            state[:,49:49+self.chair_num] = torch.eye(self.chair_num, device=self.device).repeat_interleave(self.env_per_chair, dim=0)

        return state
    
    def _refresh_observation(self) :

        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        if self.cfg["env"]["driveMode"] == "ik" :
            self.gym.refresh_jacobian_tensors(self.sim)
        
        self.obs_buf = self._get_base_observation()

    def _perform_actions(self, actions):

        actions = actions.to(self.device)
        self.actions = actions
        joints = self.franka_num_dofs - 2
        dofs = self.franka_num_dofs
        self.pos_act[:, :joints] = self.pos_act[:, :joints] + actions[:, 0:joints] * self.dt * self.action_speed_scale
        self.pos_act[:, :joints] = tensor_clamp(
            self.pos_act[:, :joints], self.franka_dof_lower_limits_tensor[:joints], self.franka_dof_upper_limits_tensor[:joints]
        )
        self.pos_act[:, dofs:dofs+joints] = self.pos_act[:, dofs:dofs+joints] + actions[:, dofs:dofs+joints] * self.dt * self.action_speed_scale
        self.pos_act[:, dofs:dofs+joints] = tensor_clamp(
            self.pos_act[:, dofs:dofs+joints], self.franka_dof_lower_limits_tensor[:joints], self.franka_dof_upper_limits_tensor[:joints]
        )
        self.eff_act[:, joints:dofs] = actions[:, joints:dofs] * self.franka_dof_max_torque_tensor[joints:dofs]
        self.eff_act[:, dofs+joints:dofs*2] = actions[:, dofs+joints:dofs*2] * self.franka_dof_max_torque_tensor[joints:dofs]
        self.gym.set_dof_position_target_tensor(
            self.sim, gymtorch.unwrap_tensor(self.pos_act.view(-1))
        )
        self.gym.set_dof_actuation_force_tensor(
            self.sim, gymtorch.unwrap_tensor(self.eff_act.view(-1))
        )
    
    def _refresh_contact(self) :

        if not self.train_mode :
            return

        chair_pos = self.chair_root_tensor[:, 0:3]
        chair_rot = self.chair_root_tensor[:, 3:7]
        hand0_pos = self.hand_tip_pos0
        hand0_rot = self.hand_tip_rot0
        hand1_pos = self.hand_tip_pos1
        hand1_rot = self.hand_tip_rot1

        chair_min = chair_pos + quat_apply(chair_rot, self.chair_min_tensor.repeat_interleave(self.env_per_chair, dim=0)*1.1)
        chair_max = chair_pos + quat_apply(chair_rot, self.chair_max_tensor.repeat_interleave(self.env_per_chair, dim=0)*1.1)

        in_box_mask0 = self._detailed_view(
            (hand0_pos[:, 0] > chair_min[:, 0]) &
            (hand0_pos[:, 0] < chair_max[:, 0]) &
            (hand0_pos[:, 1] > chair_min[:, 1]) &
            (hand0_pos[:, 1] < chair_max[:, 1]) &
            (hand0_pos[:, 2] > chair_min[:, 2]) &
            (hand0_pos[:, 2] < chair_max[:, 2])
        )
        in_box_mask1 = self._detailed_view(
            (hand1_pos[:, 0] > chair_min[:, 0]) &
            (hand1_pos[:, 0] < chair_max[:, 0]) &
            (hand1_pos[:, 1] > chair_min[:, 1]) &
            (hand1_pos[:, 1] < chair_max[:, 1]) &
            (hand1_pos[:, 2] > chair_min[:, 2]) &
            (hand1_pos[:, 2] < chair_max[:, 2])
        )

        chair_spd = torch.norm(self.chair_root_tensor[:, 0:7], p=2, dim=1)
        moving_mask = self._detailed_view(chair_spd > self.contact_moving_threshold)

        mask0 = in_box_mask0 & moving_mask
        mask1 = in_box_mask1 & moving_mask

        relative_pos0 = quat_apply(quat_conjugate(chair_rot), hand0_pos - chair_pos)
        relative_rot0 = quat_mul(quat_conjugate(chair_rot), hand0_rot)
        relative_pos1 = quat_apply(quat_conjugate(chair_rot), hand1_pos - chair_pos)
        relative_rot1 = quat_mul(quat_conjugate(chair_rot), hand1_rot)

        pose0 = self._detailed_view(torch.cat((relative_pos0, relative_rot0, self.rew_buf.view(-1, 1)), dim=1))
        pose1 = self._detailed_view(torch.cat((relative_pos1, relative_rot1, self.rew_buf.view(-1, 1)), dim=1))

        for i, buffer in enumerate(self.contact_buffer_list0) :
            non_zero_idx = torch.nonzero(mask0[i], as_tuple=True)[0]
            buffer.insert(pose0[i, non_zero_idx])

        for i, buffer in enumerate(self.contact_buffer_list1) :
            non_zero_idx = torch.nonzero(mask1[i], as_tuple=True)[0]
            buffer.insert(pose1[i, non_zero_idx])

    def _draw_line(self, src, dst) :
        line_vec = np.stack([src.cpu().numpy(), dst.cpu().numpy()]).flatten().astype(np.float32)
        color = np.array([1,0,0], dtype=np.float32)
        self.gym.clear_lines(self.viewer)
        self.gym.add_lines(
            self.viewer,
            self.env_ptr_list[0],
            self.env_num,
            line_vec,
            color
        )

    # @TimeCounter
    def step(self, actions) :

        self._perform_actions(actions)
        
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        
        if not self.headless :
            self.render()
        if self.cfg["env"]["enableCameraSensors"] == True:
            self.gym.step_graphics(self.sim)

        self.progress_buf += 1

        self._refresh_observation()

        reward, done = self._get_reward_done()
        self._refresh_contact()

        done = self.reset_buf.clone()
        success = self.success.clone()
        self._partial_reset(self.reset_buf)

        if self.average_reward == None :
            self.average_reward = self.rew_buf.mean()
        else :
            self.average_reward = self.rew_buf.mean() * 0.01 + self.average_reward * 0.99
        self.extras["successes"] = success
        self.extras["success_rate"] = self.success_rate
        return self.obs_buf, self.rew_buf, done, None
    
    def _partial_reset(self, to_reset = "all") :

        """
        reset those need to be reseted
        """

        if to_reset == "all" :
            to_reset = np.ones((self.env_num,))
        reseted = False
        for env_id, reset in enumerate(to_reset) :
            # is reset:
            if reset.item() :
                # need randomization
                reset_dof_states = self.initial_dof_states[env_id].clone()
                reset_root_states = self.initial_root_states[env_id].clone()
                franka_reset_pos_tensor = reset_root_states[0, :3]
                franka_reset_rot_tensor = reset_root_states[0, 3:7]
                franka_reset_dof_pos_tensor = reset_dof_states[:self.franka_num_dofs, 0]
                franka_reset_dof_vel_tensor = reset_dof_states[:self.franka_num_dofs, 1]
                chair_reset_pos_tensor = reset_root_states[1, :3]
                chair_reset_rot_tensor = reset_root_states[1, 3:7]

                chair_type = env_id // self.env_per_chair
                
                self.intervaledRandom_(franka_reset_pos_tensor, self.franka_reset_position_noise)
                self.intervaledRandom_(franka_reset_rot_tensor, self.franka_reset_rotation_noise)
                self.intervaledRandom_(franka_reset_dof_pos_tensor, self.franka_reset_dof_pos_interval, self.franka_dof_lower_limits_tensor, self.franka_dof_upper_limits_tensor)
                self.intervaledRandom_(franka_reset_dof_vel_tensor, self.franka_reset_dof_vel_interval)
                self.intervaledRandom_(chair_reset_pos_tensor, self.chair_reset_position_noise)
                self.intervaledRandom_(chair_reset_rot_tensor, self.chair_reset_rotation_noise)

                self.dof_state_tensor[env_id].copy_(reset_dof_states)
                self.root_tensor[env_id].copy_(reset_root_states)
                reseted = True
                self.progress_buf[env_id] = 0
                self.reset_buf[env_id] = 0
                self.success_buf[env_id] = 0
        
        if reseted :
            self.gym.set_dof_state_tensor(
                self.sim,
                gymtorch.unwrap_tensor(self.dof_state_tensor)
            )
            self.gym.set_actor_root_state_tensor(
                self.sim,
                gymtorch.unwrap_tensor(self.root_tensor)
            )
            
    def reset(self, to_reset = "all") :

        self._partial_reset(to_reset)

        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        if not self.headless :
            self.render()
        if self.cfg["env"]["enableCameraSensors"] == True:
            self.gym.step_graphics(self.sim)
        
        self._refresh_observation()
        success = self.success.clone()
        reward, done = self._get_reward_done()

        self.extras["successes"] = success
        self.extras["success_rate"] = self.success_rate
        return self.obs_buf, self.rew_buf, self.reset_buf, None

    def save(self, path, iteration) :

        buffer_tensor_list0 = []

        for buffer in self.contact_buffer_list0 :
            buffer_tensor_list0.append(buffer.buffer)
        
        buffer_tensor0 = torch.stack(buffer_tensor_list0)
        torch.save(buffer_tensor0, os.path.join(path, "buffer0_{}.pt".format(iteration)))

        buffer_tensor_list1 = []

        for buffer in self.contact_buffer_list1 :
            buffer_tensor_list1.append(buffer.buffer)
        
        buffer_tensor1 = torch.stack(buffer_tensor_list1)
        torch.save(buffer_tensor1, os.path.join(path, "buffer1_{}.pt".format(iteration)))

        save_dict = self.cfg
        chair_success_rate = self._detailed_view(self.success_rate).mean(dim=1)
        chair_train_success_rate = chair_success_rate[:self.chair_num_train]
        chair_val_success_rate = chair_success_rate[self.chair_num_train:]
        for id, (name, tensor) in enumerate(zip(self.chair_train_name_list, chair_train_success_rate)) :
            save_dict["env"]["asset"]["trainAssets"][name]["successRate"] = tensor.cpu().item()
            save_dict["env"]["asset"]["trainAssets"][name]["envIds"] = id * self.env_per_chair
        for id, (name, tensor) in enumerate(zip(self.chair_val_name_list, chair_val_success_rate)) :
            save_dict["env"]["asset"]["testAssets"][name]["successRate"] = tensor.cpu().item()
            save_dict["env"]["asset"]["testAssets"][name]["envIds"] = id * self.env_per_chair
        with open(os.path.join(path, "cfg_{}.yaml".format(iteration)), "w") as f:
            yaml.dump(save_dict, f)
    
    def load(self, path, iteration) :

        buffer_path = os.path.join(path, "buffer_{}.pt".format(iteration))
        if os.path.exists(buffer_path) :
            with open(buffer_path, "rb") as f:
                buffer_tensor = torch.load(f, map_location=self.device)
                for buffer, tmp in zip(self.contact_buffer_list, buffer_tensor) :
                    buffer.buffer = tmp
                    buffer.top = tmp.shape[0]
                    while buffer.top and buffer.buffer[buffer.top-1].sum()==0 :
                        buffer.top -= 1
    
    def update(self, it=0) :

        pass
    
    def train(self) :        # changing mode to eval

        self.train_mode = True

    def eval(self) :        # changing mode to eval

        self.train_mode = False
    
    def _detailed_view(self, tensor) :

        shape = tensor.shape
        return tensor.view(self.chair_num, self.env_per_chair, *shape[1:])
    
    def intervaledRandom_(self, tensor, dist, lower=None, upper=None) :
        tensor += torch.rand(tensor.shape, device=self.device)*dist*2 - dist
        if lower is not None and upper is not None :
            torch.clamp_(tensor, min=lower, max=upper)

def orientation_error(desired, current):
    cc = quat_conjugate(current)
    q_r = quat_mul(desired, cc)
    return q_r[:, 0:3] * torch.sign(q_r[:, 3]).unsqueeze(-1)

def control_ik(j_eef, device, dpose, num_envs):

    # Set controller parameters
    # IK params
    damping = 0.05
    # solve damped least squares
    j_eef_T = torch.transpose(j_eef, 1, 2)
    lmbda = torch.eye(6, device=device) * (damping ** 2)
    u = (j_eef_T @ torch.inverse(j_eef @ j_eef_T + lmbda) @ dpose).view(num_envs, -1)
    return u

def relative_pose(src, dst) :

    shape = dst.shape
    p = dst.view(-1, shape[-1])[:, :3] - src.view(-1, src.shape[-1])[:, :3]
    ip = dst.view(-1, shape[-1])[:, 3:]
    ret = torch.cat((p, ip), dim=1)
    return ret.view(*shape)