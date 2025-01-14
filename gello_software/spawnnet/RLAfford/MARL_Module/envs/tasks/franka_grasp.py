import os
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

class OneFrankaGrasp(BaseTask) :

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
        self.action_speed_scale = self.cfg["env"]["actionSpeedScale"]
        
        self.env_num_train = cfg["env"]["numTrain"]
        self.env_num_val = cfg["env"]["numVal"]
        self.env_num = self.env_num_train + self.env_num_val
        self.asset_root = cfg["env"]["asset"]["assetRoot"]
        self.object_num_train = cfg["env"]["asset"]["assetNumTrain"]
        self.object_num_val = cfg["env"]["asset"]["assetNumVal"]
        self.object_num = self.object_num_train+self.object_num_val
        object_train_list_len = len(cfg["env"]["asset"]["trainAssets"])
        object_val_list_len = len(cfg["env"]["asset"]["testAssets"])
        self.object_train_name_list = []
        self.object_val_name_list = []
        self.exp_name = cfg['env']["env_name"]
        print("Simulator: number of objects", self.object_num)
        print("Simulator: number of environments", self.env_num)
        if self.object_num_train :
            assert(self.env_num_train % self.object_num_train == 0)
        if self.object_num_val :
            assert(self.env_num_val % self.object_num_val == 0)
        assert(self.env_num_train*self.object_num_val == self.env_num_val*self.object_num_train)
        assert(self.object_num_train <= object_train_list_len)    # the number of used length must less than real length
        assert(self.object_num_val <= object_val_list_len)    # the number of used length must less than real length
        assert(self.env_num % self.object_num == 0)    # each object should have equal number envs
        self.env_per_object = self.env_num // self.object_num
        self.task_meta = {
            "training_env_num": self.env_num_train,
            "valitating_env_num": self.env_num_val,
            "need_update": True,
            "max_episode_length": self.max_episode_length,
            "obs_dim": cfg["env"]["numObservations"]
        }
        for name in cfg["env"]["asset"]["trainAssets"] :
            self.object_train_name_list.append(cfg["env"]["asset"]["trainAssets"][name]["name"])
        for name in cfg["env"]["asset"]["testAssets"] :
            self.object_val_name_list.append(cfg["env"]["asset"]["testAssets"][name]["name"])

        self.env_ptr_list = []
        self.obj_loaded = False
        self.franka_loaded = False

        super().__init__(cfg=self.cfg, enable_camera_sensors=cfg["env"]["enableCameraSensors"])

        # acquire tensors
        self.root_tensor = gymtorch.wrap_tensor(self.gym.acquire_actor_root_state_tensor(self.sim))
        self.dof_state_tensor = gymtorch.wrap_tensor(self.gym.acquire_dof_state_tensor(self.sim))
        self.rigid_body_tensor = gymtorch.wrap_tensor(self.gym.acquire_rigid_body_state_tensor(self.sim))
        if self.cfg["env"]["driveMode"] == "ik" :    # inverse kinetic needs jacobian tensor, other drive mode don't need
            self.jacobian_tensor = gymtorch.wrap_tensor(self.gym.acquire_jacobian_tensor(self.sim, "franka"))

        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)

        self.root_tensor = self.root_tensor.view(self.num_envs, -1, 13)
        self.dof_state_tensor = self.dof_state_tensor.view(self.num_envs, -1, 2)
        self.rigid_body_tensor = self.rigid_body_tensor.view(self.num_envs, -1, 13)
        self.franka_dof_tensor = self.dof_state_tensor
        self.actions = torch.zeros((self.num_envs, self.num_actions), device=self.device)

        self.initial_dof_states = self.dof_state_tensor.clone()
        self.initial_root_states = self.root_tensor.clone()

        # precise slices of tensors
        env_ptr = self.env_ptr_list[0]
        franka1_actor = self.franka_actor_list[0]
        object_actor = self.object_actor_list[0]
        self.hand_rigid_body_index = self.gym.find_actor_rigid_body_index(
            env_ptr,
            franka1_actor,
            "panda_hand",
            gymapi.DOMAIN_ENV
        )
        self.hand_lfinger_rigid_body_index = self.gym.find_actor_rigid_body_index(
            env_ptr,
            franka1_actor,
            "panda_leftfinger",
            gymapi.DOMAIN_ENV
        )
        self.hand_rfinger_rigid_body_index = self.gym.find_actor_rigid_body_index(
            env_ptr,
            franka1_actor,
            "panda_rightfinger",
            gymapi.DOMAIN_ENV
        )
        self.hand_rigid_body_tensor = self.rigid_body_tensor[:, self.hand_rigid_body_index, :]
        self.franka_root_tensor = self.root_tensor[:, 0, :]
        self.object_root_tensor = self.root_tensor[:, 1, :]

        self.dof_dim = self.franka_num_dofs
        self.pos_act = self.initial_dof_states[:, :, 0].clone()
        self.eff_act = torch.zeros((self.num_envs, self.dof_dim), device=self.device)

        # init collision buffer
        self.contact_buffer_size = cfg["env"]["contactBufferSize"]
        self.contact_moving_threshold = cfg["env"]["contactMovingThreshold"]
        self.near_dis_bar = cfg["env"]["near_dis_bar"]
        self.contact_buffer_list = []
        
        assert(self.contact_buffer_size >= self.env_per_object)
        for i in range(self.object_num) :
            self.contact_buffer_list.append(ContactBuffer(self.contact_buffer_size, 8, device=self.device))
        
        # params of randomization
        self.object_reset_position_noise = cfg["env"]["reset"]["object"]["resetPositionNoise"]
        self.object_reset_rotation_noise = cfg["env"]["reset"]["object"]["resetRotationNoise"]
        self.franka_reset_position_noise = cfg["env"]["reset"]["franka"]["resetPositionNoise"]
        self.franka_reset_rotation_noise = cfg["env"]["reset"]["franka"]["resetRotationNoise"]
        self.franka_reset_dof_pos_interval = cfg["env"]["reset"]["franka"]["resetDofPosRandomInterval"]
        self.franka_reset_dof_vel_interval = cfg["env"]["reset"]["franka"]["resetDofVelRandomInterval"]

        # params for success rate
        self.success = torch.zeros((self.env_num,), device=self.device)
        self.success_rate = torch.zeros((self.env_num,), device=self.device)
        self.success_queue = torch.zeros((self.env_num, 32), device=self.device)
        self.success_idx = torch.zeros((self.num_envs,), device=self.device).long()
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

    def _load_franka(self, env_ptr, env_id):

        if self.franka_loaded == False :

            self.franka_actor_list = []

            asset_root = self.asset_root
            asset_file = "franka_description/robots/franka_panda_longer.urdf"
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
        initial_franka_pose = gymapi.Transform()
        initial_franka_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
        initial_franka_pose.p = gymapi.Vec3(0.56, 0.0, 0.01)

        # set start dof
        self.franka_num_dofs = self.gym.get_asset_dof_count(self.franka_asset)
        default_dof_pos = np.zeros(self.franka_num_dofs, dtype=np.float32)
        default_dof_pos[:-2] = (franka_dof_lower_limits + franka_dof_upper_limits)[:-2] * 0.3
        # grippers open
        default_dof_pos[-2:] = franka_dof_upper_limits[-2:]
        franka_dof_state = np.zeros_like(franka_dof_max_torque, gymapi.DofState.dtype)
        franka_dof_state["pos"] = default_dof_pos

        franka_actor = self.gym.create_actor(
            env_ptr,
            self.franka_asset, 
            initial_franka_pose,
            "franka",
            env_id,
            2,
            0)
        
        # rigid props
        # franka_shape_props = self.gym.get_actor_rigid_shape_properties(env_ptr, franka_actor)
        # for shape in franka_shape_props :
        #     shape.friction = 100
        
        # self.gym.set_actor_rigid_shape_properties(env_ptr, franka_actor, franka_shape_props)
        self.gym.set_actor_dof_properties(env_ptr, franka_actor, dof_props)
        self.gym.set_actor_dof_states(env_ptr, franka_actor, franka_dof_state, gymapi.STATE_ALL)
        self.franka_actor_list.append(franka_actor)

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

        self.object_asset_name_list = []
        self.object_asset_list = []
        self.object_pose_list = []
        self.object_actor_list = []
        self.object_pc = []
        self.object_scale = []

        train_len = len(self.cfg["env"]["asset"]["trainAssets"].items())
        val_len = len(self.cfg["env"]["asset"]["testAssets"].items())
        train_len = min(train_len, self.object_num_train)
        val_len = min(val_len, self.object_num_val)
        total_len = train_len + val_len
        used_len = min(total_len, self.object_num)

        random_asset = self.cfg["env"]["asset"]["randomAsset"]
        select_train_asset = [i for i in range(train_len)]
        select_val_asset = [i for i in range(val_len)]
        if random_asset :        # if we need random asset list from given dataset, we shuffle the list to be read
            shuffle(select_train_asset)
            shuffle(select_val_asset)
        select_train_asset = select_train_asset[:train_len]
        select_val_asset = select_val_asset[:val_len]

        with tqdm(total=used_len) as pbar:
            pbar.set_description('Loading object assets:')
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
            
                self.object_asset_name_list.append(name)

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

                object_asset = self.gym.load_asset(self.sim, self.asset_root, val["path"], asset_options)
                self.object_asset_list.append(object_asset)

                obj_start_pose = gymapi.Transform()
                obj_start_pose.p = gymapi.Vec3(0.0, 0.0, 0.3)
                obj_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
                self.object_pose_list.append(obj_start_pose)

                self.object_pc.append(torch.load(os.path.join(self.asset_root, val["pc"]), map_location=self.device))
                # self.object_scale.append(val["scale"])
                
                pbar.update(1)
                cur += 1
            
        self.object_pc = torch.stack(self.object_pc).float()

    def _load_obj(self, env_ptr, env_id):

        if self.obj_loaded == False :

            self._load_obj_asset()

            self.obj_loaded = True

        object_type = env_id // self.env_per_object
        subenv_id = env_id % self.env_per_object
        obj_actor = self.gym.create_actor(
            env_ptr,
            self.object_asset_list[object_type],
            self.object_pose_list[object_type],
            "object{}-{}".format(object_type, subenv_id),
            env_id,
            1,
            0
        )
        # self.gym.set_actor_scale(
        #     env_ptr,
        #     obj_actor,
        #     self.object_scale[object_type]
        # )
        self.object_actor_list.append(obj_actor)

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

        success = torch.zeros((self.env_num,), device=self.device).long()
        self.rew_buf = torch.zeros((self.env_num,), device=self.device)

        obj_pos = self.object_root_tensor[:, :3]

        d = torch.norm(self.hand_tip_pos - obj_pos, p=2, dim=-1)
        r = quat_axis(self.hand_rot, 2)

        # dist_reward = -d
        dist_reward = 1.0 / (1.0 + d**2)
        dist_reward *= dist_reward
        dist_reward = torch.where(d <= 0.2, torch.ones_like(d)*1.1, dist_reward)

        drct_reward = -r[:,2]

        open_reward = (d>0.2) * (d-0.2) * (self.actions[:, -2] + self.actions[:, -1])

        lift_reward = obj_pos[:, 2] * (d < 0.2)

        # print(dist_reward, lift_reward, drct_reward, open_reward)

        self.rew_buf = dist_reward + lift_reward*6 + (drct_reward-1)*0.3

        success = lift_reward > 0.2

        time_out = (self.progress_buf >= self.max_episode_length)
        self.reset_buf = (self.reset_buf | time_out)
        self.success_buf = self.success_buf | success
        self.success = self.success_buf & time_out
        # ipdb.set_trace()
        # self.success_queue.view(-1)[self.success_idx + torch.arange(0, self.env_num).to(self.success_queue.device)*self.success_queue.shape[1]] *= 1 - time_out.long()
        # self.success_queue.view(-1)[self.success_idx + torch.arange(0, self.env_num).to(self.success_queue.device)*self.success_queue.shape[1]] += self.success.long()

        old_coef = 1.0 - time_out*0.1
        new_coef = time_out*0.1

        self.success_rate = self.success_rate*old_coef + success*new_coef
        self.total_success_rate = self.success_rate.sum(dim=-1)
        self.success_entropy = - self.success_rate/(self.total_success_rate+1e-8) * torch.log(self.success_rate/(self.total_success_rate+1e-8) + 1e-8) * self.env_num

        return self.rew_buf, self.reset_buf
    
    def _get_base_observation(self, suggested_gt=None) :

        hand_rot = self.hand_rigid_body_tensor[..., 3:7]
        hand_down_dir = quat_axis(hand_rot, 2)
        self.hand_tip_pos = self.hand_rigid_body_tensor[..., 0:3] + hand_down_dir * 0.130    # calculating middle of two fingers
        self.hand_rot = hand_rot

        dim = 39
        if "useTaskId" in self.cfg["task"] and self.cfg["task"]["useTaskId"]==True:
            dim += self.object_num

        state = torch.zeros((self.num_envs, dim), device=self.device)

        state[:,:7].copy_((2 * (self.franka_dof_tensor[:, :7, 0]-self.franka_dof_lower_limits_tensor[:7])/(self.franka_dof_upper_limits_tensor[:7] - self.franka_dof_lower_limits_tensor[:7])) - 1)
        state[:,7:14].copy_(self.franka_dof_tensor[:, :7, 1])

        state[:,14:27].copy_(relative_pose(self.franka_root_tensor, self.hand_rigid_body_tensor).view(self.env_num, -1))
        state[:,27:30].copy_(self.franka_root_tensor[:, 0:3])

        state[:,33:36].copy_(self.franka_root_tensor[:, 0:3] - self.object_root_tensor[:, 0:3])
        state[:,36:39].copy_(self.object_root_tensor[:, 0:3] - self.hand_tip_pos)

        if "useTaskId" in self.cfg["task"] :
            state[:,39:39+self.object_num] = torch.eye(self.object_num, device=self.device).repeat_interleave(self.env_per_object, dim=0)

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

        if self.cfg["env"]["driveMode"] == "pos":
            # print(self.franka_dof_lower_limits_tensor[3], self.franka_dof_upper_limits_tensor[3])
            # actions[:, 2] = 0
            # actions[:, 5] = 0
            self.pos_act[:, :-2] = self.pos_act[:, :-2] + actions[:, 0:-2] * self.dt * self.action_speed_scale
            self.pos_act[:, :-2] = tensor_clamp(
                self.pos_act[:, :-2], self.franka_dof_lower_limits_tensor[:-2], self.franka_dof_upper_limits_tensor[:-2])
        elif self.cfg["env"]["driveMode"] == "effort":
            self.eff_act[:, :-2] = actions[:, :-2] * self.franka_dof_max_torque_tensor[:-2]
        else :
            dof_pos = self.franka_dof_tensor[:, :, 0]
            target_pos = actions[:, :3]*self.space_range + self.space_middle
            pos_err = target_pos - self.hand_rigid_body_tensor[:, :3]
            target_rot = actions[:, 3:7] / torch.sqrt((actions[:, 3:7]**2).sum(dim=-1)+1e-8).view(-1, 1)
            rot_err = orientation_error(target_rot, self.hand_rigid_body_tensor[:, 3:7])
            
            self._draw_line(target_pos[0], target_pos[0] + quat_axis(target_rot, 0)[0])
            dpose = torch.cat([pos_err, rot_err], -1).unsqueeze(-1)
            delta = control_ik(self.jacobian_tensor[:, self.hand_rigid_body_index - 1, :, :-2], self.device, dpose, self.num_envs)
            self.pos_act[:, :-3] = dof_pos.squeeze(-1)[:, :-2] + delta

        self.eff_act[:, -2] = actions[:, -2] * self.franka_dof_max_torque_tensor[-2]
        self.eff_act[:, -1] = actions[:, -1] * self.franka_dof_max_torque_tensor[-1]
        self.gym.set_dof_position_target_tensor(
            self.sim, gymtorch.unwrap_tensor(self.pos_act.view(-1))
        )
        self.gym.set_dof_actuation_force_tensor(
            self.sim, gymtorch.unwrap_tensor(self.eff_act.view(-1))
        )
    
    def _refresh_contact(self) :

        if not self.train_mode :
            return
        
        object_spd = torch.norm(self.object_root_tensor[:, :3], p=2, dim=-1)
        moving_mask = (object_spd > self.contact_moving_threshold).view(self.object_num, self.env_per_object)

        object_pos = self.object_root_tensor[:, :3]
        object_rot = self.object_root_tensor[:, 3:7]

        relative_pos = quat_apply(quat_conjugate(object_rot), self.hand_tip_pos - object_pos)
        relative_rot = quat_mul(quat_conjugate(object_rot), self.hand_rot)                # is there any bug?

        near_mask = torch.norm(relative_pos)<self.near_dis_bar

        mask = moving_mask & near_mask

        # print(moving_mask.sum()-mask.sum())
        pose = torch.cat((relative_pos, relative_rot, self.rew_buf.view(-1, 1)), dim=1).view(self.object_num, self.env_per_object, -1)
        
        for i, buffer in enumerate(self.contact_buffer_list) :
            non_zero_idx = torch.nonzero(mask[i], as_tuple=True)[0]
            buffer.insert(pose[i, non_zero_idx])
        
    def _draw_line(self, src, dst) :
        line_vec = np.stack([src, dst]).flatten().astype(np.float32)
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
        self.extras["success_entropy"] = self.success_entropy
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
                object_reset_pos_tensor = reset_root_states[1, :3]
                object_reset_rot_tensor = reset_root_states[1, 3:7]
                franka_reset_dof_pos_tensor = reset_dof_states[:self.franka_num_dofs, 0]
                franka_reset_dof_vel_tensor = reset_dof_states[:self.franka_num_dofs, 1]

                object_type = env_id // self.env_per_object
                
                self.intervaledRandom_(object_reset_pos_tensor, self.object_reset_position_noise)
                self.intervaledRandom_(object_reset_rot_tensor, self.object_reset_rotation_noise)
                self.intervaledRandom_(franka_reset_pos_tensor, self.franka_reset_position_noise)
                self.intervaledRandom_(franka_reset_rot_tensor, self.franka_reset_rotation_noise)
                self.intervaledRandom_(franka_reset_dof_pos_tensor, self.franka_reset_dof_pos_interval, self.franka_dof_lower_limits_tensor, self.franka_dof_upper_limits_tensor)
                self.intervaledRandom_(franka_reset_dof_vel_tensor, self.franka_reset_dof_vel_interval)

                self.dof_state_tensor[env_id].copy_(reset_dof_states)
                self.root_tensor[env_id].copy_(reset_root_states)
                reseted = True
                self.progress_buf[env_id] = 0
                self.reset_buf[env_id] = 0
                self.success_buf[env_id] = 0
                self.success_idx[env_id] = (self.success_idx[env_id] + 1) % self.success_queue.shape[1]
        
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
        self.extras["success_entropy"] = self.success_entropy
        return self.obs_buf, self.rew_buf, self.reset_buf, None

    def save(self, path, iteration) :

        buffer_tensor_list = []

        for buffer in self.contact_buffer_list :
            buffer_tensor_list.append(buffer.buffer)
        
        buffer_tensor = torch.stack(buffer_tensor_list)
        torch.save(buffer_tensor, os.path.join(path, "buffer_{}.pt".format(iteration)))

        save_dict = self.cfg
        success_rate = self.success_rate.view(self.object_num, self.env_per_object).mean(dim=1)
        train_success_rate = success_rate[:self.object_num_train]
        val_success_rate = success_rate[self.object_num_train:]
        for id, (name, tensor) in enumerate(zip(self.object_train_name_list, train_success_rate)) :
            save_dict["env"]["asset"]["trainAssets"][name]["successRate"] = tensor.cpu().item()
            save_dict["env"]["asset"]["trainAssets"][name]["envIds"] = id * self.env_per_object
        for id, (name, tensor) in enumerate(zip(self.object_val_name_list, val_success_rate)) :
            save_dict["env"]["asset"]["testAssets"][name]["successRate"] = tensor.cpu().item()
            save_dict["env"]["asset"]["testAssets"][name]["envIds"] = id * self.env_per_object
        with open(os.path.join(path, "cfg_{}.yaml".format(iteration)), "w") as f:
            yaml.dump(save_dict, f)
    
    def load(self, path, iteration) :

        with open(os.path.join(path, "buffer_{}.pt".format(iteration)), "rb") as f:
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