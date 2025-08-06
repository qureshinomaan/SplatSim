import os
from isaacgym.torch_utils import *
from tasks.franka_cabinet_PC_partial_cp_map import OneFrankaCabinetPCPartialCPMap
from Collision_Predictor_Module.CollisionPredictor.code.train_with_RL import CollisionPredictor
from MARL_Module.envs.utils.gpu_mem_track import MemTracker


class OneFrankaCabinetPCPartialCPState(OneFrankaCabinetPCPartialCPMap) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.pointCloudDownsampleNum = cfg["env"]["pointDownsampleNum"]
        self.gpu_tracker = MemTracker()
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index, is_multi_agent, log_dir=log_dir)

        self.task_meta["mask_dim"] = 3
        self.task_meta['need_update'] = True
        self.num_feature = cfg["env"]["pointFeatureDim"]
        self.CP_iter = cfg['cp']['CP_iter']
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=self.device, dtype=torch.float)

        self.CollisionPredictor = CollisionPredictor(self.cfg, self.log_dir)
        self.depth_bar = self.cfg["env"]["depth_bar"]
        self.raw_map = torch.zeros((self.env_num, self.pointCloudDownsampleNum, 4), device=self.device)

    def _refresh_observation(self) :
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
            self.map = self.CollisionPredictor.pred_one_batch(point_clouds, self.success_rate, num_train=self.env_num_train)
            self.map = self.map.to(self.device)

        self.obs_buf = self._get_base_observation(self._get_max_point(self.map))