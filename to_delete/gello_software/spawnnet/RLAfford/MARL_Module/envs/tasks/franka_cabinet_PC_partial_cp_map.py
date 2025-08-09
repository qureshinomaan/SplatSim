import os
from isaacgym.torch_utils import *
from zmq import PROTOCOL_ERROR_ZAP_INVALID_STATUS_CODE
from tasks.franka_cabinet_PC_partial import OneFrankaCabinetPCPartial
from Collision_Predictor_Module.CollisionPredictor.code.train_with_RL import CollisionPredictor
from MARL_Module.envs.utils.gpu_mem_track import MemTracker


class OneFrankaCabinetPCPartialCPMap(OneFrankaCabinetPCPartial) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.CollisionPredictor = CollisionPredictor(cfg, log_dir)
        self.cabinet_mask_dim = 5
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index, is_multi_agent, log_dir=log_dir)

        self.num_obs += 1
        self.task_meta["mask_dim"] = 3
        self.task_meta["obs_dim"] = self.num_obs
        self.task_meta['need_update'] = True
        self.num_feature = cfg["env"]["pointFeatureDim"]
        self.CP_iter = cfg['cp']['CP_iter']
        self.obs_buf = torch.zeros((self.num_envs, self.pointCloudDownsampleNum, self.num_obs), device=self.device, dtype=torch.float)

        self.depth_bar = self.cfg["env"]["depth_bar"]
        self.success_rate_bar = self.cfg["cp"]["success_rate_bar"]
        self.raw_map = torch.zeros((self.cabinet_num, self.cabinetPCOriginalNum), device=self.device)
        self.map = torch.zeros((self.cabinet_num, self.cabinetPCOriginalNum), device=self.device)
        # The tensors representing pointcloud and map are:
        # self.cabinet_pc   (obj*8192*5)
        # self.raw_map      (obj*8192)
        # self.map          (obj*8192)

    def quat_apply(self, a, b):
        shape = b.shape
        a = a.reshape(-1, 4)  # 4
        a_expand = a.expand(shape[0], 4)
        b = b.reshape(-1, 3)  # num_buffer*3
        xyz = a_expand[:, :3]   # 3
        t = xyz.cross(b, dim=-1) * 2
        return (b + a_expand[:, 3:] * t + xyz.cross(t, dim=-1)).view(shape)
    
    def _refresh_map(self) :
        
        # predict all points at the begin
        with torch.no_grad() :
            sampled_pc = self.sample_points(
                self.selected_cabinet_pc,
                self.cabinetPCDownsampleNum,
                sample_method="furthest",
            )
            transformed_pc = self._get_transformed_pc(
                pc=sampled_pc[:, :, :3],
                mask=sampled_pc[:, :, 3]
            )
            stacked_pc = torch.cat(
                (
                    transformed_pc,
                    sampled_pc[:, :, 3].view(self.cabinet_num, -1, 1)
                ),
                dim=2
            )
            self.map = self.CollisionPredictor.pred_one_batch(
                stacked_pc,
                self.success_rate,
                num_train=self.env_num_train
            ).to(self.device)
            self.map *= sampled_pc[:, :, 3]
            self.sampled_cabinet_pc[:, :, :4] = sampled_pc.repeat_interleave(self.env_per_cabinet, dim=0)
            self.sampled_cabinet_pc[:, :, 4] = self.map.repeat_interleave(self.env_per_cabinet, dim=0)

    def save(self, path, iteration) :

        super().save(path, iteration)
        self.CollisionPredictor.save_checkpoint(os.path.join(path, "CP_{}.pt".format(iteration)))

        save_pc = self.compute_point_cloud_state(pc=self.sampled_cabinet_pc)
        torch.save(self._detailed_view(save_pc)[:, 0, ...], os.path.join(path, "map_{}.pt".format(iteration)))

        transformed_pc = self._get_transformed_pc()
        saving_raw_map = torch.cat((transformed_pc[:, :, :3], self.raw_map[:, :].view(self.cabinet_num, -1, 1)), dim=-1)
        torch.save(saving_raw_map, os.path.join(path, "rawmap_{}.pt".format(iteration)))
    
    def load(self, path, iteration) :
        
        cp_file = os.path.join(path, "CP_{}.pt".format(iteration))
        print("loading CP checkpoint", cp_file)
        super().load(path, iteration)
        self.CollisionPredictor.load_checkpoint(cp_file)
        self._refresh_map()

    def _data_argumentation(self, pcd):
        pcd[:, :, :3] *= torch.rand((pcd.shape[0], 1, 3), device=self.device)*0.3 + 0.85
        return pcd
    
    def _get_transformed_pc(self, pc=None, mask=None) :

        if pc is None:
            pc = self.selected_cabinet_pc[:, :, :3]
        if mask is None:
            mask = self.selected_cabinet_pc[:, :, 3]

        # select first env of each type of cabinet
        used_initial_root_state = self._detailed_view(self.initial_root_states)[:, 0, ...]
        used_initial_rigd_state = self._detailed_view(self.initial_rigid_body_states)[:, 0, ...]

        transformed_pc = self._transform_pc(
            pc,
            mask.view(self.cabinet_num, -1, 1),
            used_initial_root_state[:, 1, :7],
            used_initial_rigd_state[:, self.cabinet_rigid_body_index, :7]
        )

        return transformed_pc

    def update(self, iter) :
    
        CP_info = {}
        used_success_rate = self._detailed_view(self.success_rate).mean(dim=-1)
        
        if used_success_rate.mean() > self.success_rate_bar :
            # do training only when success rate is enough

            transformed_pc = self._get_transformed_pc()
            self.raw_map = self.get_map(self.selected_cabinet_pc[:, :, :3], self.selected_cabinet_pc[:, :, 3], self.contact_buffer_list)

            # stack them together to make resample easier
            stacked_pc_target = torch.cat(
                (
                    transformed_pc,
                    self.selected_cabinet_pc[:, :, 3].view(self.cabinet_num, -1, 1),
                    self.raw_map.view(self.cabinet_num, -1, 1)
                ),
                dim=2
            )

            # in training, sample a few points to train CP each epoch
            minibatch_size = self.cfg["cp"]["cp_minibatch_size"]
            for i in range(self.CP_iter):
                info_list = []
                sampled_pc_target = self.sample_points(
                    stacked_pc_target,
                    self.cabinetPCDownsampleNum,
                    sample_method="furthest"
                )
                sampled_pc_target = self._data_argumentation(sampled_pc_target)
                for cur_pc_target, cur_success_rate in zip(
                        torch.split(sampled_pc_target, minibatch_size),
                        torch.split(used_success_rate, minibatch_size)
                    ) :
                    # self._refresh_pointcloud_visualizer(cur_pc_target[0, :, :3], cur_pc_target[0, :, 3])
                    cur_map, cur_info = self.CollisionPredictor.pred_one_batch(
                        cur_pc_target[:, :, :4],
                        cur_success_rate,
                        target=cur_pc_target[:, :, 4],
                        num_train=self.cabinet_num_train
                    )
                    info_list.append(cur_info)
            self.CollisionPredictor.network_lr_scheduler.step()

            # collecting training info
            if self.CP_iter :
                for key in info_list[0] :
                    tmp = 0
                    for info in info_list :
                        tmp += info[key]
                    CP_info[key] = tmp/len(info_list)
        
            self._refresh_map()

        return CP_info
    
    def _get_max_point(self, pc, map) :

        env_max = map.max(dim=-1)[0]
        weight = torch.where(map > env_max.view(self.env_num, 1)-0.1, 1, 0)
        weight_reshaped = weight.view(self.env_num, -1, 1)
        mean = (pc*weight_reshaped).mean(dim=1)
        return mean

    def _get_reward_done(self) :

        rew, res = super()._get_reward_done()

        d = torch.norm(self.hand_tip_pos - self._get_max_point(self.sampled_cabinet_pc[:, :, :3], self.sampled_cabinet_pc[:, :, 4]), p=2, dim=-1)
        dist_reward = 1.0 / (1.0 + d**2)
        dist_reward *= dist_reward
        dist_reward = torch.where(d <= 0.1, dist_reward*2, dist_reward)
        rew += dist_reward * self.cfg['cp']['max_point_reward']

        return rew, res

    def _refresh_observation(self) :
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        if self.cfg["env"]["driveMode"] == "ik" :
            self.gym.refresh_jacobian_tensors(self.sim)

        point_clouds = self.compute_point_cloud_state(pc=self.sampled_cabinet_pc)

        point_clouds[:, :, :3] -= self.franka_root_tensor[:, :3].view(-1, 1, 3)
        self.obs_buf[:, :, :6].copy_(point_clouds)
        if self.cfg["cp"]["max_point_observation"] :
            self.obs_buf[:, :, 6:] = self._get_base_observation(self._get_max_point(point_clouds[:, :, :3], point_clouds[:, :, 4])).view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
        else :
            self.obs_buf[:, :, 6:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(point_clouds[0, :, :3], point_clouds[0, :, 4])