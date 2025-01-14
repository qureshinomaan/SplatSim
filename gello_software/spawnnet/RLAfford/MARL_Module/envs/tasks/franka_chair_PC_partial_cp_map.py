import os
from isaacgym.torch_utils import *
from zmq import PROTOCOL_ERROR_ZAP_INVALID_STATUS_CODE
from tasks.franka_chair_PC_partial import TwoFrankaChairPCPartial
from Collision_Predictor_Module.CollisionPredictor.code.train_with_RL import CollisionPredictor
from MARL_Module.envs.utils.gpu_mem_track import MemTracker


class TwoFrankaChairPCPartialCPMap(TwoFrankaChairPCPartial) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.gpu_tracker = MemTracker()
        self.CollisionPredictor0 = CollisionPredictor(cfg, log_dir)
        self.CollisionPredictor1 = CollisionPredictor(cfg, log_dir)
        self.chair_mask_dim = 5
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index, is_multi_agent, log_dir=log_dir)

        self.num_obs += 2
        self.task_meta["mask_dim"] = 4
        self.task_meta["obs_dim"] = self.num_obs
        self.task_meta['need_update'] = True
        self.num_feature = cfg["env"]["pointFeatureDim"]
        self.CP_iter = cfg['cp']['CP_iter']
        self.obs_buf = torch.zeros((self.num_envs, self.pointCloudDownsampleNum, self.num_obs), device=self.device, dtype=torch.float)

        self.depth_bar = self.cfg["env"]["depth_bar"]
        self.success_rate_bar = self.cfg["cp"]["success_rate_bar"]
        self.raw_map0 = torch.zeros((self.chair_num, self.chairPCOriginalNum), device=self.device)
        self.raw_map1 = torch.zeros((self.chair_num, self.chairPCOriginalNum), device=self.device)
        self.map0 = torch.zeros((self.chair_num, self.chairPCOriginalNum), device=self.device)
        self.map1 = torch.zeros((self.chair_num, self.chairPCOriginalNum), device=self.device)
        # The tensors representing pointcloud and map are:
        # self.chair_pc   (obj*8192*5)
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
                self.selected_chair_pc,
                self.chairPCDownsampleNum,
                sample_method="furthest",
            )
            transformed_pc = self._get_transformed_pc(
                pc=sampled_pc[:, :, :3]
            )
            stacked_pc = transformed_pc
            self.map0 = self.CollisionPredictor0.pred_one_batch(
                stacked_pc,
                self.success_rate,
                num_train=self.env_num_train
            ).to(self.device)
            self.map1 = self.CollisionPredictor1.pred_one_batch(
                stacked_pc,
                self.success_rate,
                num_train=self.env_num_train
            ).to(self.device)
            self.sampled_chair_pc[:, :, :3] = sampled_pc.repeat_interleave(self.env_per_chair, dim=0)
            self.sampled_chair_pc[:, :, 3] = self.map0.repeat_interleave(self.env_per_chair, dim=0)
            self.sampled_chair_pc[:, :, 4] = self.map1.repeat_interleave(self.env_per_chair, dim=0)

    def save(self, path, iteration) :

        super().save(path, iteration)
        self.CollisionPredictor0.save_checkpoint(os.path.join(path, "CP0_{}.pt".format(iteration)))
        self.CollisionPredictor1.save_checkpoint(os.path.join(path, "CP1_{}.pt".format(iteration)))

        save_pc = self.compute_point_cloud_state(pc=self.sampled_chair_pc)
        torch.save(self._detailed_view(save_pc)[:, 0, ...], os.path.join(path, "map01_{}.pt".format(iteration)))


    def load(self, path, iteration) :
        
        super().load(path, iteration)
        cp_file0 = os.path.join(path, "CP0_{}.pt".format(iteration))
        cp_file1 = os.path.join(path, "CP0_{}.pt".format(iteration))
        print("loading CP checkpoint", cp_file0, cp_file1)
        self.CollisionPredictor0.load_checkpoint(cp_file0)
        self.CollisionPredictor1.load_checkpoint(cp_file1)
        self._refresh_map()

    def _data_argumentation(self, pcd):
        pcd[:, :, :3] *= torch.rand((pcd.shape[0], 1, 3), device=self.device)*0.3 + 0.85
        return pcd
    
    def _get_transformed_pc(self, pc=None, mask=None) :

        if pc is None:
            pc = self.selected_chair_pc[:, :, :3]

        # select first env of each type of chair
        used_initial_root_state = self._detailed_view(self.initial_root_states)[:, 0, ...]

        transformed_pc = self._transform_pc(
            pc,
            used_initial_root_state[:, 1, :7]
        )

        return transformed_pc

    def update(self, iter) :

        CP_info = {}
        used_success_rate = self._detailed_view(self.success_rate).mean(dim=-1)
        
        if used_success_rate.mean() > self.success_rate_bar :
            # do training only when success rate is enough

            transformed_pc = self._get_transformed_pc()
            self.raw_map0 = self.get_map(self.selected_chair_pc[:, :, :3], self.contact_buffer_list0)
            self.raw_map1 = self.get_map(self.selected_chair_pc[:, :, :3], self.contact_buffer_list1)

            # stack them together to make resample easier
            stacked_pc_target0 = torch.cat(
                (
                    transformed_pc,
                    self.raw_map0.view(self.chair_num, -1, 1)
                ),
                dim=2
            )
            stacked_pc_target1 = torch.cat(
                (
                    transformed_pc,
                    self.raw_map1.view(self.chair_num, -1, 1)
                ),
                dim=2
            )

            # in training, sample a few points to train CP each epoch
            minibatch_size = self.cfg["cp"]["cp_minibatch_size"]
            for i in range(self.CP_iter):
                info_list = []
                sampled_pc_target0 = self.sample_points(
                    stacked_pc_target0,
                    self.chairPCDownsampleNum,
                    sample_method="furthest"
                )
                sampled_pc_target0 = self._data_argumentation(sampled_pc_target0)
                sampled_pc_target1 = self.sample_points(
                    stacked_pc_target1,
                    self.chairPCDownsampleNum,
                    sample_method="furthest"
                )
                sampled_pc_target1 = self._data_argumentation(sampled_pc_target1)
                for target, predictor in zip([sampled_pc_target0, sampled_pc_target1], [self.CollisionPredictor0, self.CollisionPredictor1]) :
                    for cur_pc_target, cur_success_rate in zip(
                            torch.split(target, minibatch_size),
                            torch.split(used_success_rate, minibatch_size)
                        ) :
                        # self._refresh_pointcloud_visualizer(cur_pc_target[0, :, :3], cur_pc_target[0, :, 3])
                        cur_map, cur_info = predictor.pred_one_batch(
                            cur_pc_target[:, :, :3],
                            cur_success_rate,
                            target=cur_pc_target[:, :, 3],
                            num_train=self.chair_num_train
                        )
                        info_list.append(cur_info)
            self.CollisionPredictor0.network_lr_scheduler.step()
            self.CollisionPredictor1.network_lr_scheduler.step()

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

        d0 = torch.norm(self.hand_tip_pos0 - self._get_max_point(self.sampled_chair_pc[:, :, :3], self.sampled_chair_pc[:, :, 3]), p=2, dim=-1)
        d1 = torch.norm(self.hand_tip_pos1 - self._get_max_point(self.sampled_chair_pc[:, :, :3], self.sampled_chair_pc[:, :, 4]), p=2, dim=-1)
        dist_reward0 = 1.0 / (1.0 + d0**2)
        dist_reward0 *= dist_reward0
        dist_reward0 = torch.where(d0 <= 0.1, dist_reward0*2, dist_reward0)
        dist_reward1 = 1.0 / (1.0 + d1**2)
        dist_reward1 *= dist_reward1
        dist_reward1 = torch.where(d1 <= 0.1, dist_reward1*2, dist_reward1)
        dist_reward = dist_reward0 + dist_reward1
        rew += dist_reward * self.cfg['cp']['max_point_reward']

        return rew, res

    def _refresh_observation(self) :
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        if self.cfg["env"]["driveMode"] == "ik" :
            self.gym.refresh_jacobian_tensors(self.sim)

        point_clouds = self.compute_point_cloud_state(pc=self.sampled_chair_pc)

        point_clouds[:, :, :3] -= self.initial_root_states[:, 2, :3].view(self.num_envs, 1, 3)
        self.obs_buf[:, :, :7].copy_(point_clouds)
        if self.cfg["cp"]["max_point_observation"] :
            max_point = torch.stack(
                (
                    self._get_max_point(self.sampled_chair_pc[:, :, :3], self.sampled_chair_pc[:, :, 3]),
                    self._get_max_point(self.sampled_chair_pc[:, :, :3], self.sampled_chair_pc[:, :, 4])
                )
            )
            max_point = torch.transpose(max_point, 0, 1)
            self.obs_buf[:, :, 7:] = self._get_base_observation(max_point).view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
        else :
            self.obs_buf[:, :, 7:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(point_clouds[0, :, :3], point_clouds[0, :, 4])