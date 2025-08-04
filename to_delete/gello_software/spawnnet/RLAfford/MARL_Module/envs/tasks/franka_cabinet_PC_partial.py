from isaacgym.torch_utils import *
import numpy as np
from tasks.franka_cabinet import OneFrankaCabinet
import matplotlib.pyplot as plt
from pointnet2_ops import pointnet2_utils
import os

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class OneFrankaCabinetPCPartial(OneFrankaCabinet) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.cabinetPCOriginalNum = cfg["env"]["cabinetPointOriginalNum"]
        self.cabinetPCDownsampleNum = cfg["env"]["cabinetPointDownsampleNum"]
        self.handPCDownsampleNum = cfg["env"]["handPointDownsampleNum"]
        self.pointCloudDownsampleNum = self.cabinetPCDownsampleNum + self.handPCDownsampleNum
        if not hasattr(self, "cabinet_mask_dim") :
            self.cabinet_mask_dim = 4
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, log_dir=log_dir)

        self.num_obs += 5
        self.task_meta["mask_dim"] = 2
        self.task_meta["obs_dim"] = self.num_obs
        self.num_feature = cfg["env"]["pointFeatureDim"]
        self.obs_buf = torch.zeros((self.num_envs, self.pointCloudDownsampleNum, self.num_obs), device=self.device, dtype=torch.float)
        
        if cfg["env"]["visualizePointcloud"] == True :
            import open3d as o3d
            from utils.o3dviewer import PointcloudVisualizer
            self.pointCloudVisualizer = PointcloudVisualizer()
            self.pointCloudVisualizerInitialized = False
            self.o3d_pc = o3d.geometry.PointCloud()
        else :
            self.pointCloudVisualizer = None

        self.cabinet_pc = self.cabinet_pc.repeat_interleave(self.env_per_cabinet, dim=0)
        self.sampled_cabinet_pc = torch.zeros((self.num_envs, self.cabinetPCDownsampleNum, self.cabinet_mask_dim), device=self.device)
        self.selected_cabinet_pc = self._detailed_view(self.cabinet_pc)[:, 0, ...]
        self._refresh_map()
    
    def _refresh_map(self) :

        self.sampled_cabinet_pc = self.sample_points(
            self.selected_cabinet_pc,
            self.cabinetPCDownsampleNum,
            sample_method="furthest"
        ).repeat_interleave(self.env_per_cabinet, dim=0)

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
    
    def get_map(self, raw_point_clouds, raw_mask, raw_buffer_list):

        map_dis_bar = self.cfg['env']['map_dis_bar']
        top_tensor = torch.tensor([x.top for x in self.contact_buffer_list], device=self.device)
        buffer_size = (raw_buffer_list[-1]).buffer.shape[0]

        buffer = torch.zeros((self.cabinet_num, buffer_size, 3)).to(self.device)
        for i in range(self.cabinet_num):
            buffer[i] = raw_buffer_list[i].buffer[:, 0:3]

        dist_mat = torch.cdist(raw_point_clouds, buffer, p=2)
        if_eff = dist_mat<map_dis_bar
        
        for i in range(self.cabinet_num):
            if_eff[i, :, top_tensor[i]:] = False

        tot = if_eff.sum(dim=2) * raw_mask
        # tot = torch.log(tot+1)
        tot_scale = tot/(tot.max()+1e-8)  # env*pc
        # tot_scale = tot_scale * raw_mask

        return tot_scale
    
    def save(self, path, iteration) :

        self.raw_map = self.get_map(self.selected_cabinet_pc[:, :, :3], self.selected_cabinet_pc[:, :, 3], self.contact_buffer_list)
        transformed_pc = self._get_transformed_pc()
        saving_raw_map = torch.cat((transformed_pc[:, :, :3], self.raw_map[:, :].view(self.cabinet_num, -1, 1)), dim=-1)
        torch.save(saving_raw_map, os.path.join(path, "rawmap_{}.pt".format(iteration)))

    '''
    The following code is used to enhance the pointcloud density on edges,
    which we think may help the detection of critical parts of objects.
    However the results showed the edge enhancement is uneecessary. 
    '''
    # def edge_detection(self, pc) :

    #     pc_pos = pc[:, :, :3] + pc[:, :, 3:4]        # add the mask prevents two parts interfering each other

    #     num_pc = pc.shape[0]    # number of point clouds
    #     num_p = pc.shape[1]        # number of points in a point cloud
    #     num_top_k = 32            # top-k used to calculate pca
    #     top_dist = 0.3            # dist of the ball

    #     # The parallel version of this block of code requires huge memory, so abandoned
    #     # pointwise_dist = torch.cdist(pc_pos, pc_pos, p=2)
    #     # top_k_id = torch.topk(pointwise_dist, k=num_top_k, largest=False, dim=-1).indices

    #     # pc_repeat = pc_pos.view(num_pc, num_p, 1, 3).repeat_interleave(num_top_k, dim=2)
    #     # top_k_id_repeat = top_k_id.view(num_pc, num_p, num_top_k, 1).repeat_interleave(3, dim=3)

    #     # gathered = torch.gather(pc_repeat, dim=1, index=top_k_id_repeat)
    #     # gathered = gathered - pc_pos.view(num_pc, num_p, 1, 3)

    #     # gdist = torch.norm(gathered, p=2, dim=-1, keepdim=True)
    #     # gathered *= (gdist < top_dist)

    #     # u, s, v = torch.pca_lowrank(gathered, q=3)

    #     # min_eig = torch.min(s, dim=-1, keepdim=False).values

    #     # kth = torch.kthvalue(min_eig, min_eig.shape[1]-self.cabinetPCDownsampleNum, dim=-1).values

    #     top_k_id = torch.zeros((num_pc, num_p, num_top_k), device=self.device).long()

    #     with tqdm(total=num_pc) as pbar:
    #         pbar.set_description('Preparing Point Cloud:')
    #         for i in range(num_pc) :

    #             pointwise_dist = torch.cdist(pc_pos[i], pc_pos[i], p=2)
    #             top_k_id[i] = torch.topk(pointwise_dist, k=num_top_k, largest=False, dim=-1).indices

    #             pbar.update(1)

    #     pc_repeat = pc_pos.view(num_pc, num_p, 1, 3).repeat_interleave(num_top_k, dim=2)
    #     top_k_id_repeat = top_k_id.view(num_pc, num_p, num_top_k, 1).repeat_interleave(3, dim=3)

    #     gathered = torch.gather(pc_repeat, dim=1, index=top_k_id_repeat)
    #     gathered = gathered - pc_pos.view(num_pc, num_p, 1, 3)

    #     gdist = torch.norm(gathered, p=2, dim=-1, keepdim=True)
    #     gathered *= (gdist < top_dist)

    #     u, s, v = torch.pca_lowrank(gathered, q=3)

    #     min_eig = torch.min(s, dim=-1, keepdim=False).values

    #     kth = torch.kthvalue(min_eig, min_eig.shape[1]-self.cabinetPCDownsampleNum, dim=-1, keepdim=True).values

    #     var = torch.var(min_eig)
    #     min_eig = (min_eig - kth) / (var + 1e-9)

    #     return torch.sigmoid(min_eig*0.05)
    
    def _transform_pc(self, pc, moving_mask, fixed_seven, moving_seven) :

        fixed_p = fixed_seven[:, :3]
        fixed_r = fixed_seven[:, 3:7]
        moving_p = moving_seven[:, :3]
        moving_r = moving_seven[:, 3:7]

        shape = pc.shape
        
        fixed_point_clouds = quat_apply(fixed_r.view(-1, 1, 4).repeat_interleave(shape[1], dim=1), pc[:, :, :3]) + fixed_p.view(-1, 1, 3)
        moving_point_clouds = quat_apply(moving_r.view(-1, 1, 4).repeat_interleave(shape[1], dim=1), pc[:, :, :3]) + moving_p.view(-1, 1, 3)
        merged_point_clouds = torch.where(moving_mask>0.5, moving_point_clouds, fixed_point_clouds)

        return merged_point_clouds

    def compute_point_cloud_state(self, pc=None) :

        lfinger_p = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index, :3]
        lfinger_r = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index, 3:7]
        rfinger_p = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index, :3]
        rfinger_r = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index, 3:7]

        if pc == None :
            selected_pc = self.sample_points(self.cabinet_pc, self.cabinetPCDownsampleNum, sample_method='furthest')
        else :
            selected_pc = pc
        selected_lfinger_pc = self.sample_points(self.franka_left_finger_pc, self.handPCDownsampleNum//2, sample_method='furthest')
        selected_rfinger_pc = self.sample_points(self.franka_right_finger_pc, self.handPCDownsampleNum//2, sample_method='furthest')

        lfinger_shape = selected_lfinger_pc.shape
        rfinger_shape = selected_rfinger_pc.shape
        moving_mask = selected_pc[:, :, 3].view(self.num_envs, self.cabinetPCDownsampleNum, 1)
        pc_masks = selected_pc[:, :, 3:].view(self.num_envs, self.cabinetPCDownsampleNum, -1)
        mask_num = pc_masks.shape[-1]
        finger_mask = torch.tensor([0]*mask_num+[1], device=self.device)

        lfinger_point_clouds = quat_apply(lfinger_r.view(-1, 1, 4).repeat_interleave(lfinger_shape[1], dim=1), selected_lfinger_pc) + lfinger_p.view(-1, 1, 3)
        rfinger_point_clouds = quat_apply(rfinger_r.view(-1, 1, 4).repeat_interleave(rfinger_shape[1], dim=1), selected_rfinger_pc) + rfinger_p.view(-1, 1, 3)
        merged_point_clouds = self._transform_pc(selected_pc[:, :, :3], moving_mask, self.root_tensor[:, 1, :7], self.rigid_body_tensor[:, self.cabinet_rigid_body_index, :7])

        merged_point_clouds = append_mask(torch.cat((merged_point_clouds, pc_masks), dim=-1), torch.tensor([0], device=self.device))
        lfinger_point_clouds = append_mask(lfinger_point_clouds, finger_mask)
        rfinger_point_clouds = append_mask(rfinger_point_clouds, finger_mask)

        point_clouds = torch.cat((merged_point_clouds, lfinger_point_clouds, rfinger_point_clouds), dim=1)

        return point_clouds


    def rand_row(self, tensor, dim_needed):  
        row_total = tensor.shape[0]
        return tensor[torch.randint(low=0, high=row_total, size=(dim_needed,)),:]

    # @TimeCounter
    def sample_points(self, points, sample_num=1000, sample_method='random', sample_prob=None):
        eff_points = points
        if sample_method == 'random':
            sampled_points = self.rand_row(eff_points, sample_num)
        elif sample_method == 'furthest':
            idx = pointnet2_utils.furthest_point_sample(points[:, :, :3].contiguous().cuda(), sample_num).long().to(self.device)
            idx = idx.view(*idx.shape, 1).repeat_interleave(points.shape[-1], dim=2)
            sampled_points = torch.gather(points, dim=1, index=idx)
        elif sample_method == 'edge' :
            if sample_prob == None :
                sample_prob = torch.ones((eff_points.shape[0], eff_points.shape[1]), device=self.device)
            # idx = torch.topk(sample_prob, sample_num, dim=-1).indices
            idx = torch.multinomial(sample_prob, sample_num)
            idx = idx.view(*idx.shape, 1).repeat_interleave(points.shape[-1], dim=2)
            sampled_points = torch.gather(points, dim=1, index=idx)
        return sampled_points
    
    def _refresh_pointcloud_visualizer(self, point_clouds, data) :

        if isinstance(point_clouds, list) :
            points = np.concatenate([a.cpu().numpy() for a in point_clouds], axis=0)
        else :
            points = point_clouds.cpu().numpy()
        
        if isinstance(data, list):
            colors = np.concatenate([a.cpu().numpy() for a in data], axis=0)
        else :
            colors = data.cpu().numpy()

        import open3d as o3d
        colors = plt.get_cmap()(colors)[:, :3]
        self.o3d_pc.points = o3d.utility.Vector3dVector(points)
        self.o3d_pc.colors = o3d.utility.Vector3dVector(colors)

        if self.pointCloudVisualizerInitialized == False :
            self.pointCloudVisualizer.add_geometry(self.o3d_pc)
            self.pointCloudVisualizerInitialized = True
        else :
            self.pointCloudVisualizer.update(self.o3d_pc)
    
    def _refresh_observation(self) :
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        if self.cfg["env"]["driveMode"] == "ik" :
            self.gym.refresh_jacobian_tensors(self.sim)

        point_clouds = self.compute_point_cloud_state(pc=self.sampled_cabinet_pc)

        point_clouds[:, :, :3] -= self.franka_root_tensor[:, :3].view(-1, 1, 3)
        self.obs_buf[:, :, :5].copy_(point_clouds)
        self.obs_buf[:, :, 5:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)

        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(
                [point_clouds[0, :, :3], self.contact_buffer_list[0].all()[:, :3]],
                [point_clouds[0, :, 3], torch.ones((self.contact_buffer_list[0].top,)) * 0.5]
            )


# HELPER FUNCTIONS

def append_mask(x, mask) :

    return torch.cat((x, mask.view(1, 1, -1).repeat_interleave(x.shape[0], dim=0).repeat_interleave(x.shape[1], dim=1)), dim=2)

'''
This function is used to compute realtime pointcloud.
However, Isaacgym didn't support realtime pointcloud capturing very well,
from our observation, Isaacgym will not process multiple captures in parallel,
so we end up using simulated pointcloud, not realtime pointcloud.
'''
@torch.jit.script
def depth_image_to_point_cloud_GPU(camera_tensor, camera_view_matrix_inv, camera_proj_matrix, u, v, width:float, height:float, depth_bar:float, device:torch.device):
    # time1 = time.time()
    depth_buffer = camera_tensor.to(device)

    # Get the camera view matrix and invert it to transform points from camera to world space
    vinv = camera_view_matrix_inv

    # Get the camera projection matrix and get the necessary scaling
    # coefficients for deprojection
    
    proj = camera_proj_matrix
    fu = 2/proj[0, 0]
    fv = 2/proj[1, 1]

    centerU = width/2
    centerV = height/2

    Z = depth_buffer
    X = -(u-centerU)/width * Z * fu
    Y = (v-centerV)/height * Z * fv

    Z = Z.view(-1)
    valid = Z > -depth_bar
    X = X.view(-1)
    Y = Y.view(-1)

    position = torch.vstack((X, Y, Z, torch.ones(len(X), device=device)))[:, valid]
    position = position.permute(1, 0)
    position = position@vinv

    points = position[:, 0:3]

    return points