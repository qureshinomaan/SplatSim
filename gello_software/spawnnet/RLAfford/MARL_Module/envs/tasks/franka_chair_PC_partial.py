from isaacgym.torch_utils import *
import numpy as np
from tasks.franka_chair import TwoFrankaChair
import matplotlib.pyplot as plt
from pointnet2_ops import pointnet2_utils
from tqdm import tqdm
from MARL_Module.envs.utils.time_counter import TimeCounter
import os

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class TwoFrankaChairPCPartial(TwoFrankaChair) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.chairPCOriginalNum = cfg["env"]["chairPointOriginalNum"]
        self.chairPCDownsampleNum = cfg["env"]["chairPointDownsampleNum"]
        self.handPCDownsampleNum = cfg["env"]["handPointDownsampleNum"]
        self.pointCloudDownsampleNum = self.chairPCDownsampleNum + self.handPCDownsampleNum
        if not hasattr(self, "chair_mask_dim") :
            self.chair_mask_dim = 3
        
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

        self.chair_pc = self.chair_pc.repeat_interleave(self.env_per_chair, dim=0)
        self.sampled_chair_pc = torch.zeros((self.num_envs, self.chairPCDownsampleNum, self.chair_mask_dim), device=self.device)
        self.selected_chair_pc = self._detailed_view(self.chair_pc)[:, 0, ...]
        self._refresh_map()
    
    def _refresh_map(self) :

        self.sampled_chair_pc = self.sample_points(
            self.selected_chair_pc,
            self.chairPCDownsampleNum,
            sample_method="furthest"
        ).repeat_interleave(self.env_per_chair, dim=0)

    def _get_transformed_pc(self, pc=None, mask=None) :

        if pc is None:
            pc = self.selected_chair_pc[:, :, :3]

        # select first env of each type of chair
        used_initial_root_state = self._detailed_view(self.initial_root_states)[:, 0, ...]
        used_initial_rigd_state = self._detailed_view(self.initial_rigid_body_states)[:, 0, ...]

        transformed_pc = self._transform_pc(
            pc,
            used_initial_root_state[:, 2, :7]
        )

        return transformed_pc
    
    def get_map(self, raw_point_clouds, raw_buffer_list):

        map_dis_bar = self.cfg['env']['map_dis_bar']
        top_tensor = torch.tensor([x.top for x in raw_buffer_list], device=self.device)
        buffer_size = (raw_buffer_list[-1]).buffer.shape[0]

        buffer = torch.zeros((self.chair_num, buffer_size, 3)).to(self.device)
        for i in range(self.chair_num):
            buffer[i] = raw_buffer_list[i].buffer[:, 0:3]

        dist_mat = torch.cdist(raw_point_clouds, buffer, p=2)
        if_eff = dist_mat<map_dis_bar
        
        for i in range(self.chair_num):
            if_eff[i, :, top_tensor[i]:] = False

        tot = if_eff.sum(dim=2)
        tot_scale = tot/(tot.max()+1e-8)  # env*pc

        return tot_scale
    
    def save(self, path, iteration) :

        raw_map0 = self.get_map(self.selected_chair_pc[:, :, :3], self.contact_buffer_list0)
        raw_map1 = self.get_map(self.selected_chair_pc[:, :, :3], self.contact_buffer_list1)
        transformed_pc = self._get_transformed_pc()

        saving_raw_map = torch.cat((
            transformed_pc[:, :, :3],
            raw_map0.view(self.chair_num, -1, 1),
            raw_map1.view(self.chair_num, -1, 1)
        ), dim=-1)

        torch.save(saving_raw_map, os.path.join(path, "rawmap01_{}.pt".format(iteration)))
    
    def _transform_pc(self, pc, fixed_seven, moving_seven=None) :

        fixed_p = fixed_seven[:, :3]
        fixed_r = fixed_seven[:, 3:7]
        shape = pc.shape
        
        fixed_point_clouds = quat_apply(fixed_r.view(-1, 1, 4).repeat_interleave(shape[1], dim=1), pc[:, :, :3]) + fixed_p.view(-1, 1, 3)
        
        return fixed_point_clouds

    def compute_point_cloud_state(self, pc=None) :

        lfinger_p0 = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index[0], :3]
        lfinger_r0 = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index[0], 3:7]
        rfinger_p0 = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index[0], :3]
        rfinger_r0 = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index[0], 3:7]
        lfinger_p1 = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index[1], :3]
        lfinger_r1 = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index[1], 3:7]
        rfinger_p1 = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index[1], :3]
        rfinger_r1 = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index[1], 3:7]

        if pc == None :
            selected_pc = self.sample_points(self.chair_pc, self.chairPCDownsampleNum, sample_method='furthest')
        else :
            selected_pc = pc
        selected_lfinger_pc = self.sample_points(self.franka_left_finger_pc, self.handPCDownsampleNum//4, sample_method='furthest')
        selected_rfinger_pc = self.sample_points(self.franka_right_finger_pc, self.handPCDownsampleNum//4, sample_method='furthest')

        lfinger_shape = selected_lfinger_pc.shape
        rfinger_shape = selected_rfinger_pc.shape
        pc_masks = selected_pc[:, :, 3:].view(self.num_envs, self.chairPCDownsampleNum, -1)
        mask_num = pc_masks.shape[-1]
        finger_mask0 = torch.tensor([0]*mask_num+[1, 0], device=self.device)
        finger_mask1 = torch.tensor([0]*mask_num+[0, 1], device=self.device)

        lfinger_point_clouds0 = quat_apply(lfinger_r0.view(-1, 1, 4).repeat_interleave(lfinger_shape[1], dim=1), selected_lfinger_pc) + lfinger_p0.view(-1, 1, 3)
        rfinger_point_clouds0 = quat_apply(rfinger_r0.view(-1, 1, 4).repeat_interleave(rfinger_shape[1], dim=1), selected_rfinger_pc) + rfinger_p0.view(-1, 1, 3)
        lfinger_point_clouds1 = quat_apply(lfinger_r1.view(-1, 1, 4).repeat_interleave(lfinger_shape[1], dim=1), selected_lfinger_pc) + lfinger_p1.view(-1, 1, 3)
        rfinger_point_clouds1 = quat_apply(rfinger_r1.view(-1, 1, 4).repeat_interleave(rfinger_shape[1], dim=1), selected_rfinger_pc) + rfinger_p1.view(-1, 1, 3)
        merged_point_clouds = self._transform_pc(selected_pc[:, :, :3], self.root_tensor[:, 2, :7])

        merged_point_clouds = append_mask(torch.cat((merged_point_clouds, pc_masks), dim=-1), torch.tensor([0, 0], device=self.device))
        lfinger_point_clouds0 = append_mask(lfinger_point_clouds0, finger_mask0)
        rfinger_point_clouds0 = append_mask(rfinger_point_clouds0, finger_mask0)
        lfinger_point_clouds1 = append_mask(lfinger_point_clouds1, finger_mask1)
        rfinger_point_clouds1 = append_mask(rfinger_point_clouds1, finger_mask1)

        point_clouds = torch.cat((merged_point_clouds, lfinger_point_clouds0, rfinger_point_clouds0, lfinger_point_clouds1, rfinger_point_clouds1), dim=1)

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

        point_clouds = self.compute_point_cloud_state(pc=self.sampled_chair_pc)

        point_clouds[:, :, :3] -= self.initial_root_states[:, 2, :3].view(self.num_envs, 1, 3)
        self.obs_buf[:, :, :5].copy_(point_clouds)
        self.obs_buf[:, :, 5:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)

        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(
                [point_clouds[0, :, :3], self.contact_buffer_list0[0].all()[:, :3]],
                [point_clouds[0, :, 3], torch.ones((self.contact_buffer_list0[0].top,)) * 0.5]
            )


# HELPER FUNCTIONS

def append_mask(x, mask) :

    return torch.cat((x, mask.view(1, 1, -1).repeat_interleave(x.shape[0], dim=0).repeat_interleave(x.shape[1], dim=1)), dim=2)

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