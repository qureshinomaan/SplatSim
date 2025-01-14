from isaacgym.torch_utils import *
import numpy as np
from MARL_Module.envs.utils.o3dviewer import PointcloudVisualizer
from tasks.franka_grasp import OneFrankaGrasp
import matplotlib.pyplot as plt
from pointnet2_ops import pointnet2_utils
from tqdm import tqdm

def quat_axis(q, axis=0):
    """ ?? """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)

class OneFrankaGraspPCPartial(OneFrankaGrasp) :

    def __init__(self, cfg, sim_params, physics_engine, device_type, device_id, headless, agent_index=[[[0, 1, 2, 3, 4, 5]], [[0, 1, 2, 3, 4, 5]]], is_multi_agent=False, log_dir=None):

        self.objectPCDownsampleNum = cfg["env"]["objectPointDownsampleNum"]
        self.handPCDownsampleNum = cfg["env"]["handPointDownsampleNum"]
        self.pointCloudDownsampleNum = self.objectPCDownsampleNum + self.handPCDownsampleNum 
        
        super().__init__(cfg, sim_params, physics_engine, device_type, device_id, headless, log_dir=log_dir)

        self.num_feature = cfg["env"]["pointFeatureDim"]
        self.obs_buf = torch.zeros((self.num_envs, self.pointCloudDownsampleNum, self.num_obs), device=self.device, dtype=torch.float)
        self.task_meta["mask_dim"] = 2
        
        if cfg["env"]["visualizePointcloud"] == True :
            import open3d as o3d
            from utils.o3dviewer import PointcloudVisualizer
            self.pointCloudVisualizer = PointcloudVisualizer()
            self.pointCloudVisualizerInitialized = False
            self.o3d_pc = o3d.geometry.PointCloud()
        else :
            self.pointCloudVisualizer = None
        
        self.object_pc = self.object_pc.repeat_interleave(self.env_per_object, dim=0)

    def compute_point_cloud_state(self) :

        fixed_p = self.root_tensor[:, 1, :3]
        fixed_r = self.root_tensor[:, 1, 3:7]
        lfinger_p = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index, :3]
        lfinger_r = self.rigid_body_tensor[:, self.hand_lfinger_rigid_body_index, 3:7]
        rfinger_p = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index, :3]
        rfinger_r = self.rigid_body_tensor[:, self.hand_rfinger_rigid_body_index, 3:7]

        selected_pc = self.sample_points(self.object_pc, self.objectPCDownsampleNum, sample_method='edge')
        selected_lfinger_pc = self.sample_points(self.franka_left_finger_pc, self.handPCDownsampleNum//2, sample_method='edge')
        selected_rfinger_pc = self.sample_points(self.franka_right_finger_pc, self.handPCDownsampleNum//2, sample_method='edge')

        object_shape = selected_pc.shape
        lfinger_shape = selected_lfinger_pc.shape
        rfinger_shape = selected_rfinger_pc.shape

        fixed_point_clouds = quat_apply(fixed_r.view(-1, 1, 4).repeat_interleave(object_shape[1], dim=1), selected_pc[:, :, :3]) + fixed_p.view(-1, 1, 3)
        lfinger_point_clouds = quat_apply(lfinger_r.view(-1, 1, 4).repeat_interleave(lfinger_shape[1], dim=1), selected_lfinger_pc) + lfinger_p.view(-1, 1, 3)
        rfinger_point_clouds = quat_apply(rfinger_r.view(-1, 1, 4).repeat_interleave(rfinger_shape[1], dim=1), selected_rfinger_pc) + rfinger_p.view(-1, 1, 3)

        fixed_point_clouds = append_mask(fixed_point_clouds, torch.tensor([1,0], device=self.device))
        lfinger_point_clouds = append_mask(lfinger_point_clouds, torch.tensor([0,1], device=self.device))
        rfinger_point_clouds = append_mask(rfinger_point_clouds, torch.tensor([0,1], device=self.device))

        point_clouds = torch.cat((fixed_point_clouds, lfinger_point_clouds, rfinger_point_clouds), dim=1)

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
            sampled_points_id = pointnet2_utils.furthest_point_sample(eff_points.reshape(1, *eff_points.shape), sample_num)
            sampled_points = eff_points.index_select(0, sampled_points_id[0].long())
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
        hand_pos = self.hand_tip_pos - self.franka_root_tensor[:, :3]
        hand_color = np.zeros(hand_pos.shape)
        hand_color[:, 0] = 1
        points = np.concatenate((points, hand_pos.cpu().numpy()), axis=0)
        colors = plt.get_cmap()(colors)[:, :3]
        colors = np.concatenate((colors, hand_color), axis=0)
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

        point_clouds = self.compute_point_cloud_state()

        # print(point_clouds[0].mean(dim=0), point_clouds[1].mean(dim=0))

        point_clouds[:, :, :3] -= self.franka_root_tensor[:, :3].view(-1, 1, 3)
        self.obs_buf[:, :, :5].copy_(point_clouds)
        self.obs_buf[:, :, 5:] = self._get_base_observation().view(self.num_envs, 1, -1).repeat_interleave(self.pointCloudDownsampleNum, dim=1)
        # self.obs_buf_states = self._get_base_observation()

        if self.pointCloudVisualizer != None :
            self._refresh_pointcloud_visualizer(
                [point_clouds[0, :, :3], transform(self.contact_buffer_list[0].all()[:, :3], self.object_root_tensor[:, :3]-self.franka_root_tensor[:, :3],  self.object_root_tensor[:, 3:7])],
                [point_clouds[0, :, 3], torch.ones((self.contact_buffer_list[0].top,)) * 0.5]
            )


# HELPER FUNCTIONS

def transform(x, pos, rot) :

    return quat_apply(rot, x) + pos

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