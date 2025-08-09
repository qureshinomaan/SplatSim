import numpy as np
from isaacgym.torch_utils import *
from isaacgym import gymutil, gymtorch
from isaacgym import gymapi
import torch
from pointnet2_ops import pointnet2_utils

gym = gymapi.acquire_gym()
sim_params = gymapi.SimParams()

def set_sim_params_up_axis(sim_params, axis):
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity.x = 0
    sim_params.gravity.y = 0
    sim_params.gravity.z = -9.81

set_sim_params_up_axis(sim_params, "z")
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
gym.add_ground(sim, plane_params)
gym.add_ground(sim, plane_params)
camera_prop = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, camera_prop)


asset_root = "./assets"

# load table
table_asset_options = gymapi.AssetOptions()
table_asset_options.density = 500
table_asset_options.fix_base_link = True
table_asset_options.override_com = True  # recompute center of mesh
table_asset_options.override_inertia = True  # recompute inertia
table_asset = gym.load_asset(
    sim, asset_root, "dataset/pap_data/table/19203/mobility.urdf", table_asset_options)
# rig_dict = gym.get_asset_rigid_body_dict(table_asset)
# table_rig_name = list(rig_dict.keys())[0]
table_start_pose = gymapi.Transform()
table_start_pose.p = gymapi.Vec3(0.0, 0.3, 0.15)
table_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 1.0)

# load table_object
# 1: KitchenPot
fixed_object_1_asset = gym.load_asset(
    sim, asset_root, "dataset/pap_data/object-on-table/100015/mobility.urdf", table_asset_options)
fixed_object_1_start_pose = gymapi.Transform()
fixed_object_1_start_pose.p = gymapi.Vec3(0.03, 0.23, 0.33)
fixed_object_1_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 1.0)
# 2: Toaster
fixed_object_2_asset = gym.load_asset(
    sim, asset_root, "dataset/pap_data/object-on-table/103477/mobility.urdf", table_asset_options)
fixed_object_2_start_pose = gymapi.Transform()
fixed_object_2_start_pose.p = gymapi.Vec3(0.2, 0.4, 0.33)
fixed_object_2_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 1.0)
# 3: Bottle
fixed_object_3_asset = gym.load_asset(
    sim, asset_root, "dataset/pap_data/object-on-table/8848/mobility.urdf", table_asset_options)
fixed_object_3_start_pose = gymapi.Transform()
fixed_object_3_start_pose.p = gymapi.Vec3(-0.2, 0.4, 0.33)
fixed_object_3_start_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 1.0)



spacing = 1.25
lower = gymapi.Vec3(-spacing, -spacing, 0.0)
upper = gymapi.Vec3(spacing, spacing, spacing)
num_per_row = 10
envs = []
cameras = []
camera_tensor_list = []
camera_view_matrix_inv_list = []
camera_proj_matrix_list = []
env_origin = torch.zeros((50, 3), device=0, dtype=torch.float)
camera_props = gymapi.CameraProperties()
camera_props.width = 256
camera_props.height = 256
camera_props.enable_tensors = True
camera_u = torch.arange(0, camera_props.width, device=0)
camera_v = torch.arange(0, camera_props.height, device=0)

camera_v2, camera_u2 = torch.meshgrid(camera_v, camera_u, indexing='ij')
for i in range(50):
    env = gym.create_env(sim, lower, upper, num_per_row)
    envs.append(env)
    # create handle
    table_actor = gym.create_actor(
        env,
        table_asset,
        table_start_pose,
        "table{}-{}".format(0, i),
        i,
        0,
        0)
    fixed_object_1_actor = gym.create_actor(
        env,
        fixed_object_1_asset,
        fixed_object_1_start_pose,
        "fixed_object_1_{}-{}".format(0, i),
        i,
        0,
        0)
    fixed_object_2_actor = gym.create_actor(
        env,
        fixed_object_2_asset,
        fixed_object_2_start_pose,
        "fixed_object_2_{}-{}".format(0, i),
        i,
        0,
        0)
    fixed_object_3_actor = gym.create_actor(
        env,
        fixed_object_3_asset,
        fixed_object_3_start_pose,
        "fixed_object_3_{}-{}".format(0, i),
        i,
        0,
        0)

    camera_handle1 = gym.create_camera_sensor(env, camera_props)
    print(camera_handle1)
    exit()
    # set on the front and look towards bottom
    gym.set_camera_location(camera_handle1, env, gymapi.Vec3(0.0,0.3,2.5), gymapi.Vec3(0.01,0.31, 1.5))
    cam1_tensor = gym.get_camera_image_gpu_tensor(sim, env, camera_handle1, gymapi.IMAGE_DEPTH)
    torch_cam1_tensor = gymtorch.wrap_tensor(cam1_tensor)
    cam1_vinv = torch.inverse((torch.tensor(gym.get_camera_view_matrix(sim, env, camera_handle1)))).to(0)
    cam1_proj = torch.tensor(gym.get_camera_proj_matrix(sim, env, camera_handle1), device=0)
    per_env_camera_tensor_list = [torch_cam1_tensor]
    per_env_camera_view_matrix_inv_list = [cam1_vinv]
    per_env_camera_proj_matrix_list = [cam1_proj]
    cameras.append([camera_handle1])
    camera_tensor_list.append(per_env_camera_tensor_list)
    camera_view_matrix_inv_list.append(per_env_camera_view_matrix_inv_list)
    camera_proj_matrix_list.append(per_env_camera_proj_matrix_list)

    origin = gym.get_env_origin(env)
    env_origin[i][0] = origin.x
    env_origin[i][1] = origin.y
    env_origin[i][2] = origin.z

gym.viewer_camera_look_at(viewer, None, gymapi.Vec3(5, 5, 1), gymapi.Vec3(6, 6, 0))

def depth_image_to_point_cloud_GPU(camera_tensor, camera_view_matrix_inv, camera_proj_matrix, u, v, width, height, depth_bar):
    # time1 = time.time()
    depth_buffer = camera_tensor.to(0)
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
    position = torch.vstack((X, Y, Z, torch.ones(len(X), device=0)))[:, valid]
    position = position.permute(1, 0)
    position = position@vinv
    points = position[:, 0:3]
    return points

def sample_points(points, sample_num=1000, sample_method='random'):
    eff_points = points[points[:, 2]>0.04]
    sampled_points_id = pointnet2_utils.furthest_point_sample(eff_points.reshape(1, *eff_points.shape), sample_num)
    sampled_points = eff_points.index_select(0, sampled_points_id[0].long())
    return sampled_points

def compute_point_cloud_state(depth_bar):
    gym.render_all_camera_sensors(sim)
    gym.start_access_image_tensors(sim)
    num_envs=50
    pointCloudDownsampleNum=2048
    point_clouds = torch.zeros((num_envs, pointCloudDownsampleNum, 3), device=0)
    for i in range(num_envs):

        points1 = depth_image_to_point_cloud_GPU(camera_tensor_list[i][0], camera_view_matrix_inv_list[i][0], camera_proj_matrix_list[i][0], camera_u2, camera_v2, camera_props.width, camera_props.height, depth_bar)
        
        points = points1

        selected_points = sample_points(points, sample_num=pointCloudDownsampleNum, sample_method='furthest')
        point_clouds[i]=selected_points

    gym.end_access_image_tensors(sim)

    point_clouds -= env_origin.view(num_envs, 1, 3)
        
    return point_clouds



while not gym.query_viewer_has_closed(viewer):
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    gym.sync_frame_time(sim)
    point_clouds = compute_point_cloud_state(5.)
    # print(point_clouds.shape)
    # torch.save(point_clouds, "/home/shengjie/gyr/E2EAff/assets/dataset/pap_data/table_pc/point_clouds_tensor")
    # exit()

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)



