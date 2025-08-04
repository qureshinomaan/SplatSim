from cv2 import transform
from isaacgym import gymtorch
from isaacgym import gymapi
from sympy import re
import torch
from isaacgym.torch_utils import *
import random
import math
# get parameters 
import numpy as np
import ipdb
from utils import from_transform_to_tensor
import copy

asset_root = "assets"
cabinet_num = 46019
link_num = 0
asset_file = "dataset/one_door_cabinet/"+str(cabinet_num)+"_link_"+str(link_num)+"/mobility.urdf"
buffer_folder_path = "MARL_Module/runs/cabinet"
obj_type = str(cabinet_num)+"_link_0"
movable_link_name = "link_"+str(link_num)
num_step = 1024
obj_buffer_path = buffer_folder_path + '/' + obj_type + '/' + str(num_step) + '.pt'
print(obj_buffer_path)
buffer = torch.load(obj_buffer_path)
print(buffer.shape)


def create_sim(gym):
    # get default set of parameters
    sim_params = gymapi.SimParams()

    # set common parameters
    sim_params.dt = 1 / 60
    sim_params.substeps = 2
    sim_params.up_axis = gymapi.UP_AXIS_Z   # Z_axis up
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)  # gravity

    # set PhysX-specific parameters
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 6
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.contact_offset = 0.01
    sim_params.physx.rest_offset = 0.0

    # set Flex-specific parameters
    sim_params.flex.solver_type = 5
    sim_params.flex.num_outer_iterations = 4
    sim_params.flex.num_inner_iterations = 20
    sim_params.flex.relaxation = 0.8
    sim_params.flex.warm_start = 0.5

    # set GPU ID
    compute_device_id = 0
    graphics_device_id = 0

    # create simulator
    sim = gym.create_sim(compute_device_id, graphics_device_id, gymapi.SIM_PHYSX, sim_params)
    return sim

def create_ground(gym, sim):
    # configure the ground plane
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1) # z-up!
    plane_params.distance = 0
    plane_params.static_friction = 1
    plane_params.dynamic_friction = 1
    plane_params.restitution = 0

    # create the ground plane
    gym.add_ground(sim, plane_params)

def load_obj(gym, sim, asset_root, asset_file):
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = True
    asset_options.disable_gravity = True
    asset_options.collapse_fixed_joints = True
    # Switch Meshes from Z-up left-handed system to Y-up Right-handed coordinate system.
    asset_options.flip_visual_attachments = False
    asset_options.armature = 0.01
    asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    return asset

def get_obj_pos():
    obj_pose = gymapi.Transform()
    obj_pose.p = gymapi.Vec3(*get_axis_params(1.5, 2)) 
    obj_pose.r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
    return obj_pose


def set_camera(gym, envs, viewer, num_per_row):
    global num_envs
    cam_pos = gymapi.Vec3(7, 0, 2)
    cam_target = gymapi.Vec3(-20, 0, 0)
    middle_env = envs[num_envs // 2 + num_per_row // 2]
    gym.viewer_camera_look_at(viewer, middle_env, cam_pos, cam_target)

def get_viewer(gym, sim):
    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise Exception("Failed to create viewer")
    return viewer

def get_absolute_pose(movable_link_pose, movable_link_ori, relative_pose):
    relative_pose = torch.tensor(relative_pose, device=0)
    relative_pose = relative_pose[0:3]
    return quat_rotate(movable_link_ori, movable_link_pose)+relative_pose


# main function
if __name__ == '__main__':
    gym = gymapi.acquire_gym()
    sim = create_sim(gym)
    create_ground(gym, sim)
    obj_asset = load_obj(gym, sim, asset_root, asset_file)
    obj_pose = get_obj_pos()
    obj_link_dict = gym.get_asset_rigid_body_dict(obj_asset)
    print(obj_link_dict)

    # set up the env grid
    num_envs = 1
    envs_per_row = 1
    env_spacing = 2.0
    env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing)
    env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

    # cache some common handles for later use
    envs = []
    actor_handles = []
    
    # create viewer
    viewer = get_viewer(gym, sim)

    # create and populate the environments
    for i in range(num_envs):
        env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
        envs.append(env)

        obj_handle = gym.create_actor(env, obj_asset, obj_pose, "obj", i, 1)
        actor_handles.append(obj_handle)


    movable_link_handle = gym.find_actor_rigid_body_handle(envs[0], actor_handles[0], movable_link_name)
    # movable_link_index = gym.find_actor_rigid_body_index(envs[0], actor_handles[0], movable_link_name)
    print(movable_link_handle)
    # print(movable_link_index)  # To the Assigned IndexDomain, the index of the joint.
    set_camera(gym, envs, viewer, envs_per_row)
    gym.prepare_sim(sim)

    i=0
    while not gym.query_viewer_has_closed(viewer):
        i+=1
        # step the physics
        gym.simulate(sim)
        gym.fetch_results(sim, True)

        # refresh tensors
        gym.refresh_rigid_body_state_tensor(sim)
        gym.refresh_dof_state_tensor(sim)
        gym.refresh_jacobian_tensors(sim)
        gym.refresh_mass_matrix_tensors(sim)

        # update viewer
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, False)
        gym.sync_frame_time(sim)

        transform = gym.get_rigid_transform(env, movable_link_handle)
        movable_link_pose, movable_link_ori = from_transform_to_tensor(transform)
        # show pointcloud
        # PC = np.load("assets/dataset/one_door_cabinet/46019_link_0/point_sample/full_PC.npy")
        # for point in PC:
        #     point[2]+=1
        #     p = copy.deepcopy(point)
        #     p[0]-=0.03
        #     gym.add_lines(viewer, env, 1, [p,point], [1,0,0])

        for relative_pose in buffer:

            # absolute_pose = get_absolute_pose(movable_link_pose, movable_link_ori, relative_pose)
            # gym.add_lines(viewer, envs[0], 1, [float(absolute_pose[0][0]), 
            #     float(absolute_pose[0][1]), float(absolute_pose[0][2]), 
            #     float(absolute_pose[0][0])+0.1, float(absolute_pose[0][1]),
            #     float(absolute_pose[0][2])], [1, 0, 0])

            pos = movable_link_pose + quat_apply(movable_link_ori, relative_pose[0:3]).view(-1)
            d_pos = pos.clone()
            # ipdb.set_trace()
            d_pos[0][1]-=0.03
            gym.add_lines(viewer, env, 1, [np.array(d_pos[0].cpu()), np.array(pos[0].cpu())], [0,0,1])
        if i%100==0:
            gym.clear_lines(viewer)
    # cleanup
    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)
