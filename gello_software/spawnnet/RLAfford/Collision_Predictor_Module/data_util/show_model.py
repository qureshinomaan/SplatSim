from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
from isaacgym.torch_utils import *

import math
import numpy as np
import torch
import random
import time

def get_dof_property(gym, asset) :
    dof_props = gym.get_asset_dof_properties(asset)
    dof_num = gym.get_asset_dof_count(asset)
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

torch.set_printoptions(precision=4, sci_mode=False)

# acquire gym interface
gym = gymapi.acquire_gym()
custom_parameters = [
    {"name": "--num_envs", "type": int, "default": 1, "help": "Number of environments to create"},
]
args = gymutil.parse_arguments(
    description="Franka Jacobian Inverse Kinematics (IK) + Operational Space Control (OSC) Example",
    custom_parameters=custom_parameters,
)

# set torch device
device = args.sim_device if args.use_gpu_pipeline else 'cpu'

# configure sim
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
sim_params.use_gpu_pipeline = args.use_gpu_pipeline
if args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.rest_offset = 0.0
    sim_params.physx.contact_offset = 0.001
    sim_params.physx.friction_offset_threshold = 0.001
    sim_params.physx.friction_correlation_distance = 0.0005
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
else:
    raise Exception("This example can only be used with PhysX")


# create sim
sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    raise Exception("Failed to create sim")

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")
franka_asset_root = "../../assets"
franka_asset_file = "franka_description/robots/franka_panda.urdf"
cabinet1_asset_root = "/home/boshi/Documents/Science/E2EAff/assets/dataset/custom_cabinet/custom/"
cabinet1_asset_file = "mobility.urdf"
cabinet2_asset_root = "/home/boshi/Documents/Science/E2EAff/assets/dataset/one_door_cabinet/44781_link_0"
cabinet2_asset_file = "mobility.urdf"

asset_options = gymapi.AssetOptions()
asset_options.armature = 0.01
asset_options.fix_base_link = True
asset_options.disable_gravity = False
asset_options.flip_visual_attachments = False
# franka_asset = gym.load_asset(sim, franka_asset_root, franka_asset_file, asset_options)
cabinet1_asset = gym.load_asset(sim, cabinet1_asset_root, cabinet1_asset_file, asset_options)
cabinet2_asset = gym.load_asset(sim, cabinet2_asset_root, cabinet2_asset_file, asset_options)

# configure env grid
num_envs = args.num_envs
num_per_row = int(math.sqrt(num_envs))
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
env_upper = gymapi.Vec3(spacing, spacing, spacing)
print("Creating %d environments" % num_envs)

# add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

envs = []
box_idxs = []
hand_idxs = []
init_pos_list = []
init_rot_list = []

cabinet1_pose = gymapi.Transform()
cabinet1_pose.p = gymapi.Vec3(0.0, 0.0, 0.305)
cabinet2_pose = gymapi.Transform()
cabinet2_pose.p = gymapi.Vec3(1.0, 0.0, 1.0)

def _draw_line(src, dst) :
    line_vec = np.stack([np.array(src), np.array(dst)]).flatten().astype(np.float32)
    color = np.array([1,0,0], dtype=np.float32)
    gym.clear_lines(viewer)
    gym.add_lines(
        viewer,
        envs[0],
        num_envs,
        line_vec,
        color
    )

for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # add table
    cabinet_handle = gym.create_actor(env, cabinet1_asset, cabinet1_pose, "door", i, 0)
    cabinet_handle = gym.create_actor(env, cabinet2_asset, cabinet2_pose, "cabinet", i, 0)

# point camera at middle env
cam_pos = gymapi.Vec3(1, 1, 1)
cam_target = gymapi.Vec3(-1, -1, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

gym.prepare_sim(sim)

dof_state_tensor = gymtorch.wrap_tensor(gym.acquire_dof_state_tensor(sim)).view(num_envs, -1, 2)
dof_effort, dof_lower, dof_upper = get_dof_property(gym, cabinet1_asset)
dof_lower = dof_lower[0]
dof_upper = dof_upper[0]
ddof = 0.1

# simulation loop
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.refresh_dof_state_tensor(sim)

    gym.simulate(sim)

    if dof_state_tensor[0, 0, 0] + ddof > dof_upper or dof_state_tensor[0, 0, 0] + ddof < dof_lower :
        ddof *= -1
        
    dof_state_tensor[:, 0, 0] += ddof
    gym.set_dof_state_tensor(
        sim,
        gymtorch.unwrap_tensor(dof_state_tensor)
    )

    _draw_line([0,0,0], [0,0,0.07])

    gym.fetch_results(sim, True)

    # update viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, False)
    gym.sync_frame_time(sim)

# cleanup
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)