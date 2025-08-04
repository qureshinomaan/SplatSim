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

def create_cabinet(gym, sim, cabinet_path):
	asset_root = "../../assets"
	asset_file = cabinet_path
	asset_options = gymapi.AssetOptions()
	asset_options.fix_base_link = True
	asset_options.disable_gravity = True
	asset_options.collapse_fixed_joints = True
	cabinet_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
	return cabinet_asset

def show(cabinet_path, pointcloud_path) :

	# set random seed
	np.random.seed(42)

	torch.set_printoptions(precision=4, sci_mode=False)

	# acquire gym interface
	gym = gymapi.acquire_gym()

	# parse arguments

	# Add custom arguments
	custom_parameters = [
		{"name": "--controller", "type": str, "default": "ik",
		"help": "Controller to use for Franka. Options are {ik, osc}"},
		{"name": "--num_envs", "type": int, "default": 1, "help": "Number of environments to create"},
	]
	args = gymutil.parse_arguments(
		description="Franka Jacobian Inverse Kinematics (IK) + Operational Space Control (OSC) Example",
		custom_parameters=custom_parameters,
	)

	# Grab controller
	controller = args.controller
	assert controller in {"ik", "osc"}, f"Invalid controller specified -- options are (ik, osc). Got: {controller}"

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

	# Set controller parameters
	# IK params
	damping = 0.05

	# OSC params
	kp = 150.
	kd = 2.0 * np.sqrt(kp)
	kp_null = 10.
	kd_null = 2.0 * np.sqrt(kp_null)

	# create sim
	sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
	if sim is None:
		raise Exception("Failed to create sim")

	# create viewer
	viewer = gym.create_viewer(sim, gymapi.CameraProperties())
	if viewer is None:
		raise Exception("Failed to create viewer")


	asset = create_cabinet(gym, sim, cabinet_path)

	dof_dict = gym.get_asset_dof_dict(asset)
	assert(len(dof_dict) == 1)
	cabinet_dof_name = list(dof_dict.keys())[0]

	rig_dict = gym.get_asset_rigid_body_dict(asset)
	assert(len(rig_dict) == 2)
	cabinet_rig_name = list(rig_dict.keys())[1]
	assert(cabinet_rig_name != "base")

	# configure env grid
	num_envs = args.num_envs
	num_per_row = int(math.sqrt(num_envs))
	spacing = 1.0
	env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
	env_upper = gymapi.Vec3(spacing, spacing, spacing)
	print("Creating %d environments" % num_envs)



	envs = []
	cabinet_idxs = []
	hand_idxs1 = []
	init_pos_list1 = []
	init_rot_list1 = []
	hand_idxs2 = []
	init_pos_list2 = []
	init_rot_list2 = []
	# add ground plane
	plane_params = gymapi.PlaneParams()
	plane_params.normal = gymapi.Vec3(0, 0, 1)
	gym.add_ground(sim, plane_params)

	for i in range(num_envs):
		# create env
		env = gym.create_env(sim, env_lower, env_upper, num_per_row)
		envs.append(env)

		cabinet_start_pose = gymapi.Transform()
		up_axis_idx = 2
		cabinet_start_pose.p = gymapi.Vec3(*get_axis_params(1.5, up_axis_idx))
		# cabinet_start_pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(0,0,1.), math.pi)
		cabinet_pose = cabinet_start_pose
		
		# add cabinet
		cabinet_handle = gym.create_actor(env,asset, cabinet_pose, "cabinet", i, 2, 0)
		drawer_handle = gym.find_actor_rigid_body_handle(env, cabinet_handle, "cabinet_rig_name")
		# get global index of cabinet in rigid body state tensor
		cabinet_idx = gym.get_actor_rigid_body_index(env, cabinet_handle, 0, gymapi.DOMAIN_SIM)  # saved by sim(all of the envs)
		cabinet_idxs.append(cabinet_idx)
	
	# point camera at middle env
	cam_pos = gymapi.Vec3(-7, 0, 2)
	cam_target = gymapi.Vec3(20, 0, 0)
	middle_env = envs[num_envs // 2 + num_per_row // 2]
	gym.viewer_camera_look_at(viewer, middle_env, cam_pos, cam_target)

	# ==== prepare tensors =====
	# from now on, we will use the tensor API that can run on CPU or GPU
	gym.prepare_sim(sim)


	data = torch.load(pointcloud_path, map_location=device)
	root_tensor = gymtorch.wrap_tensor(gym.acquire_actor_root_state_tensor(sim)).view(num_envs, -1, 13)
	dof_state_tensor = gymtorch.wrap_tensor(gym.acquire_dof_state_tensor(sim)).view(num_envs, -1, 2)
	rigid_body_tensor = gymtorch.wrap_tensor(gym.acquire_rigid_body_state_tensor(sim)).view(num_envs, -1, 13)

	dof_effort, dof_lower, dof_upper = get_dof_property(gym, asset)
	dof_lower = dof_lower[0]
	dof_upper = dof_upper[0]

	ddof = 0.1
	# drot = torch.tensor([0, 0, 0, 1.0], device=device)

	# simulation loop
	while not gym.query_viewer_has_closed(viewer):

		gym.refresh_actor_root_state_tensor(sim)
		gym.refresh_dof_state_tensor(sim)
		gym.refresh_rigid_body_state_tensor(sim)

		if dof_state_tensor[:, 0, 0] + ddof > dof_upper or dof_state_tensor[:, 0, 0] + ddof < dof_lower :
			ddof *= -1
		dof_state_tensor[:, 0, 0] += ddof
		gym.set_dof_state_tensor(
			sim,
			gymtorch.unwrap_tensor(dof_state_tensor)
		)
		gym.set_actor_root_state_tensor(
			sim,
			gymtorch.unwrap_tensor(root_tensor)
		)

		# step the physics
		gym.simulate(sim)
		gym.fetch_results(sim, True)

		# refresh tensors
		gym.refresh_rigid_body_state_tensor(sim)

		drawer_pos = rigid_body_tensor[0, drawer_handle, 0:3]
		drawer_rot = rigid_body_tensor[0, drawer_handle, 3:7]

		# gym.set_dof_actuation_force_tensor(sim, gymtorch.unwrap_tensor(effort_action))
		# print(gym.get_actor_rigid_body_properties(0, franka_handle1))
		# gym.add_lines(viewer, env, 1, [np.array([0,0,0.6]), np.array([1,0,0.6])], [0,0,1])
		# pot cover coordinate
		gym.clear_lines(viewer)
		for i in range(1000):

			sample = data[random.randint(0, data.shape[0]-1), :3]

			print(drawer_rot.shape, sample.shape)

			pos = drawer_pos + quat_apply(drawer_rot, sample).view(-1)
			d_pos = pos.clone()
			d_pos[1]-=0.01
			gym.add_lines(viewer, env, 1, [np.array(d_pos.cpu()), np.array(pos.cpu())], [0,0,1])
		# update viewer
		gym.step_graphics(sim)
		gym.draw_viewer(viewer, sim, False)
		gym.sync_frame_time(sim)

	# cleanup
	gym.destroy_viewer(viewer)
	gym.destroy_sim(sim)

if __name__ == "__main__":

	show("dataset/one_door_cabinet/35059_link_0/mobility.urdf", "/home/boshi/Documents/Science/MARL-CP/logs/franka_cabinet_PC_partial/ppo_pc_pure/ppo_pc_pure_seed4444/buffer_2800.pt")