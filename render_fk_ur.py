#
# This software is free for non-commercial, research and evaluation use 
# under the terms of the LICENSE.md file.
#
# For inquiries contact  george.drettakis@inria.fr
#

import copy

import torch
from scene import Scene
import os
from tqdm import tqdm
from os import makedirs
from gaussian_renderer import render
import torchvision
from utils.general_utils import safe_state
from argparse import ArgumentParser
from arguments import ModelParams, PipelineParams, get_combined_args
from gaussian_renderer import GaussianModel
from utils_fk import *
from e3nn import o3
from einops import einsum

import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.spatial.transform.rotation import Rotation as R

import einops
import pickle
import numpy as np
import yaml

# ######### Setup the pybullet engine #########
import pybullet as p
p.connect(p.DIRECT)
p.resetSimulation()
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
pandaUid = p.loadURDF('./pybullet-playground_2/urdf/sisbot.urdf', useFixedBase=True, basePosition=[0.0, 0.0, -0.1])
# pandaUid = p.loadURDF('pybullet_ur5_gripper/robots/urdf/ur5e.urdf', useFixedBase=True, basePosition=[0.0, 0.0, 0.0], flags=flags, globalScaling=1)



joint_poses = [0, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, 0.0, 0.7999999999999996, 0.0, -0.8000070728762431, 0.0, 0.7999947291384548, 0.799996381456464, 0.0, -0.799988452159267, 0.0, 0.7999926186486127]  # Example configuration

for i in range(len(joint_poses)):
    p.resetJointState(pandaUid, i, joint_poses[i])
    
initial_joints = []
for joint_index in range(19):
    joint_info = p.getJointInfo(pandaUid, joint_index)
    joint_name = joint_info[1].decode("utf-8")  
    link_state = p.getLinkState(pandaUid, joint_index, computeForwardKinematics=True)
    initial_joints.append(link_state)
    


def get_transfomration_list(new_joint_poses):

    for i in range(len(joint_poses)):
        p.resetJointState(pandaUid, i, new_joint_poses[i])
        
    new_joints = []
    for joint_index in range(19):
        joint_info = p.getJointInfo(pandaUid, joint_index)
        joint_name = joint_info[1].decode("utf-8") 
        link_state = p.getLinkState(pandaUid, joint_index, computeForwardKinematics=True)
        new_joints.append(link_state)
        
        
    transformations_list = []
    for joint_index in range(19):
        input_1 = (initial_joints[joint_index][0][0], initial_joints[joint_index][0][1], initial_joints[joint_index][0][2], np.array(initial_joints[joint_index][1]))
        input_2 = (new_joints[joint_index][0][0], new_joints[joint_index][0][1], new_joints[joint_index][0][2], np.array(new_joints[joint_index][1]))
        r_rel, t = compute_transformation(input_1, input_2)
        r_rel = torch.from_numpy(r_rel).to(device='cuda').float()
        t = torch.from_numpy(t).to(device='cuda').float()
        
        transformations_list.append((r_rel, t))

    return transformations_list



def render_set(model_path, name, iteration, views, gaussians, pipeline, background, object_list, robot_name, object_splat_folder):
    render_path = os.path.join(model_path, name, "ours_{}".format(iteration), "renders")
    gts_path = os.path.join(model_path, name, "ours_{}".format(iteration), "gt")


    makedirs(render_path, exist_ok=True)
    makedirs(gts_path, exist_ok=True)  
    
    gaussians_backup = copy.deepcopy(gaussians)

    object_gaussians = [GaussianModel(3) for _ in range(len(object_list))]

    for _ in range(len(object_list)):
        object_gaussians[_].load_ply(object_splat_folder + "{}/point_cloud/iteration_7000/point_cloud.ply".format(object_list[_]))



    object_gaussians_backup = copy.deepcopy(object_gaussians)

    # load object configs 
    with open('object_configs/objects.yaml', 'r') as file:
        object_config = yaml.safe_load(file)
        robot_transformation = object_config[robot_name]['transformation']['matrix']


    for idx, view in enumerate(tqdm(views, desc="Rendering progress")):
        print('idx : ', idx)
        if idx == 255:
            break

        if idx not in [5, 254]:
            continue
        

        traj_folder = args.traj_folder
        # traj folder contains folders containing the trajectory data
        for j, folder in enumerate(sorted(os.listdir(traj_folder))):
            # if j>0:
            #     break
            if idx == 254:
                image_folder = os.path.join(traj_folder, folder, 'images_2')
            if idx == 5:
                image_folder = os.path.join(traj_folder, folder, 'images_1')

            #print all the variables in view object
            # for key in view.__dict__.keys():
            #     print(key, view.__dict__[key])
            makedirs(image_folder, exist_ok=True)
            print('image_folder : ', image_folder)

            # take only those filenames which end with .pkl
            file_names = os.listdir(os.path.join(traj_folder, folder))
            file_names.sort()

            # print("filenames", file_names)
            
            iterator = tqdm(range(0, len(file_names)), desc="Rendering progress")
            images = []
            for k in iterator:
                #read the pkl file
                file = file_names[k]
                if not file.endswith('.pkl'):
                    continue
                file_path = os.path.join(traj_folder, folder, file)
                file = open(file_path, 'rb')
                data = pickle.load(file)

                cur_joint = data['joint_positions_dummy'][:]
                #add 0 at front of cur_joint
                cur_joint = [0] + cur_joint.tolist()
                # cur_joint = [0, 0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
                cur_joint = np.array(cur_joint)

                cur_object_position_list = []
                cur_object_rotation_list = []

                for object_name in object_list:
                    cur_object_position = np.array(data[object_name + '_position'])
                    cur_object_position_list.append(torch.from_numpy(cur_object_position).to(device='cuda').float())
                    cur_object_rotation = np.array(data[object_name + '_orientation'])
                    cur_object_rotation = np.roll(cur_object_rotation, 1)
                    cur_object_rotation_list.append(torch.from_numpy(cur_object_rotation).to(device='cuda').float())

                
                
                transformations_list = get_transfomration_list(cur_joint)
                segmented_list, xyz = get_segmented_indices(gaussians_backup, robot_transformation, object_config[robot_name]['aabb']['bounding_box'], robot_name)

                xyz, rot, opacity, shs_featrest, shs_dc  = transform_means(gaussians_backup, xyz, segmented_list, transformations_list, robot_transformation)
                xyz_obj_list = []
                rot_obj_list = []
                opacity_obj_list = []
                scales_obj_list = []
                features_dc_obj_list = []
                features_rest_obj_list = []


                for i in range(len(object_list)):
                    xyz_obj, rot_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj = transform_object(object_gaussians_backup[i], object_config=object_config[object_list[i]], pos=cur_object_position_list[i], quat=cur_object_rotation_list[i], robot_transformation=robot_transformation)
                    xyz_obj_list.append(xyz_obj)
                    rot_obj_list.append(rot_obj)
                    opacity_obj_list.append(opacity_obj)
                    scales_obj_list.append(scales_obj)
                    features_dc_obj_list.append(features_dc_obj)
                    features_rest_obj_list.append(features_rest_obj)
                
                with torch.no_grad():
                    # gaussians.active_sh_degree = 0
                    gaussians._xyz = torch.cat([xyz] + xyz_obj_list, dim=0)
                    gaussians._rotation = torch.cat([rot] + rot_obj_list, dim=0)
                    gaussians._opacity = torch.cat([opacity] + opacity_obj_list, dim=0)
                    gaussians._features_rest = torch.cat([shs_featrest] + features_rest_obj_list, dim=0)
                    gaussians._features_dc = torch.cat([shs_dc] +features_dc_obj_list, dim=0)
                    gaussians._scaling = torch.cat([gaussians_backup._scaling] +scales_obj_list, dim=0)
                
                rendering = render(view, gaussians, pipeline, background)["render"]

                #add gaussian noise to the rendering
                # noise = torch.randn_like(rendering) * torch.rand(1).item() * 0.3
                # rendering = rendering + noise

                torchvision.utils.save_image(rendering, os.path.join(image_folder, '{0:05d}'.format(k)+".png"))
                object_gaussians = copy.deepcopy(object_gaussians_backup)

            
            #save renderings 
            

            print("Done with folder: ", folder)
            
        # if idx>-1:
        #     break

def render_sets(dataset : ModelParams, iteration : int, pipeline : PipelineParams, skip_train : bool, skip_test : bool, object_list : list, robot_name : str, object_splat_folder : str):
    with torch.no_grad():
        gaussians = GaussianModel(dataset.sh_degree)
                
        scene = Scene(dataset, gaussians, load_iteration=iteration, shuffle=False)

        bg_color = [1,1,1] if dataset.white_background else [0, 0, 0]
        background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")
        
        if not skip_train:
             render_set(dataset.model_path, "train", scene.loaded_iter, scene.getTrainCameras(), gaussians, pipeline, background,  object_list=object_list, robot_name=robot_name, object_splat_folder=object_splat_folder)

        if not skip_test:
            render_set(dataset.model_path, "test", scene.loaded_iter, scene.getTestCameras(), gaussians, pipeline, background, object_list=object_list, robot_name=robot_name, object_splat_folder=object_splat_folder)



def transform_means(pc, xyz, segmented_list, transformations_list, robot_transformation):

    Trans = torch.tensor(robot_transformation).to(device=xyz.device).float()

    
    scale_robot = torch.pow(torch.linalg.det(Trans[:3, :3]), 1/3)
    rotation_matrix = Trans[:3, :3] / scale_robot
    translation = Trans[:3, 3]
    inv_transformation_matrix = torch.inverse(Trans)
    inv_rotation_matrix = inv_transformation_matrix[:3, :3] 
    inv_translation = inv_transformation_matrix[:3, 3]
    
    rot = pc.get_rotation
    opacity = pc.get_opacity_raw
    with torch.no_grad():
        shs_dc = copy.deepcopy(pc._features_dc)
        shs_featrest = copy.deepcopy(pc._features_rest)

    for joint_index in range(19):
        r_rel, t = transformations_list[joint_index]
        segment = segmented_list[joint_index]
        transformed_segment = torch.matmul(r_rel, xyz[segment].T).T + t
        xyz[segment] = transformed_segment
        
        # Defining rotation matrix for the covariance
        rot_rotation_matrix = (inv_rotation_matrix*scale_robot) @ r_rel @ rotation_matrix
        
        tranformed_rot = rot[segment]  
        tranformed_rot = o3.quaternion_to_matrix(tranformed_rot) ### --> zyx    
        
        transformed_rot = rot_rotation_matrix  @ tranformed_rot # shape (N, 3, 3)
        
        transformed_rot = o3.matrix_to_quaternion(transformed_rot)
        
        rot[segment] = transformed_rot

        #transform the shs features
        shs_feat = shs_featrest[segment]
        shs_dc_segment = shs_dc[segment]
        shs_feat = transform_shs(shs_feat, rot_rotation_matrix)
        # print('shs_feat : ', shs_feat.shape)
        with torch.no_grad():
            shs_featrest[segment] = shs_feat
        # shs_dc[segment] = shs_dc_segment
        # shs_featrest[segment] = torch.zeros_like(shs_featrest[segment])
           
    #transform_back
    xyz = torch.matmul(inv_rotation_matrix, xyz.T).T + inv_translation
    
        
    return xyz, rot, opacity, shs_featrest, shs_dc


def transform_object(pc, object_config, pos, quat, robot_transformation):
            
    
    Trans_canonical = torch.from_numpy(np.array(object_config['transformation']['matrix'])).to(device=pc.get_xyz.device).float() # shape (4, 4)

    
    
    rotation_matrix_c = Trans_canonical[:3, :3]
    translation_c = Trans_canonical[:3, 3]
    scale_obj = torch.pow(torch.linalg.det(rotation_matrix_c), 1/3)

    
    Trans_robot = torch.tensor(robot_transformation).to(device=pc.get_xyz.device).float()

    
    rotation_matrix_r = Trans_robot[:3, :3]
    scale_r = torch.pow(torch.linalg.det(rotation_matrix_r), 1/3)

    translation_r = Trans_robot[:3, 3]

    inv_transformation_r = torch.inverse(Trans_robot)
    inv_rotation_matrix_r = inv_transformation_r[:3, :3]
    inv_translation_r = inv_transformation_r[:3, 3]
    inv_scale = torch.pow(torch.linalg.det(inv_rotation_matrix_r), 1/3)

    # print('scale_obj : ', scale_obj)
    # print('inv_scale : ', inv_scale)
    
    xyz_obj = pc.get_xyz
    rotation_obj = pc.get_rotation
    opacity_obj = pc.get_opacity_raw
    scales_obj = pc.get_scaling
    scales_obj = scales_obj * scale_obj * inv_scale 
    scales_obj = torch.log(scales_obj)

    with torch.no_grad():
        features_dc_obj = copy.deepcopy(pc._features_dc)
        features_rest_obj = copy.deepcopy(pc._features_rest)
    
    #transform the object to the canonical frame
    xyz_obj = torch.matmul(rotation_matrix_c, xyz_obj.T).T + translation_c
    
    
    rot_rotation_matrix = ( inv_rotation_matrix_r/inv_scale) @ o3.quaternion_to_matrix(quat)  @  (rotation_matrix_c/scale_obj)
    rotation_obj_matrix = o3.quaternion_to_matrix(rotation_obj)
    rotation_obj_matrix = rot_rotation_matrix @ rotation_obj_matrix 
    rotation_obj = o3.matrix_to_quaternion(rotation_obj_matrix) 
    
    
    aabb = object_config['aabb']['bounding_box']
    #segment according to axis aligned bounding box
    segmented_indices = ((xyz_obj[:, 0] > aabb[0][0]) & (xyz_obj[:, 0] < aabb[1][0]) & (xyz_obj[:, 1] > aabb[0][1] ) & (xyz_obj[:, 1] < aabb[1][1]) & (xyz_obj[:, 2] > aabb[0][2] ) & (xyz_obj[:, 2] < aabb[1][2]))
    

    #offset the object by the position and rotation
    xyz_obj = torch.matmul(o3.quaternion_to_matrix(quat), xyz_obj.T).T + pos
    # xyz_obj = xyz_obj + pos
    
    xyz_obj = torch.matmul(inv_rotation_matrix_r, xyz_obj.T).T + inv_translation_r

    xyz_obj = xyz_obj[segmented_indices]
    rotation_obj = rotation_obj[segmented_indices]
    opacity_obj = opacity_obj[segmented_indices]
    scales_obj = scales_obj[segmented_indices]
    # cov3D_obj = cov3D_obj[segmented_indices]
    features_dc_obj = features_dc_obj[segmented_indices]
    features_rest_obj = features_rest_obj[segmented_indices]
    features_rest_obj= transform_shs( features_rest_obj, rot_rotation_matrix)
    # features_rest_obj = torch.zeros_like(features_rest_obj)
    
    return xyz_obj, rotation_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj


def transform_shs(shs_feat, rotation_matrix):

    ## rotate shs
    P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]) # switch axes: yzx -> xyz
    permuted_rotation_matrix = np.linalg.inv(P) @ rotation_matrix.cpu().numpy() @ P
    rot_angles = o3._rotation.matrix_to_angles(torch.from_numpy(permuted_rotation_matrix).to(device=shs_feat.device).float())
    
    # Construction coefficient
    D_1 = o3.wigner_D(1, rot_angles[0].cpu(), - rot_angles[1].cpu(), rot_angles[2].cpu()).to(device=shs_feat.device)
    D_2 = o3.wigner_D(2, rot_angles[0].cpu(), - rot_angles[1].cpu(), rot_angles[2].cpu()).to(device=shs_feat.device)
    D_3 = o3.wigner_D(3, rot_angles[0].cpu(), - rot_angles[1].cpu(), rot_angles[2].cpu()).to(device=shs_feat.device)

    #rotation of the shs features
    one_degree_shs = shs_feat[:, 0:3]
    one_degree_shs = einops.rearrange(one_degree_shs, 'n shs_num rgb -> n rgb shs_num')
    one_degree_shs = einsum(
            D_1,
            one_degree_shs,
            "... i j, ... j -> ... i",
        )
    one_degree_shs = einops.rearrange(one_degree_shs, 'n rgb shs_num -> n shs_num rgb')
    shs_feat[:, 0:3] = one_degree_shs

    two_degree_shs = shs_feat[:, 3:8]
    two_degree_shs = einops.rearrange(two_degree_shs, 'n shs_num rgb -> n rgb shs_num')
    two_degree_shs = einsum(
            D_2,
            two_degree_shs,
            "... i j, ... j -> ... i",
        )
    two_degree_shs = einops.rearrange(two_degree_shs, 'n rgb shs_num -> n shs_num rgb')
    shs_feat[:, 3:8] = two_degree_shs

    three_degree_shs = shs_feat[:, 8:15]
    three_degree_shs = einops.rearrange(three_degree_shs, 'n shs_num rgb -> n rgb shs_num')
    three_degree_shs = einsum(
            D_3,
            three_degree_shs,
            "... i j, ... j -> ... i",
        )
    three_degree_shs = einops.rearrange(three_degree_shs, 'n rgb shs_num -> n shs_num rgb')
    shs_feat[:, 8:15] = three_degree_shs

    return shs_feat


def get_segmented_indices(pc, robot_transformation, aabb, robot_name):
    # empty torch cache
    torch.cuda.empty_cache()
    means3D = pc.get_xyz # 3D means shape (N, 3)
    
    # Defining a cube in Gaussian space to segment out the robot
    xyz = pc.get_xyz # shape (N, 3)

    Trans = torch.tensor(robot_transformation).to(device=means3D.device).float() # shape (4, 4)
    

    R = Trans[:3, :3]
    translation = Trans[:3, 3]
    
    
    points = copy.deepcopy(means3D)
    #transform the points to the new frame
    points = torch.matmul(R, points.T).T + translation
    
    segmented_points = []

    #load labels.npy
    labels = np.load('./labels_path/'+robot_name+'_labels.npy')
    labels = torch.from_numpy(labels).to(device=xyz.device).long()

    
    condition = (points[:, 0] > aabb[0][0]) & (points[:, 0] < aabb[1][0]) & (points[:, 1] > aabb[0][1]) & (points[:, 1] < aabb[1][1]) & (points[:, 2] > aabb[0][2]) & (points[:, 2] < aabb[1][2])
    condition = torch.where(condition)[0]
    
    for i in range(19):
        segmented_points.append(condition[labels==i])
    
    return segmented_points, points



if __name__ == "__main__":
    # Set up command line argument parser
    parser = ArgumentParser(description="Testing script parameters")
    model = ModelParams(parser, sentinel=True)
    pipeline = PipelineParams(parser)
    parser.add_argument("--iteration", default=-1, type=int)
    parser.add_argument("--skip_train", action="store_true")
    parser.add_argument("--skip_test", action="store_true")
    parser.add_argument("--objects", default='plastic_apple', type=str)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--traj_folder", default='/home/nomaan/bc_data/gello/', type=str,
                      help="Path to the trajectory folder containing demonstration data")
    args = get_combined_args(parser)


    #get the robot name from model path 
    robot_name = args.model_path.split('/')[-1]
    
    #get the object output folder
    object_splat_folder = args.model_path.replace(robot_name, '')

    
    if args.objects == " ":
        object_list = []
    else:
        object_list = args.objects.split(' ')

    print("Rendering " + args.model_path)

    # Initialize system state (RNG)
    safe_state(args.quiet)

    render_sets(model.extract(args), args.iteration, pipeline.extract(args), args.skip_train, args.skip_test, object_list=object_list, robot_name=robot_name, object_splat_folder=object_splat_folder)