import copy

import torch
from gaussian_splatting.scene import Scene
import os
from tqdm import tqdm
from os import makedirs
from gaussian_renderer import render
import torchvision
from gaussian_splatting.utils.general_utils import safe_state
from argparse import ArgumentParser
from arguments import ModelParams, PipelineParams, get_combined_args
from gaussian_renderer import GaussianModel
from splatsim.utils.utils_fk import *
from e3nn import o3
from einops import einsum

import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.spatial.transform.rotation import Rotation as R

import einops
import pickle
import numpy as np
import yaml
import math

import pybullet as p

def get_curr_link_states(robot_uid, use_link_centers=True):
    link_states = []
    num_joints = p.getNumJoints(robot_uid)

    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(robot_uid, joint_index)
        joint_name = joint_info[1].decode("utf-8") 

        if use_link_centers:
            link_state = p.getLinkState(robot_uid, joint_index, computeForwardKinematics=True)
            link_states.append({
                "pos": link_state[0],
                "q": link_state[1]
            })
        else:
            link_state = p.getLinkState(robot_uid, joint_index, computeForwardKinematics=True)
            link_states.append({
                "pos": link_state[4],
                "q": link_state[5]
            })
    
    return link_states


def get_transfomration_list(robot_uid, initial_link_states, use_link_centers=True):
    num_joints = p.getNumJoints(robot_uid)

    new_joints = get_curr_link_states(robot_uid, use_link_centers=use_link_centers)

    if len(initial_link_states) != num_joints or len(new_joints) != num_joints:
        print(f"Error: Number of joints mismatch. Initial: {len(initial_link_states)}, New: {len(new_joints)}, Expected: {num_joints}")

        
    transformations_list = []
    for joint_index in range(num_joints):
        # x, y, z, q
        input_1 = (initial_link_states[joint_index]["pos"][0], initial_link_states[joint_index]["pos"][1], initial_link_states[joint_index]["pos"][2], np.array(initial_link_states[joint_index]["q"]))
        input_2 = (new_joints[joint_index]["pos"][0], new_joints[joint_index]["pos"][1], new_joints[joint_index]["pos"][2], np.array(new_joints[joint_index]["q"]))
        r_rel, t = compute_transformation(input_1, input_2)
        r_rel = torch.from_numpy(r_rel).to(device='cuda').float()
        t = torch.from_numpy(t).to(device='cuda').float()
        
        transformations_list.append((r_rel, t))

    return transformations_list

def transform_means(robot_uid, pc, xyz, segmented_list, transformations_list, robot_transformation):
    # xyz is in global frame. pc is in splat frame

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

    for joint_index in range(p.getNumJoints(robot_uid)):
        r_rel, t = transformations_list[joint_index] # T between initial link and current link

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
           
    #transform_back to splat frame
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


def get_segmented_indices(robot_uid, pc, robot_transformation, aabb, robot_name):
    # empty torch cache
    torch.cuda.empty_cache()
    # Defining a cube in Gaussian space to segment out the robot
    means3D = pc.get_xyz # 3D means shape (N, 3)

    Trans = torch.tensor(robot_transformation).to(device=means3D.device).float() # shape (4, 4)

    R = Trans[:3, :3]
    translation = Trans[:3, 3]
    
    points = copy.deepcopy(means3D)
    #transform the points to the new frame
    points = torch.matmul(R, points.T).T + translation
    
    segmented_points = []

    #load labels.npy
    labels = np.load('./data/labels_path/'+robot_name+'_labels.npy')
    labels = torch.from_numpy(labels).to(device=means3D.device).long()

    condition = (points[:, 0] > aabb[0][0]) & (points[:, 0] < aabb[1][0]) & (points[:, 1] > aabb[0][1]) & (points[:, 1] < aabb[1][1]) & (points[:, 2] > aabb[0][2]) & (points[:, 2] < aabb[1][2])
    condition = torch.where(condition)[0]
    for i in range(p.getNumJoints(robot_uid)):
        segmented_points.append(condition[labels==i])
    
    return segmented_points, points
