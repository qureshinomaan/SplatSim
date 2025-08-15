import copy

import torch
from e3nn import o3
import einops
import numpy as np
import pybullet as p

from splatsim.utils.utils_fk import *

def get_curr_link_states(robot_uid, use_link_centers=True):
    link_states = []
    num_joints = p.getNumJoints(robot_uid)

    for joint_index in range(num_joints):
        link_state = p.getLinkState(robot_uid, joint_index, computeForwardKinematics=True)
        if use_link_centers:
            link_states.append({
                "pos": link_state[0],
                "q": link_state[1]
            })
        else:
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

def transform_means(robot_uid, pc, xyz, segmented_list, transformations_list, robot_transformation, robot_name, transformations_cache=None):
    # xyz is in global frame. pc is in splat frame

    if transformations_cache is None:
        Trans = torch.tensor(robot_transformation).to(device=xyz.device).float()
        scale_robot = torch.pow(torch.linalg.det(Trans[:3, :3]), 1/3)
        inv_transformation_matrix = torch.inverse(Trans)
    else:
        Trans = transformations_cache[robot_name]["transformation"]
        scale_robot = transformations_cache[robot_name]["transformation_scale"]
        inv_transformation_matrix = transformations_cache[robot_name]["inv_transformation"]
    rotation_matrix = Trans[:3, :3] / scale_robot
    inv_rotation_matrix = inv_transformation_matrix[:3, :3] 
    inv_translation = inv_transformation_matrix[:3, 3]
    
    rot = pc.get_rotation
    opacity = pc.get_opacity_raw
    with torch.no_grad():
        shs_dc = pc._features_dc.clone()
        shs_featrest = pc._features_rest.clone()

    for joint_index in range(p.getNumJoints(robot_uid)):
        r_rel, t = transformations_list[joint_index] # T between initial link and current link
        segment = segmented_list[joint_index]

        transformed_segment = torch.matmul(r_rel, xyz[segment].T).T + t
        xyz[segment] = transformed_segment
        
        # Defining rotation matrix for the covariance
        rot_rotation_matrix = (inv_rotation_matrix*scale_robot) @ r_rel @ rotation_matrix
        
        tranformed_rot = rot[segment]  
        tranformed_rot = o3.quaternion_to_matrix(tranformed_rot) ### --> zyx    
        transformed_rot = rot_rotation_matrix @ tranformed_rot # shape (N, 3, 3)
        transformed_rot = o3.matrix_to_quaternion(transformed_rot)
        rot[segment] = transformed_rot

        #transform the shs features
        shs_feat = shs_featrest[segment]
        shs_feat = transform_shs(shs_feat, rot_rotation_matrix)
        with torch.no_grad():
            shs_featrest[segment] = shs_feat
           
    #transform_back to splat frame
    xyz = torch.matmul(inv_rotation_matrix, xyz.T).T + inv_translation

    return xyz, rot, opacity, shs_featrest, shs_dc


def transform_object(pc, object_config, pos, quat, robot_transformation, object_name, robot_name, transformations_cache=None):
    if transformations_cache is None:
        Trans_canonical = torch.from_numpy(np.array(object_config['transformation']['matrix'])).to(device=pc.get_xyz.device).float() # shape (4, 4)
        scale_obj = torch.pow(torch.linalg.det(Trans_canonical[:3, :3]), 1/3)
        
        Trans_robot = torch.tensor(robot_transformation).to(device=pc.get_xyz.device).float()
        inv_transformation_r = torch.inverse(Trans_robot)
        inv_scale = torch.pow(torch.linalg.det(inv_transformation_r[:3, :3]), 1/3)
    else:
        Trans_canonical = transformations_cache[object_name]["transformation"]
        scale_obj = transformations_cache[object_name]["transformation_scale"]

        Trans_robot = transformations_cache[robot_name]["transformation"]
        inv_transformation_r = transformations_cache[robot_name]["inv_transformation"]
        inv_scale = transformations_cache[robot_name]["inv_transformation_scale"]
    rotation_matrix_c = Trans_canonical[:3, :3]
    translation_c = Trans_canonical[:3, 3]
    inv_rotation_matrix_r = inv_transformation_r[:3, :3]
    inv_translation_r = inv_transformation_r[:3, 3]

    xyz_obj = pc.get_xyz
    rotation_obj = pc.get_rotation
    opacity_obj = pc.get_opacity_raw
    scales_obj = pc.get_scaling
    scales_obj = scales_obj * scale_obj * inv_scale 
    scales_obj = torch.log(scales_obj)

    with torch.no_grad():
        features_dc_obj = pc._features_dc.clone()
        features_rest_obj = pc._features_rest.clone()
    
    #transform the object to the canonical frame
    xyz_obj = torch.matmul(rotation_matrix_c, xyz_obj.T).T + translation_c
    
    rot_rotation_matrix = (inv_rotation_matrix_r/inv_scale) @ o3.quaternion_to_matrix(quat)  @  (rotation_matrix_c/scale_obj)
    rotation_obj_matrix = o3.quaternion_to_matrix(rotation_obj)
    rotation_obj_matrix = rot_rotation_matrix @ rotation_obj_matrix 
    rotation_obj = o3.matrix_to_quaternion(rotation_obj_matrix) 
    
    aabb = object_config['aabb']['bounding_box']
    #segment according to axis aligned bounding box
    segmented_indices = ((xyz_obj[:, 0] > aabb[0][0]) & (xyz_obj[:, 0] < aabb[1][0]) & (xyz_obj[:, 1] > aabb[0][1] ) & (xyz_obj[:, 1] < aabb[1][1]) & (xyz_obj[:, 2] > aabb[0][2] ) & (xyz_obj[:, 2] < aabb[1][2]))
    
    #offset the object by the position and rotation
    xyz_obj = torch.matmul(o3.quaternion_to_matrix(quat), xyz_obj.T).T + pos
    
    xyz_obj = torch.matmul(inv_rotation_matrix_r, xyz_obj.T).T + inv_translation_r

    xyz_obj = xyz_obj[segmented_indices]
    rotation_obj = rotation_obj[segmented_indices]
    opacity_obj = opacity_obj[segmented_indices]
    scales_obj = scales_obj[segmented_indices]
    features_dc_obj = features_dc_obj[segmented_indices]
    features_rest_obj = features_rest_obj[segmented_indices]
    features_rest_obj= transform_shs(features_rest_obj, rot_rotation_matrix)
    
    return xyz_obj, rotation_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj


def transform_shs(shs_feat, rotation_matrix):

    ## rotate shs
    P = torch.tensor([[0, 0, 1], [1, 0, 0], [0, 1, 0]], device=rotation_matrix.device).float() # switch axes: yzx -> xyz
    permuted_rotation_matrix = torch.linalg.inv(P) @ rotation_matrix @ P
    rot_angles = o3._rotation.matrix_to_angles(permuted_rotation_matrix)
    rot_angles = (rot_angles[0].cpu(), rot_angles[1].cpu(), rot_angles[2].cpu())
    
    # Construction coefficient
    D_1 = o3.wigner_D(1, rot_angles[0], - rot_angles[1], rot_angles[2]).to(device=shs_feat.device)
    D_2 = o3.wigner_D(2, rot_angles[0], - rot_angles[1], rot_angles[2]).to(device=shs_feat.device)
    D_3 = o3.wigner_D(3, rot_angles[0], - rot_angles[1], rot_angles[2]).to(device=shs_feat.device)

    # shs_feat: (..., 15, 3)   # [SH-index, RGB]
    # D_1: (..., 3, 3) or (3,3)
    # D_2: (..., 5, 5) or (5,5)
    # D_3: (..., 7, 7) or (7,7)

    device = shs_feat.device
    dtype  = shs_feat.dtype

    # Build block-diagonal Wigner-D. If D_1/2/3 are batched (have leading ...),
    # make a batched block-diagonal; otherwise a single 15x15 is fine.

    if D_1.dim() == 2:  # unbatched (3,3),(5,5),(7,7)
        D = torch.block_diag(D_1.to(device=device, dtype=dtype),
                            D_2.to(device=device, dtype=dtype),
                            D_3.to(device=device, dtype=dtype))                    # (15,15)
    else:
        # batched: D_1: (...,3,3), etc. Build (...,15,15)
        *batch, _, _ = D_1.shape
        D = torch.zeros(*batch, 15, 15, device=device, dtype=dtype)
        D[...,  0: 3,  0: 3] = D_1
        D[...,  3: 8,  3: 8] = D_2
        D[...,  8:15,  8:15] = D_3

    # take l=1..3 bands and move RGB before SH for a single einsum
    sh = einops.rearrange(shs_feat[..., :15, :], '... s r -> ... r s')  # (..., 3, 15)

    # one einsum: (...,i,j) @ (...,r,j) -> (...,r,i)
    sh_rot = torch.einsum('...ij, ...rj -> ...ri', D, sh)               # (..., 3, 15)

    # restore layout and write back
    shs_feat[..., :15, :] = einops.rearrange(sh_rot, '... r i -> ... i r')  # (..., 15, 3)

    return shs_feat


def get_segmented_indices(robot_uid, pc, robot_transformation, aabb, robot_name, robot_labels=None, transformations_cache=None):
    # Defining a cube in Gaussian space to segment out the robot
    means3D = pc.get_xyz # 3D means shape (N, 3)

    if transformations_cache is None:
        Trans = torch.tensor(robot_transformation).to(device=means3D.device).float() # shape (4, 4)
    else:
        Trans = transformations_cache[robot_name]["transformation"]
    R = Trans[:3, :3]
    translation = Trans[:3, 3]
    
    xyz = copy.deepcopy(means3D)
    #transform the points to the new frame
    xyz = torch.matmul(R, xyz.T).T + translation
    
    segmented_points = []

    if robot_labels is None:
        # For best speed, load labels outside of this function so that it's not loaded every loop
        #load labels.npy
        robot_labels = np.load('./data/labels_path/'+robot_name+'_labels.npy')
        robot_labels = torch.from_numpy(robot_labels).to(device=means3D.device).long()

    condition = (xyz[:, 0] > aabb[0][0]) & (xyz[:, 0] < aabb[1][0]) & (xyz[:, 1] > aabb[0][1]) & (xyz[:, 1] < aabb[1][1]) & (xyz[:, 2] > aabb[0][2]) & (xyz[:, 2] < aabb[1][2])
    condition = torch.where(condition)[0]
    for i in range(p.getNumJoints(robot_uid)):
        segmented_points.append(condition[robot_labels==i])
    
    return segmented_points, xyz
