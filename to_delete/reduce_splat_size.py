#
# Copyright (C) 2023, Inria
# GRAPHDECO research group, https://team.inria.fr/graphdeco
# All rights reserved.
#
# This software is free for non-commercial, research and evaluation use 
# under the terms of the LICENSE.md file.
#
# For inquiries contact  george.drettakis@inria.fr
#

import copy

import torch
from gaussian_splatting.scene import Scene
import os
from tqdm import tqdm
from os import makedirs
from gaussian_renderer import render
import torchvision
from utils.general_utils import safe_state
from argparse import ArgumentParser
from arguments import ModelParams, PipelineParams, get_combined_args
from gaussian_renderer import GaussianModel
from splatsim.utils.utils_fk import *
import sphecerix
from e3nn import o3
from einops import einsum
from e3nn.o3 import matrix_to_angles, wigner_D

from pathlib import Path
import matplotlib.pyplot as plt
from e3nn.o3 import spherical_harmonics
from matplotlib import cm
from scipy.spatial.transform.rotation import Rotation as R
import math

import einops
import yaml
import numpy as np

def transform_and_segment_object(pc, object_config):
    
    Trans_canonical = torch.from_numpy(np.array(object_config['transformation']['matrix'])).to(device=pc.get_xyz.device).float() # shape (4, 4)

    
    
    rotation_matrix_c = Trans_canonical[:3, :3]
    translation_c = Trans_canonical[:3, 3]
    scale_obj = torch.pow(torch.linalg.det(rotation_matrix_c), 1/3)

    inv_rotation_matrix_r = torch.inverse(Trans_canonical)[:3, :3]
    inv_translation_r = torch.inverse(Trans_canonical)[:3, 3]

    # print('scale_obj : ', scale_obj)
    # print('inv_scale : ', inv_scale)
    
    xyz_obj = pc.get_xyz
    rotation_obj = pc.get_rotation
    opacity_obj = pc.get_opacity_raw
    scales_obj = pc.get_scaling
    # scales_obj = scales_obj * scale_obj * inv_scale 
    scales_obj = scales_obj * 1

    scales_obj = torch.log(scales_obj)

    with torch.no_grad():
        features_dc_obj = copy.deepcopy(pc._features_dc)
        features_rest_obj = copy.deepcopy(pc._features_rest)
    
    #transform the object to the canonical frame
    xyz_obj = torch.matmul(rotation_matrix_c, xyz_obj.T).T + translation_c
    

    
    # aabb = ((-0.10300000149011612, -0.17799999701976776, -0.0030000000000000027), (0.10300000149011612, 0.028000000372529033, 0.022999999552965167))
    aabb = object_config['aabb']['bounding_box']
    #segment according to axis aligned bounding box
    segmented_indices = ((xyz_obj[:, 0] > aabb[0][0]) & (xyz_obj[:, 0] < aabb[1][0]) & (xyz_obj[:, 1] > aabb[0][1] ) & (xyz_obj[:, 1] < aabb[1][1]) & (xyz_obj[:, 2] > aabb[0][2] ) & (xyz_obj[:, 2] < aabb[1][2]))
    
    xyz_obj = torch.matmul(inv_rotation_matrix_r, xyz_obj.T).T + inv_translation_r

    xyz_obj = xyz_obj[segmented_indices]
    rotation_obj = rotation_obj[segmented_indices]
    opacity_obj = opacity_obj[segmented_indices]
    scales_obj = scales_obj[segmented_indices]
    # cov3D_obj = cov3D_obj[segmented_indices]
    features_dc_obj = features_dc_obj[segmented_indices]
    features_rest_obj = features_rest_obj[segmented_indices]
    # features_rest_obj = torch.zeros_like(features_rest_obj)
    
    return xyz_obj, rotation_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj


if __name__ == "__main__":
    # Set up command line argument parser
    
    object_name = 'T_object'

    # Load model
    model = GaussianModel(3)
    model.load_ply('output/{}/point_cloud/iteration_7000/point_cloud.ply'.format(object_name))

    # Load object config file
    with open('configs/object_configs/objects.yaml', 'r') as file:
        object_config = yaml.safe_load(file)

    # Transform object
    xyz_obj, rotation_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj = transform_and_segment_object(model, object_config[object_name])

    with torch.no_grad():
        model._xyz = xyz_obj
        model._rotation = rotation_obj
        model._opacity = opacity_obj
        model._scaling = scales_obj
        model._features_dc = features_dc_obj
        model._features_rest = features_rest_obj
    



    #save the gaussian model as a ply file
    model.save_ply('output/{}/point_cloud/iteration_7000/point_cloud.ply'.format(object_name))


    