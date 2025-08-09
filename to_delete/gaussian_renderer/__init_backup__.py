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

import torch
import math
from diff_gaussian_rasterization import GaussianRasterizationSettings, GaussianRasterizer
from gaussian_splatting.scene.gaussian_model import GaussianModel
from utils.sh_utils import eval_sh, RGB2SH
import open3d as o3d
import numpy as np

def render(viewpoint_camera, pc : GaussianModel, pipe, bg_color : torch.Tensor, scaling_modifier = 1.0, override_color = None):
    """
    Render the scene. 
    
    Background tensor (bg_color) must be on GPU!
    """
 
    # Create zero tensor. We will use it to make pytorch return gradients of the 2D (screen-space) means
    screenspace_points = torch.zeros_like(pc.get_xyz, dtype=pc.get_xyz.dtype, requires_grad=True, device="cuda") + 0
    try:
        screenspace_points.retain_grad()
    except:
        pass

    # Set up rasterization configuration
    tanfovx = math.tan(viewpoint_camera.FoVx * 0.5)
    tanfovy = math.tan(viewpoint_camera.FoVy * 0.5)

    raster_settings = GaussianRasterizationSettings(
        image_height=int(viewpoint_camera.image_height),
        image_width=int(viewpoint_camera.image_width),
        tanfovx=tanfovx,
        tanfovy=tanfovy,
        bg=bg_color,
        scale_modifier=scaling_modifier,
        viewmatrix=viewpoint_camera.world_view_transform,
        projmatrix=viewpoint_camera.full_proj_transform,
        sh_degree=pc.active_sh_degree,
        campos=viewpoint_camera.camera_center,
        prefiltered=False,
        debug=pipe.debug
    )

    rasterizer = GaussianRasterizer(raster_settings=raster_settings)
    # empty torch cache
    torch.cuda.empty_cache()
    means3D = pc.get_xyz # 3D means shape (N, 3)
    print("original means3D", means3D.shape)
    means2D = screenspace_points
    opacity = pc.get_opacity # shape (N, 1)
    
    # Defining a cube in Gaussian space to segment out the robot
    xyz = pc.get_xyz # shape (N, 3)
    N = xyz.shape[0]
    segment_robot = True
    if segment_robot:
        #find indices where x is between -2 and 2, y is between -2 and 2, z is between -2 and 2
        x_indices = ((xyz[:, 0] > -2) & (xyz[:, 0] < 20) & (xyz[:, 1] > -1.3) & (xyz[:, 1] < 1.3) & (xyz[:, 2] > -2.4) & (xyz[:, 2] < 2.4))
        
        # Trans = torch.tensor([[-0.272776812315, -0.074060574174, 0.179010376334, -0.366493761539],
        #                         [-0.193020626903, 0.130256026983, -0.240235880017, 0.399390488863],
        #                         [-0.016514312476, -0.299140989780, -0.148925781250, 0.708794176579],
        #                         [0, 0, 0, 1]]).to(device=means3D.device) # shape (4, 4)
      
        # R = Trans[:3, :3]
        # translation = Trans[:3, 3]
        
      
        # points = xyz
        # #transform the points to the new frame
        # points = torch.matmul(R, points.T).T + translation
        
        
        # centers = torch.tensor([[0, 0, 0.0213], [-0.0663-0.00785, 0 , .0892], [-0.0743, 0, .5142], [-0.0743 +0.0174 -0.00785, 0.39225, .5142], [-0.0743 +0.0174-0.0531, 0.04165+0.39225+0.00785, .5142], [-0.0743 +0.0174-0.0531, 0.04165+0.39225+0.0531 , .5142 -0.04165-0.00785]]) # length = 6
        # centers = centers.to(device=xyz.device)
        # segmented_points = []
        
        # # Box condition
        # box_condition = ((points[:, 0] > -0.3) * (points[:, 0] < 0.2) * (points[:, 1] > -0.3) * (points[:, 1] < 0.6) * (points[:, 2] > 0.0) * (points[:, 2] < 0.6))
        
        
        # # Segment Base
        # condition = torch.where((points[:, 2] < centers[0, 2]) * box_condition)[0]
        # segmented_points.append(condition)
        
        # # Segment Link 1
        # condition = torch.where(((points[:, 2] > centers[0, 2])*(points[:, 0] > centers[1, 0])* (points[:, 2] < 0.2)) * box_condition
        #              )[0]
        # segmented_points.append(condition)
        
        # # Segment Link 2
        # condition1 = torch.where(((points[:,0] < centers[1,0]) * (points[:,2] > centers[0,2]) * (points[:,2] < 0.3))*box_condition)[0]
        # condition2 = torch.where(((points[:,0] < centers[2,0]) * (points[:, 2] >= 0.3) * (points[:, 1] < 0.1))*box_condition)[0]
        # condition = torch.cat([condition1, condition2])
        # segmented_points.append(condition)
        
        # # Segment Link 3
        # condition1 = torch.where(((points[:,0] > centers[2,0]) * (points[:,1] > (centers[2,1] - 0.1)) * (points[:,1] < 0.3) * (points[:,2] > 0.3))*box_condition)[0]
        # condition2 = torch.where(((points[:, 0] > centers[3, 0]) * (points[:, 1] >= 0.3) * (points[:, 2] > 0.3))*box_condition)[0]
        # condition = torch.cat([condition1, condition2])
        
        # segmented_points.append(condition)
        
        # # Segment Link 4
        # condition = torch.where(((points[:, 0] < centers[3, 0]) * (points[:, 1] > 0.25) * (points[:,1] < centers[4, 1]) * (points[:,2] > 0.3))*box_condition)[0]

        # segmented_points.append(condition)
        
        # # Segment Link 5
        # condition = torch.where(((points[:, 0] < centers[3, 0]) * (points[:,1] > centers[4, 1]) * (points[:, 2] > centers[5, 2]))*box_condition)[0]
        # segmented_points.append(condition)
        
        # # Transform the segmented points back to the original frame
        # # for i in range(len(segmented_points)):
        # #     segmented_points[i] = torch.matmul(R_inv, segmented_points[i].T).T + translation_inv
        # #     segmented_points[i] = segmented_points[i].to(device=xyz.device)
            
        # # # Just to visualize the segmented points
        # x_indices = segmented_points[1]
        
        
    else:
        x_indices = True * torch.ones(N, dtype=bool, device=xyz.device)
 
    xyz = xyz[x_indices]

    opacity = pc.get_opacity[x_indices] # shape (N, 1)
    
    means3D = xyz
    means2D = screenspace_points[x_indices] # shape (N, 3)

    # from 3d points in means3D, make a pointcloud using open3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(means3D.cpu().numpy())

    # Parametric equation for cylinder
    # sample theta and phi from 0 to 2pi
    t = torch.linspace(0, 2*math.pi, 100)
    h = torch.linspace(0, 1, 100)
    t, h = torch.meshgrid(t, h)
    x = 0.1*torch.cos(t)
    y = 0.1*torch.sin(t)
    z = h
    x = x.reshape(-1, 1)
    y = y.reshape(-1, 1)
    z = z.reshape(-1, 1)
    
    # Append the sphere to the original means3D
    means3D_orig = torch.stack((x, y, z), dim=1).to(device=means3D.device).squeeze()
    
    # Now cylinder along y-axis
    t = torch.linspace(0, 2*math.pi, 100)
    h = torch.linspace(0, 1, 100)
    t, h = torch.meshgrid(t, h)
    x = 0.1*torch.cos(t)
    y = h
    z = 0.1*torch.sin(t)
    x = x.reshape(-1, 1)
    y = y.reshape(-1, 1)
    z = z.reshape(-1, 1)
    
    # Append the cylinder to the original means3D
    means3D_orig = torch.cat((means3D_orig, torch.stack((x, y, z), dim=1).to(device=means3D.device).squeeze()), dim=0)
    
    # Now cone along x-axis
    t = torch.linspace(0, 2*math.pi, 100)
    h = torch.linspace(0, 1, 100)
    t, h = torch.meshgrid(t, h)
    x = h
    y = 0.1*torch.cos(t)
    z = 0.1*torch.sin(t)
    x = x.reshape(-1, 1)
    y = y.reshape(-1, 1)
    z = z.reshape(-1, 1)
    
    # Append the cone to the original means3D
    # means3D_orig = torch.cat((means3D_orig, torch.stack((x, y, z), dim=1).to(device=means3D.device).squeeze()), dim=0)
    
    # Add a red cube in the scene by adding gaussians
    # cube dimensions 0.2 x 0.2 x 0.2 with centre at (0, 0.2, 0)
    Trans = torch.tensor([[-0.272776812315, -0.074060574174, 0.179010376334, -0.366493761539],
                            [-0.193020626903, 0.130256026983, -0.240235880017, 0.399390488863],
                            [-0.016514312476, -0.299140989780, -0.148925781250, 0.708794176579],
                            [0, 0, 0, 1]]).to(device=xyz.device) # shape (4, 4)
    
    scale_robot = 0.33456
    rotation_matrix = Trans[:3, :3] #/ scale_robot
    translation = Trans[:3, 3]
    inv_transformation = torch.inverse(Trans)
    inv_rotation_matrix = inv_transformation[:3, :3]
    translation_inv = inv_transformation[:3, 3]
    
    cube = torch.arange(-0.1, 0.1, 0.01).to(device=xyz.device)
    cube = torch.cartesian_prod(cube, cube, cube) # shape (N, 3)
    n_cube = cube.shape[0]
    cube = cube + torch.tensor([0, 0.2, 0]).to(device=xyz.device)
    
    # parametric form of a sphere
    # sample theta and phi from 0 to 2pi
    theta = torch.linspace(0, 2*math.pi, 100)# parametric form of a cube
    
    phi = torch.linspace(0, math.pi, 100)
    theta, phi = torch.meshgrid(theta, phi)
    x = 0.1*torch.sin(phi)*torch.cos(theta)
    y = 0.1*torch.sin(phi)*torch.sin(theta)
    z = 0.1*torch.cos(phi)
    x = x.reshape(-1, 1)
    y = y.reshape(-1, 1)
    z = z.reshape(-1, 1)
    
    
    # Transform the cube to the robot frame
    cube = torch.matmul(inv_rotation_matrix, cube.T).T + translation_inv
    
    # means3D = torch.cat((means3D, cube), dim=0) # shape (N + N_cube, 3)
    means3D = torch.cat((means3D, cube), dim=0) # shape (N + N_sphere, 3)
    print("means3D shape after cube concat ", means3D.shape)
    opacity_cube = torch.ones((n_cube, 1), dtype=opacity.dtype, device=opacity.device)
    opacity = torch.cat((opacity, opacity_cube), dim=0) # shape (N + N_cube, 1)
    
    # print('means3D_orig = ', means3D_orig.shape)
    # means3D = torch.cat((means3D, means3D_orig), dim=0) # shape (N + N_orig, 3)
    # print("means3D shape after sphere concat ", means3D.shape)
  
    scales = None
    rotations = None
    cov3D_precomp = None
    if pipe.compute_cov3D_python:
        cov3D_precomp = pc.get_covariance(scaling_modifier) # shape (N, 3, 3)
        cov3D_precomp = cov3D_precomp[x_indices]
        # Apply the transformation matrix to the covariance matrix
        # cov3D_eye = torch.eye(4, dtype=cov3D_precomp.dtype, device=cov3D_precomp.device)
        # cov3D_eye[:3, :3] = cov3D_precomp
        # cov3D_precomp = cov3D_eye
        # cov3D_precomp = torch.matmul(Trans, cov3D_precomp)
        print('cov3D_precomp = ', cov3D_precomp[0])
        print('cov3D_precomp = ', cov3D_precomp.shape)
        # conv3D_orig = 0.01*torch.eye(N, cov3D_precomp.shape[1], dtype=cov3D_precomp.dtype, device=cov3D_precomp.device)
        # cov3D_precomp = torch.cat((cov3D_precomp, conv3D_orig), dim=0) # shape (N + N_orig, 3, 3)
        
    else:
        scales = pc.get_scaling # shape (N, 3)
        rotations = pc.get_rotation
        
        scales = scales[x_indices]
        scale_avg = torch.mean(scales, dim=0, keepdim=True)
        scales_orig = 0.1 * torch.ones((n_cube, scales.shape[1]), dtype=means3D.dtype, device=means3D.device)
        scales = torch.cat((scales, scales_orig), dim=0) # shape (N + n_cube, 3)
        # Rotate the scales
        # scales = torch.mm(R, scales.T).T
        rotations = rotations[x_indices] 
        rot_cube = torch.zeros(n_cube, 4).to(device=rotations.device)
        rotations = torch.cat((rotations, rot_cube), dim=0) # shape (N + n_cube, 4)
        # rotate rotations
        # print('rotations = ', rotations)
        # rotations = torch.mm(R, rotations.T).T
        # quat2rot = torch.tensor([0.966,0.259,0,0]).reshape(1, 4).to(device=rotations.device)
        
        # rotations = quaternion_multiply(quat2rot, rotations) # shape (N, 4)
        # print('rotations = ', rotations.shape)
        
        # torch.cross(quat2rot, rotations)
        # rotations_orig = torch.eye(N, rotations.shape[1], dtype=rotations.dtype, device=rotations.device)
        # rotations = torch.cat((rotations, rotations_orig), dim=0) # shape (N + N_orig, 3)
    # If precomputed colors are provided, use them. Otherwise, if it is desired to precompute colors
    # from SHs in Python, do it. If not, then SH -> RGB conversion will be done by rasterizer.
    shs = None
    colors_precomp = None
    if override_color is None:
        if pipe.convert_SHs_python:
            shs_view = pc.get_features.transpose(1, 2).view(-1, 3, (pc.max_sh_degree+1)**2)
            dir_pp = (pc.get_xyz - viewpoint_camera.camera_center.repeat(pc.get_features.shape[0], 1))
            dir_pp_normalized = dir_pp/dir_pp.norm(dim=1, keepdim=True)
            sh2rgb = eval_sh(pc.active_sh_degree, shs_view, dir_pp_normalized)
            colors_precomp = torch.clamp_min(sh2rgb + 0.5, 0.0)
            colors_precomp = colors_precomp[x_indices] # shape (N, 3)
            # concatenate red color for the last 1000 points
            red_tensor = torch.tensor([[1.0, 0.0, 0.0]]).repeat(n_cube, 1).to(colors_precomp.device)
            colors_precomp = torch.cat([colors_precomp, red_tensor], dim=0)
            print('colors_precomp = ', colors_precomp.shape)
            # colors_precomp = torch.cat((colors_precomp, torch.ones((N, colors_precomp.shape[1]), dtype=colors_precomp.dtype, device=colors_precomp.device)), dim=0)
            
        else:
            shs = pc.get_features 
            shs = shs[x_indices]
            rgb = torch.tensor([[1.0, 0.0, 0.0]]).repeat(n_cube, 1).to(device=shs.device)
            red_sh = RGB2SH(rgb).unsqueeze(1).repeat(1, 16, 1)
            print('red shs = ', red_sh.shape)
            shs = torch.cat((shs, red_sh), dim=0)
            shs_orig = torch.ones((N, shs.shape[1], shs.shape[2]), dtype=shs.dtype, device=shs.device)
            #red color
            print('shs = ', shs.shape)
            shs_orig[:, 0, :] = 3
            shs_orig[:, 1, :] = 0
            shs_orig[:, 2, :] = 0
            # shs = torch.cat((shs, shs_orig), dim=0)
    else:
        colors_precomp = override_color
        colors_precomp = colors_precomp[x_indices]
        

    # Rasterize visible Gaussians to image, obtain their radii (on screen). 
    rendered_image, radii = rasterizer(
        means3D = means3D,
        means2D = means2D,
        shs = shs,
        colors_precomp = colors_precomp,
        opacities = opacity,
        scales = scales,
        rotations = rotations,
        cov3D_precomp = cov3D_precomp)
    print("rasterization done")

    # Those Gaussians that were frustum culled or had a radius of 0 were not visible.
    # They will be excluded from value updates used in the splitting criteria.
    return {"render": rendered_image,
            "viewspace_points": screenspace_points,
            "visibility_filter" : radii > 0,
            "radii": radii}
