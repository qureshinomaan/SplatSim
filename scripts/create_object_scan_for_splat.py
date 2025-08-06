import os
import pybullet as p
import pybullet_data
import numpy as np
import random
import imageio
import urdf_models.models_data as md
import time

import argparse
from PIL import Image

def load_textures(texture_folder, scale_factor=1.0):
    textures = []
    for f in os.listdir(texture_folder):
        if f.endswith('.png'):
            # Load image and get original size
            img_path = os.path.join(texture_folder, f)
            # img = Image.open(img_path)
            # orig_width, orig_height = img.size
            
            # # Calculate new size based on scale factor
            # new_size = (int(orig_width * scale_factor), int(orig_height * scale_factor))
            # img = img.resize(new_size, Image.LANCZOS)
            
            # # Save temporarily and load into PyBullet
            # temp_path = os.path.join(texture_folder, f'temp_{f}')
            # img.save(temp_path)
            texture_id = p.loadTexture(img_path)
            textures.append(texture_id)
            
            # Clean up temporary file
            # os.remove(temp_path)
    
    return textures

def create_textured_walls(textures):
    wall_size = [3, 3, 0.1]  # Size of each wall (length, height, thickness)
    wall_positions = [
        [0, 0, 2],  # Front wall
        [0, 0, -2],  # Back wall
        [2, 0, 0],  # Right wall
        [-2, 0, 0],  # Left wall
        [0, 2, 0],   # Ceiling
        [0, -2, 0]   # Ceiling
    ]

    angles = [0, 0, np.pi/2, np.pi/2] #np.pi, 3 * np.pi / 2]

    quats = [p.getQuaternionFromEuler([0, angle, 0]) for angle in angles]
    quats.append(p.getQuaternionFromEuler([np.pi/2, 0, 0]))
    quats.append(p.getQuaternionFromEuler([np.pi/2, 0, 0]))

    quats = [list(q) for q in quats]

    wall_orientations = quats
    
    wall_ids = []
    for pos, orn, tex in zip(wall_positions, wall_orientations, textures):
        wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_size)
        visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_size)
        body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_id, baseVisualShapeIndex=visual_shape_id, basePosition=pos, baseOrientation=orn)
        p.changeVisualShape(body_id, -1, textureUniqueId=tex)
        wall_ids.append(body_id)
    return wall_ids

def spherical_camera_params(radius, theta, phi):
    x = radius * np.sin(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.sin(phi)
    z = radius * np.cos(theta)
    camera_position = [x, y, z]
    target_position = [0, 0, 0]
    up_vector = [0, 0, 1] if z > 0 else [0, 0, -1]
    return camera_position, target_position, up_vector

def create_spherical_views(output_folder, texture_folder, num_views=100):
    os.makedirs(output_folder, exist_ok=True)
    width, height = 1000, 1000

    textures = load_textures(texture_folder)
    create_textured_walls(textures)

    radius = 1.5
    num_theta_steps = int(np.sqrt(num_views))
    num_phi_steps = int(np.sqrt(num_views))
    
    view_index = 0
    for theta_step in range(num_theta_steps):
        theta = np.pi * theta_step / (num_theta_steps - 1)  # theta from 0 to pi
        for phi_step in range(num_phi_steps):
            phi = 2 * np.pi * phi_step / (num_phi_steps - 1)  # phi from 0 to 2*pi
            camera_position, target_position, up_vector = spherical_camera_params(radius, theta, phi)
            viewMatrix = p.computeViewMatrix(camera_position, target_position, up_vector)
            projectionMatrix = p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=5.1)

            image_arr = p.getCameraImage(width=width, height=height, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix, renderer=p.ER_TINY_RENDERER, lightDirection=[0,0,-1], lightDistance=0.)
            rgb_image = np.array(image_arr[2]).reshape(height, width, 4)[:, :, :3]

            imageio.imwrite(os.path.join(output_folder, f"view_{view_index:03d}.png"), rgb_image)
            print(f"Saved view {view_index+1}/{num_views}")
            view_index += 1



if __name__ == "__main__":

    #argument parser --> object_name, urdf_path, texture_folder, num_views
    parser = argparse.ArgumentParser(description='Create random views of an object')
    parser.add_argument('--texture_folder', type=str, default='textures', help='Folder containing background textures')
    parser.add_argument('--num_views', type=int, default=300, help='Number of views to be rendered')
    parser.add_argument('--object_name', type=str, help='Name of the object to be rendered')
    parser.add_argument('--urdf_path', type=str, default='../../../virtual_objects/greenblock/object.urdf', help='Path to the URDF file of the object')

    args = parser.parse_args()

    p.connect(p.GUI)
    p.resetSimulation()
    time.sleep(1)
    urdfRootPath = pybullet_data.getDataPath()
    models_lib = md.model_lib()
    object_name_list = models_lib.model_name_list
 
    object_name = args.object_name
    pandaUid = p.loadURDF(args.urdf_path, useFixedBase=True, globalScaling=0.33)
    #load the texture from ../textures/flower.png
    texture_id = p.loadTexture(os.path.join(args.texture_folder, 'object/flower.png'))
    texture_id_1 = p.loadTexture(os.path.join(args.texture_folder, 'object/link1.png'))
    texture_id_2 = p.loadTexture(os.path.join(args.texture_folder, 'object/link2.png'))
    p.changeVisualShape(pandaUid, 0, textureUniqueId=texture_id_1)
    p.changeVisualShape(pandaUid, 1, textureUniqueId=texture_id_2)
    p.changeVisualShape(pandaUid, 2, textureUniqueId=texture_id)

    output_folder = "scans_for_splat/random_views_" + object_name
    texture_folder = args.texture_folder
    num_views = args.num_views
    create_spherical_views(output_folder, texture_folder, num_views)


