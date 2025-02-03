
import pybullet as p

import numpy as np
import open3d as o3d


import urdf_models.models_data as md   ### object models

def get_point_cloud(width, height, view_matrix, proj_matrix):
    # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

    # get a depth image
    # "infinite" depths will have a value close to 1
    image_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    depth = np.array(image_arr[3])
    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    color = np.array(image_arr[2])
    color = color.reshape(height, width, 4)
    color = color[:, :, :3]
    color = color.reshape(-1, 3)


    # filter out "infinite" depths
    pixels = pixels[z < 0.99]
    
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    return points, color





###############################################################################
#### Camera Parameters ####
###############################################################################
width, height = 1000, 1000
viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[1.4, 0, 0.8],
    cameraTargetPosition=[0, 0, 0.5],
    cameraUpVector=[-1, 0, 0])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

width_1, height_1 = width, height
viewMatrix_1 = p.computeViewMatrix(
    cameraEyePosition=[-1.4, 0, 0.8],
    cameraTargetPosition=[0, 0, 0.5],
    cameraUpVector=[1, 0, 0])

projectionMatrix_1 = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)


width_2, height_2 = width, height
viewMatrix_2 = p.computeViewMatrix(
    cameraEyePosition=[1.5, 0, 0.5],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[-1, 0, 0])

projectionMatrix_2 = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)



# Camera Setup for Surrounding the Origin

# Front Camera
width_front, height_front = width, height
viewMatrix_front = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 1],
    cameraTargetPosition=[0, 0, 0.0],
    cameraUpVector=[0, 1, 0])
projectionMatrix_front = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# Back Camera
width_back, height_back = width, height
viewMatrix_back = p.computeViewMatrix(
    cameraEyePosition=[0, 0, -1],
    cameraTargetPosition=[0, 0, 0.0],
    cameraUpVector=[0, 1, 0])
projectionMatrix_back = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# Left Camera
width_left, height_left = width, height
viewMatrix_left = p.computeViewMatrix(
    cameraEyePosition=[1, 0, 0],
    cameraTargetPosition=[0, 0, 0.0],
    cameraUpVector=[0, 1, 0])
projectionMatrix_left = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# Right Camera
width_right, height_right = width, height
viewMatrix_right = p.computeViewMatrix(
    cameraEyePosition=[-1, 0, 0],
    cameraTargetPosition=[0, 0, 0.0],
    cameraUpVector=[0, 1, 0])
projectionMatrix_right = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# Top Camera
width_top, height_top = width, height
viewMatrix_top = p.computeViewMatrix(
    cameraEyePosition=[0, 1, 0],
    cameraTargetPosition=[0, 0, 0.0],
    cameraUpVector=[0, 0, -1])
projectionMatrix_top = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# Bottom Camera
width_bottom, height_bottom = width, height
viewMatrix_bottom = p.computeViewMatrix(
    cameraEyePosition=[0, -1, 0],
    cameraTargetPosition=[0, 0, 0.0],
    cameraUpVector=[0, 0, 1])
projectionMatrix_bottom = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

viewMatrixs = [viewMatrix_front, viewMatrix_back, viewMatrix_left, viewMatrix_right, viewMatrix_top, viewMatrix_bottom, viewMatrix, viewMatrix_1, viewMatrix_2]
projectionMatrixs = [projectionMatrix_front, projectionMatrix_back, projectionMatrix_left, projectionMatrix_right, projectionMatrix_top, projectionMatrix_bottom, projectionMatrix, projectionMatrix_1, projectionMatrix_2]


def get_overall_pcd(imgx = 1000, imgy = 1000):
    point_list = []
    color_list = []
    for viewMatrix, projectionMatrix in zip(viewMatrixs, projectionMatrixs):
        points, color = get_point_cloud(imgx, imgy, viewMatrix, projectionMatrix)
        point_list.append(points)
        color_list.append(color)

    points = np.concatenate(point_list, axis=0)
    colors = np.concatenate(color_list, axis=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd