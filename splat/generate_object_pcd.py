import os
import pickle
import pybullet as p
import pybullet_data
import math
import numpy as np
import time
# from ikfast_franka_panda import get_fk, get_ik, get_dof, get_free_dof
# from pybullet_planning import plan_joint_motion, get_movable_joints, set_joint_positions
import random
import imageio
import open3d as o3d


import urdf_models.models_data as md   ### object models
# from pybullet_planning.interfaces.robots.collision import pairwise_collision
import time

def get_point_cloud(width, height, view_matrix, proj_matrix):
    # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

    # get a depth image
    # "infinite" depths will have a value close to 1
    image_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    depth = np.array(image_arr[3])
    print('image_arr : ', type(image_arr[3][0]))
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


p.connect(p.GUI)
# p.setPhysicsEngineParameter(solverResidualThreshold=0)
p.resetSimulation()


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


###############################################################################


urdfRootPath=pybullet_data.getDataPath()
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
# pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
# pandaUid = p.loadURDF('pybullet-playground/urdf/sisbot.urdf',useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
# pandaUid = p.loadURDF('ros_industrial_training/training/ref/2.8/lesson_urdf/urdf/ur5.urdf',useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
models_lib = md.model_lib()
object_name_list = models_lib.model_name_list
object_name = object_name_list[random.randint(0, len(object_name_list))]
print('object_name_list:', object_name_list)
#find index of 'power_drill' in object_name_list
object_name = 'glasses'
# pandaUid = p.loadURDF(models_lib[object_name], [0.5, 0.15, 0.0],  p.getQuaternionFromEuler([0,0,0]), flags=flags, useFixedBase=False)

# object_name = 'orange_bin'
# pandaUid = p.loadURDF('../../virtual_objects/T_object/object.urdf', useFixedBase=True)
pandaUid = p.loadURDF('articulated_objects/articulated_object_urdfs/101297/mobility.urdf', useFixedBase=True, globalScaling=0.33)
# pandaUid = p.loadURDF(models_lib[object_name], useFixedBase=True)

#shift the robot to 0,0,0
p.resetBasePositionAndOrientation(pandaUid, [0, 0, 0], [0, 0, 0, 1])

joint_ids = []
param_ids = []
# num_joints = p.getNumJoints(pandaUid)
# print('num_joints:', num_joints)

# joint_states = p.getJointStates(pandaUid, range(0, num_joints))
# joint_poses = [x[0] for x in joint_states]
# print('joint_poses:', joint_poses)

#colors for each joint
# colors = [np.random.rand(3).tolist() +[1] for _ in range(num_joints)]


#joint positions in degrees
joint_poses = [0, 90, -90, 90, -90, -90, 0]
#convert to radians
joint_poses = [x * np.pi / 180.0 for x in joint_poses]

#set joint positions
# for i in range(6):
#     p.resetJointState(pandaUid, i, joint_poses[i])


# for i in range(6):
#     p.changeVisualShape(pandaUid, linkIndex=i, rgbaColor=colors[i])

#get axis aligned bounding box of the object
aabb = p.getAABB(pandaUid)
print('aabb:', aabb)
# exit()

while True:
    image_arr = p.getCameraImage(width=width, height=height, viewMatrix=viewMatrix_1, projectionMatrix=projectionMatrix_1)
    points =[]
    colors = []
    for i in range(len(viewMatrixs)):
        # image_arr = p.getCameraImage(width=width, height=height, viewMatrix=viewMatrixs[i], projectionMatrix=projectionMatrixs[i])
        points_, color = get_point_cloud(width, height, viewMatrixs[i], projectionMatrixs[i])
        points.append(points_)
        colors.append(color)

    points = np.concatenate(points, axis=0)
    colors = np.concatenate(colors, axis=0)

    #get aabb from the point cloud
    aabb = np.array([np.min(points, axis=0), np.max(points, axis=0)])

    print('aabb:', aabb)

    #mirror points
    # points[:, 0] = -points[:, 0]
    # poitns2 = get_point_cloud(width_2, height_2, viewMatrix_2, projectionMatrix_2)


    # save point cloud using open3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    pcd.colors = o3d.utility.Vector3dVector(colors) 

    o3d.io.write_point_cloud("object_pcds/"+object_name+".ply", pcd)

    p.stepSimulation()
    time.sleep(1./240.)