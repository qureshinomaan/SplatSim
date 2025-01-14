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
from collections import namedtuple
import cv2


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
    #save depth image
    print('max : ', np.max(depth))
    grayscale_depth = np.array((1-depth) * 255, dtype=np.uint8)
    cv2.imwrite('stream/depth_image_' + str(time.time()) + '.png', grayscale_depth)

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
    color = color[z < 0.99] / 255.
    
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
    cameraTargetPosition=[0, 0, 0.5],
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
    cameraTargetPosition=[0, 0, 0.5],
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
    cameraTargetPosition=[0, 0, 0.5],
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
    cameraTargetPosition=[0, 0, 0.5],
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
    cameraTargetPosition=[0, 0, 0.5],
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
    cameraTargetPosition=[0, 0, 0.5],
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
pandaUid = p.loadURDF('pybullet-playground_2/urdf/sisbot.urdf',useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
# pandaUid = p.loadURDF('ros_industrial_training/training/ref/2.8/lesson_urdf/urdf/ur5.urdf',useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
models_lib = md.model_lib()
object_name_list = models_lib.model_name_list
object_name = object_name_list[random.randint(0, len(object_name_list))]
print('object_name_list:', object_name_list)
#find index of 'power_drill' in object_name_list
# object_name = 'power_drill'
# pandaUid = p.loadURDF(models_lib[object_name], [0.5, 0.15, 0.0],  p.getQuaternionFromEuler([0,0,0]), flags=flags, useFixedBase=False)


#shift the robot to 0,0,0
p.resetBasePositionAndOrientation(pandaUid, [0, 0, -0.1], [0, 0, 0, 1])

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
joint_poses = [0, 0, -90, 90, -90, -90, 0]
#convert to radians
joint_poses = [x * np.pi / 180.0 for x in joint_poses]

#set joint positions
for i in range(6):
    p.resetJointState(pandaUid, i, joint_poses[i])

joints = []
controllable_joints = []

def __parse_joint_info__():
        numJoints = p.getNumJoints(pandaUid)
        jointInfo = namedtuple('jointInfo',
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        
        for i in range(numJoints):
            info = p.getJointInfo(pandaUid, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                controllable_joints.append(jointID)
                p.setJointMotorControl2(pandaUid, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            joints.append(info)

mimic_parent_id = None
def setup_gripper():
    __parse_joint_info__()
    global mimic_parent_id
    gripper_range = [0, 0.085]
    mimic_parent_name = 'finger_joint'
    mimic_children_names = {'right_outer_knuckle_joint': 1,
                            'left_inner_knuckle_joint': 1,
                            'right_inner_knuckle_joint': 1,
                            'left_inner_finger_joint': -1,
                            'right_inner_finger_joint': -1}
    # self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)
    mimic_parent_id = [joint.id for joint in joints if joint.name == mimic_parent_name][0]
    mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in joints if joint.name in mimic_children_names}
    for joint_id, multiplier in mimic_child_multiplier.items():
        c = p.createConstraint(pandaUid, mimic_parent_id,
                                pandaUid, joint_id,
                                jointType=p.JOINT_GEAR,
                                jointAxis=[0, 1, 0],
                                parentFramePosition=[0, 0, 0],
                                childFramePosition=[0, 0, 0])
        p.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)  # Note: the mysterious `erp` is of EXTREME importance

setup_gripper()

import math
def move_gripper(open_length):
        # open_length = np.clip(open_length, *self.gripper_range)
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation
        # Control the mimic gripper joint(s)
        p.setJointMotorControl2(pandaUid, mimic_parent_id, p.POSITION_CONTROL, targetPosition=open_angle,
                                force=joints[mimic_parent_id].maxForce, maxVelocity=joints[mimic_parent_id].maxVelocity)

## print joint states
for i in range(p.getNumJoints(pandaUid)):
    joint_info = p.getJointInfo(pandaUid, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_state = p.getJointState(pandaUid, i)
    print('joint_name:', joint_name)
    print('joint_state:', joint_state)
    print('joint_info:', joint_info)
    print()



colors = [[0, 0, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0], [1, 0, 1], [1, 1, 0], [1, 1, 0.5], [0, 0.5, 1]]

colors = [[0, 0, 0]]


for i in range(15):
    max_dist = 0
    potential_color = None
    for a in range(10):
        for b in range(10):
            for c in range(10):
                new_color = [a/10., b/10., c/10.]
                distances = []
                for color in colors:
                    dist = np.linalg.norm(np.array(color) - np.array(new_color))
                    distances.append(dist)
                if min(distances) > max_dist and new_color not in colors:
                    max_dist = min(distances)
                    potential_color = new_color

    if potential_color is not None:
        copy_color = potential_color.copy()
        colors.append(copy_color)

print('colors:', colors)
                  
                 


for i in range(0, 7):
    p.changeVisualShape(pandaUid, i, rgbaColor=colors[0]+[1])


num_links = p.getNumJoints(pandaUid)
for i in range(8, num_links):
    print('i:', i)
    if i - 7 >= len(colors):
        break
    #change color of each link to random color

    random_color =  colors[i-7]
    p.changeVisualShape(pandaUid, i, rgbaColor=random_color+[1])

for i in range(10000):
    p.stepSimulation()
    move_gripper(0.0)


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
    print('colors.shape :', colors.shape)

    print('points.shape:', points.shape)
    #mirror points
    # points[:, 0] = -points[:, 0]



    # poitns2 = get_point_cloud(width_2, height_2, viewMatrix_2, projectionMatrix_2)


    # save point cloud using open3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    pcd.colors = o3d.utility.Vector3dVector(colors) 

    #see pcd.colors shape
    print('pcd.colors:', np.asarray(pcd.colors).shape)

    o3d.io.write_point_cloud("robot_pcd.ply", pcd)

    #visualize point cloud
    o3d.visualization.draw_geometries([pcd])

    p.stepSimulation()
    time.sleep(1./240.)