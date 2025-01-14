import pybullet as p
import numpy as np
import pybullet_data
from collections import namedtuple


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



def quaternion_to_rot_matrix(quat):
    """
    Convert a quaternion into a rotation matrix.
    """
    w, x, y, z = quat
    return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                     [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])

p.connect(p.GUI)
p.resetSimulation()
urdfRootPath = pybullet_data.getDataPath()
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
pandaUid = p.loadURDF('pybullet-playground_2/urdf/sisbot.urdf', useFixedBase=True, basePosition=[0.0, 0.0, 0.0], flags=flags)
p.resetBasePositionAndOrientation(pandaUid, [0, 0, -0.1], [0, 0, 0, 1])


# Set the robot to a known configuration
joint_poses = [0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]  # Example configuration
for i in range(len(joint_poses)):
    p.resetJointState(pandaUid, i, joint_poses[i])

for i in range(7, 20):
    p.resetJointState(pandaUid, i, 0)

# Getting global axis and position of each joint
for joint_index in range(p.getNumJoints(pandaUid)):
    joint_info = p.getJointInfo(pandaUid, joint_index)
    joint_name = joint_info[1].decode("utf-8")
    print('joint info:', joint_info)
    joint_axis_local = np.array(joint_info[13])  # Local axis of the joint
    link_state = p.getLinkState(pandaUid, joint_index, computeForwardKinematics=True)
    link_world_pos = link_state[0]
    link_world_ori = link_state[1]
    print('link_world_pos:', link_world_pos)

    #place a sphere at the link_world_pos
    # sphereUid = p.loadURDF('sphere.urdf', basePosition=link_world_pos)
    # p.changeVisualShape(sphereUid, -1, rgbaColor=[1, 0, 0, 1])
    p.stepSimulation()
    print()

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

move_gripper(0.0)

for i in range(10000):
    p.stepSimulation()
    

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


    

p.disconnect()
