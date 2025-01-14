from roboticstoolbox import ERobot
import pybullet as p
import numpy as np

# Start the PyBullet physics server
p.connect(p.DIRECT)

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# Load your URDF file
base_transform = np.eye(4)
base_transform[2, 3] = -0.1
robot = ERobot.URDF("/home/nomaan/Desktop/corl24/main/gaussian-splatting/pybullet-playground/urdf/sisbot.urdf", gripper='robotiq_arg2f_base_link')

#Load the robot in the PyBullet physics server
robot_pybullet = p.loadURDF("/home/nomaan/Desktop/corl24/main/gaussian-splatting/pybullet-playground/urdf/sisbot.urdf", [0, 0, -0.1], useFixedBase=True, )

# forwards kinematics
q = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
Transform_rtb = np.asarray(robot.fkine(q)).reshape(4, 4)
print('Transform_rtb', Transform_rtb)

#do the forward kinematics on the PyBullet robot
for i in range(1, 7):
    p.resetJointState(robot_pybullet, i, q[i-1])

#get the end effector position and orientation
pos, ori = p.getLinkState(robot_pybullet, 6, computeForwardKinematics=True)[4:6]
ori_matrix = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
Transformation_pybulet = np.eye(4)
Transformation_pybulet[:3, :3] = ori_matrix
Transformation_pybulet[:3, 3] = np.array(pos)
print(pos, ori_matrix)



test_end_effector_pos = np.array([0.2, 0.2, 0.2])
test_end_effector_ori = np.eye(3)
quat = p.getQuaternionFromEuler(rotation_matrix_to_euler_angles(test_end_effector_ori))

#use pybullet inverse kinematics to get the joint angles
joint_angles = p.calculateInverseKinematics(robot_pybullet, 6, test_end_effector_pos, quat, maxNumIterations=10000)

print('joint_angles', joint_angles)

#set the joint angles
for i in range(1, 7):
    p.resetJointState(robot_pybullet, i, joint_angles[i-1])

#get the end effector position and orientation
pos, ori = p.getLinkState(robot_pybullet, 6, computeForwardKinematics=True)[4:6]
ori_matrix = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
print('pos', pos)
print('ori', ori_matrix)
exit()

#first step : transform the end effector position and orientation from the PyBullet frame to the roboticstoolbox frame
test_end_effector_ori = np.linalg.inv(Transformation_pybulet)[:3, :3] @ test_end_effector_ori
test_end_effector_pos = np.linalg.inv(Transformation_pybulet)[:3, :3] @ (test_end_effector_pos) + np.linalg.inv(Transformation_pybulet)[:3, 3]

test_end_effector_ori = Transform_rtb[:3, :3] @ test_end_effector_ori
test_end_effector_pos = Transform_rtb[:3, :3] @ (test_end_effector_pos) + Transform_rtb[:3, 3]

#printing overall transformation
overall_transformation = np.matmul(Transform_rtb, np.linalg.inv(Transformation_pybulet))
print('overall_transformation', overall_transformation)

#second step : do the inverse kinematics on the roboticstoolbox robot
print('new pos', test_end_effector_pos)
print('new ori', test_end_effector_ori)


tep = np.eye(4)
tep[:3, :3] = test_end_effector_ori
tep[:3, 3] = test_end_effector_pos
q = robot.ik_LM(tep, mask=[2, 2, 2, 1, 1, 1], tol=1e-10)

print('q', q)


#third step : do the forward kinematics on the PyBullet robot
for i in range(1, 7):
    p.resetJointState(robot_pybullet, i, q[0][i-1])

#get the end effector position and orientation
pos, ori = p.getLinkState(robot_pybullet, 6, computeForwardKinematics=True)[4:6]
ori_matrix = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)

print('pos', pos)
print('ori', ori_matrix)

