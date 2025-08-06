import os
import pickle
import pybullet as p
import pybullet_data
import math
import numpy as np
import time
from ikfast_franka_panda import get_fk, get_ik, get_dof, get_free_dof
from pybullet_planning import plan_joint_motion, get_movable_joints, set_joint_positions
import random
import imageio

import urdf_models.models_data as md   ### object models
from pybullet_planning.interfaces.robots.collision import pairwise_collision
import time


import os.path
from os import path

p.connect(p.GUI)
# p.setPhysicsEngineParameter(solverResidualThreshold=0)
p.resetSimulation()


###############################################################################
#### Camera Parameters ####
###############################################################################
width, height = 100, 100
viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[1.5, 0, 0.5],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[-1, 0, 0])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    width=width,
    height=height,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)


viewMatrix_1 = p.computeViewMatrix(
    cameraEyePosition=[0, -1, 1],
    cameraTargetPosition=[0.5, 0, 0],
    cameraUpVector=[0.5, 1, 0])

projectionMatrix_1 = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

width_1, height_1, rgbImg_1, depthImg_1, segImg_1 = p.getCameraImage(
    width=width,
    height=height,
    viewMatrix=viewMatrix_1,
    projectionMatrix=projectionMatrix_1)


viewMatrix_2 = p.computeViewMatrix(
    cameraEyePosition=[0, 1, 1],
    cameraTargetPosition=[0.5, 0, 0],
    cameraUpVector=[0.5, -1, 0])

projectionMatrix_2 = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

width_2, height_2, rgbImg_2, depthImg_2, segImg_2 = p.getCameraImage(
    width=width,
    height=height,
    viewMatrix=viewMatrix_2,
    projectionMatrix=projectionMatrix_2)
###############################################################################


urdfRootPath=pybullet_data.getDataPath()
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
joint_ids = []
param_ids = []
num_joints = p.getNumJoints(pandaUid)

joint_states = p.getJointStates(pandaUid, range(0, num_joints))
joint_poses = [x[0] for x in joint_states]
idx = 0
for i in range(num_joints):
    joint_info = p.getJointInfo(pandaUid, i)
    joint_name = joint_info[1]
    joint_type = joint_info[2]

    if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
        joint_ids.append(i)
        param_ids.append(
            p.addUserDebugParameter(joint_name.decode("utf-8"), joint_info[8], joint_info[9], joint_poses[i]))
        idx += 1


models_lib = md.model_lib()
object_name_list = models_lib.model_name_list
object_name = object_name_list[random.randint(0, len(object_name_list))]
# object_name = 'potato_chip_1'

print('Object Name : ', object_name)
if path.exists('save_done_2/'+ object_name):
	exit()
else :
	f = open('save_done_2/'+ object_name, "w+")

# tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65], useFixedBase=True)
tableUid = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"),basePosition=[0.0,0,-0.02], useFixedBase=True)

p.changeDynamics(tableUid, -1, lateralFriction=1)
for _id in range(p.getNumJoints(tableUid)):
    p.changeDynamics(tableUid, _id, lateralFriction=1)

flags = p.URDF_USE_INERTIA_FROM_FILE
doorId = p.loadURDF(models_lib[object_name], [0.5, 0.15, 0.0],  p.getQuaternionFromEuler([0,0,0]), flags=flags, useFixedBase=False)

p.changeDynamics(doorId, -1, lateralFriction=1)
for _id in range(p.getNumJoints(doorId)):
    p.changeDynamics(doorId, _id, lateralFriction=1)

for i in range(0, 4500):
	if pairwise_collision(tableUid, doorId) == False : 
		print('i here : ', i)
		break
	p.resetBasePositionAndOrientation(doorId, [0.5, 0.15, i/1000], p.getQuaternionFromEuler([0,0,0]))


# boxId = p.loadURDF(models_lib['blue_cup'], [0.8, 0.15, 0.0],  p.getQuaternionFromEuler([0, 0,0]), useFixedBase=False)
p.setGravity(0,0,-10)
print('Dynamics information : ', p.getDynamicsInfo(doorId, -1))

p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[1.5, 0, 0.5],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[-1, 0, 0])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    width=1000,
    height=1000,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)

state_durations = [2, 4,1.0,0.6]
control_dt = 1./240
p.setTimestep = control_dt
state_t = 0.
current_state = 0

x = 0
while x < 1000:
    p.stepSimulation()
    x+=1


# pcd = get_point_cloud(width, height, viewMatrix, projectionMatrix)

### Inverse kinematics.
ll = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
ul = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

def get_optimal_joint(joints):
	optimal_index = 0
	for i in range(len(joints)):
		good =True
		for j in range(7):
			pass
			if joints[i][j] > ul[j] or joints[i][j] < ll[j]:
				good = False
		if good:
			return joints[i]

	# print('optimal not found!!')
	return []

def get_joint(t, r):
	for i in reversed(range(-4000, 4000, 1)):
		joints = get_ik(t, r, [float(i/1000)])
		optimal_joint = get_optimal_joint(joints)
		if optimal_joint:

			return optimal_joint
			break

	print('Joint Not Found!!!')
	return None

f = open('./grasp_parameters/'+object_name+'_pointcloud_pred_grasps_cam.pkl', 'rb')
pred_grasps_cam = pickle.load(f)
# print('pred grasps shape : ', pred_grasps_cam[-2].shape)

f = open('./grasp_parameters/'+object_name+'_pointcloud_scores.pkl', 'rb')
scores = pickle.load(f)
print('scores length : ', len(scores[-1]))


#### finding initial successfull grasp_pose :
i = len(scores[-1]) - 1
pandacount = 0
while i > 0 :
    print('ith time : ', i)
    grasp_pose = pred_grasps_cam[-1][np.argsort(scores[-1])[i]]
    r = np.copy(grasp_pose[:3, :3].T)
    r = np.matmul(np.array([[1, 0, 0],[0, -1, 0], [0, 0, -1]]).T, r.T)
    approach_vector = r[:, 2]/np.linalg.norm(r[:, 2])
    t = np.copy(grasp_pose[:3, 3])
    tnew = np.copy(t)
    t[0] = tnew[0]
    t[1] = -tnew[1]
    t[2] = -tnew[2]

    t_actual_copy = t
    t_actual = t + 0.00 * approach_vector
    t = t - 0.00 * approach_vector
    t_adjusted = t

    t_final = np.array(t) + np.array([0.0, 0.0, 0.2])
    trajs, joint_trajs = [], []
    goodTraj = True

    for j in range(11):
        trajs.append(t_actual + j*(t_final - t_actual)/10)
        possiblejoint = get_joint(trajs[-1], r)

        if possiblejoint is None or get_joint(t_adjusted, r) is None or t[0] < 0.2:
            print('joint not found')
            goodTraj = False
            break

        if j == 0:
            pj_2 = get_joint(t_adjusted, r)
            set_joint_positions(pandaUid, get_movable_joints(pandaUid), pj_2+[0.04, 0.04])
            print('Checking grasp collison : ')
            # if pairwise_collision(pandaUid, doorId) or pairwise_collision(pandaUid, tableUid):
            if pairwise_collision(pandaUid, doorId):
                print('collison checking : True')
                goodTraj = False
                break

            if -approach_vector[2] < 0.7:
                print('The grasp is not top-down')
                goodTraj = False
                break
            print('Not Found')

        joint_trajs.append(get_joint(trajs[-1], r) + [0.00, 0.00])


    if goodTraj == True:
        final_pose_joint = get_joint(t_final, r) + [0.00, 0.00]
        break
    i -= 1


print('t : ', t)
print('r now : ', r)
# p.resetBasePositionAndOrientation(doorId, [0.5, 0.15, 0.15], p.getQuaternionFromEuler([0,0,0]))
grasp_joint = get_joint(t, r) + [0.00, 0.00] ### actual grasp pose
pre_grasp_joint = get_joint(t, r) + [0.04, 0.04]   #### grasp pose, pregrasp
actual_grasp_joint = get_joint(t_actual, r) + [0.04, 0.04] #### actual grasp joint



t = np.array([0.5, -0.3, 0.6])
initial_pose_joint = get_joint(t, np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])) + [0.04, 0.04]  ### initial pose

# optimal_joint4 = optimal_joint2 + [0.00, 0.00]  ### Final pose

ik_joints = get_movable_joints(pandaUid)

set_joint_positions(pandaUid, ik_joints, initial_pose_joint)
path_1 = plan_joint_motion(pandaUid, ik_joints, pre_grasp_joint, obstacles=[doorId, tableUid], self_collisions=False)
print('plan 1 done')
set_joint_positions(pandaUid, ik_joints, grasp_joint)
path_2 = plan_joint_motion(pandaUid, ik_joints, final_pose_joint, obstacles=[tableUid], self_collisions=False)
path_2 = joint_trajs

if path_1 is None :
	print('Path to grasp not found!!!')
	exit()
elif path_2 is None :
	print('Path to execute skill not found!!!')
	exit()
else :
	print('Path found to grasp and execute skill')

path_1.append(actual_grasp_joint)
images = []
images_1 = []
images_2 = []

set_joint_positions(pandaUid, ik_joints, initial_pose_joint)

overallcnt = 0
cnt = 0
lasterror = 100
while True:
    if current_state > 0 and overallcnt % 50 == 0:
        image_arr = p.getCameraImage(width=width, height=height, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)
        images.append(np.array(image_arr[2]).reshape(width, height, 4))

        image_arr = p.getCameraImage(width=width, height=height, viewMatrix=viewMatrix_1, projectionMatrix=projectionMatrix_1)
        images_1.append(np.array(image_arr[2]).reshape(width, height, 4))

        image_arr = p.getCameraImage(width=width, height=height, viewMatrix=viewMatrix_2, projectionMatrix=projectionMatrix_2)
        images_2.append(np.array(image_arr[2]).reshape(width, height, 4))

    print('Current State : ', current_state)
    state_t += control_dt
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    if current_state == 0:
        pass

    if current_state == 1:
        if path_1 is not None:
            if cnt > len(path_1)-3:
                p.setJointMotorControlArray(pandaUid, joint_ids[:7], p.POSITION_CONTROL, targetPositions=path_1[cnt][:7], positionGains=[0.01]*7)
            else : 
                p.setJointMotorControlArray(pandaUid, joint_ids[:7], p.POSITION_CONTROL, targetPositions=path_1[cnt][:7])

            p.setJointMotorControlArray(pandaUid, joint_ids[7:], p.POSITION_CONTROL, targetPositions=path_1[cnt][7:])
            current_joint = np.array([p.getJointState(pandaUid, i)[0] for i in range(7)])
            error = np.linalg.norm(current_joint - np.array(path_1[cnt][:7]))
            print("error : ", error)

            if error < 0.05 and cnt < len(path_1) - 3:
                lasterror = 100
                cnt+=1
            elif error < 0.01:
                lasterror = 100
                cnt+=1
                state_t = 0

            if cnt >= len(path_1):
            	cnt = 0
            	current_state+=1
            	state_t = 0
        else :
            cnt = 0
            current_state+=1
            state_t = 0


    if current_state == 2:
        p.setJointMotorControl2(pandaUid, 9,
                        p.POSITION_CONTROL, 0.00, force = 10)
        p.setJointMotorControl2(pandaUid, 10,
                        p.POSITION_CONTROL, 0.00, force = 10)


    if current_state == 3:

    	p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.00, force = 200)
    	p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.00, force = 200)

    	if path_2 is not None and cnt < len(path_2):
            print(path_2[cnt])
            p.setJointMotorControlArray(pandaUid, joint_ids[:7], p.POSITION_CONTROL, targetPositions=path_2[cnt][:7], positionGains=[0.1]*7)
            p.setJointMotorControlArray(pandaUid, joint_ids[7:], p.POSITION_CONTROL, targetPositions=path_2[cnt][7:], forces=[200, 200])
            current_joint = np.array([p.getJointState(pandaUid, i)[0] for i in range(7)])
            error = np.linalg.norm(current_joint - np.array(path_2[cnt][:7]))
            print("error : ", error)
            if error < 0.05 and cnt < len(path_2) - 3:
                cnt+=1
            elif error < 0.01:
                cnt+=1
                state_t = 0

            if cnt >= len(path_2):
                current_state+=1
                state_t = 0
    	else :
    		current_state+=1
    		state_t = 0

    if current_state > 3:
    	break
    if (state_t >state_durations[current_state] and state_t != 1):
    	if current_state==0:
    		set_joint_positions(pandaUid, ik_joints, initial_pose_joint)
    	current_state += 1
    	if current_state >= len(state_durations) and current_state==0:
    		print('HERE!!')
    		current_state = 0
    	elif current_state >= len(state_durations):
    		break
    	state_t = 0
    p.stepSimulation()

    overallcnt += 1


imageio.mimsave('./gifs/'+object_name+'x.gif', images)
imageio.mimsave('./gifs/'+object_name+'x_1.gif', images_1)
imageio.mimsave('./gifs/'+object_name+'x_2.gif', images_2)