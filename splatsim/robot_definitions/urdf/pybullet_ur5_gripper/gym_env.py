# E. Culurciello
# February 2021

# PyBullet UR-5 from https://github.com/josepdaniel/UR5Bullet
# gripper from: https://github.com/matafela/pybullet_grasp_annotator_robotiq_85

import random
import time
import numpy as np
import sys
from gym import spaces
import gym

import os
import math 
import pybullet
import pybullet_data
from datetime import datetime
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict

ROBOT_URDF_PATH = "robots/urdf/ur5e_with_gripper.urdf"
TABLE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "table/table.urdf")
CUBE_URDF_PATH = os.path.join(pybullet_data.getDataPath(), "cube_small.urdf")

# x,y,z distance
def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


# x,y distance
def goal_distance2d(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a[0:2] - goal_b[0:2], axis=-1)


class ur5GymEnv(gym.Env):
    def __init__(self,
                 camera_attached=False,
                 useIK=True,
                 actionRepeat=1,
                 renders=False,
                 maxSteps=100,
                 simulatedGripper=False,
                 randObjPos=False,
                 task=0, # here target number
                 learning_param=0):

        self.renders = renders
        self.actionRepeat = actionRepeat
        self.useIK = useIK

        # setup pybullet sim:
        if self.renders:
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

        pybullet.setTimeStep(1./240.)
        pybullet.setGravity(0,0,-10)
        pybullet.setRealTimeSimulation(False)
        # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_WIREFRAME,1)
        pybullet.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=60, cameraPitch=-30, cameraTargetPosition=[0.3,0,0])
        
        # gripper info:
        self.gripper_main_control_joint_name = "robotiq_85_left_knuckle_joint"
        self.mimic_joint_name = ["robotiq_85_right_knuckle_joint",
                    "robotiq_85_left_inner_knuckle_joint",
                    "robotiq_85_right_inner_knuckle_joint",
                    "robotiq_85_left_finger_tip_joint",
                    "robotiq_85_right_finger_tip_joint"]
        self.mimic_multiplier = [1, 1, 1, -1, -1]

        # setup robot arm:
        self.end_effector_index = 7
        self.table = pybullet.loadURDF(TABLE_URDF_PATH, [0.5, 0, -0.6300], [0, 0, 0, 1])
        flags = pybullet.URDF_USE_SELF_COLLISION
        self.ur5 = pybullet.loadURDF(ROBOT_URDF_PATH, [0, 0, 0], [0, 0, 0, 1], flags=flags)
        self.num_joints = pybullet.getNumJoints(self.ur5)
        self.control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        self.joint_info = namedtuple("jointInfo", ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity", "controllable"])

        self.joints = AttrDict()
        for i in range(self.num_joints):
            info = pybullet.getJointInfo(self.ur5, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = self.joint_type_list[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = True if jointName in self.control_joints else False
            controllable = True if jointName in self.mimic_joint_name else controllable
            info = self.joint_info(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            print(info)
            if info.type == "REVOLUTE":
                pybullet.setJointMotorControl2(self.ur5, info.id, pybullet.VELOCITY_CONTROL, targetVelocity=0, force=0)
            self.joints[info.name] = info

        # object:
        self.initial_obj_pos = [0.8, 0.1, 0.0] # initial object pos
        self.initial_target_pos = [0.9, -0.2, 0.0] # initial drop-off position
        self.obj = pybullet.loadURDF(CUBE_URDF_PATH, self.initial_obj_pos)

        # self.name = 'ur5GymEnv'
        # self.simulatedGripper = simulatedGripper
        if self.useIK:
            self.action_dim = 4 # IK: 3 coordinates, 1 gripper
        else:
            self.action_dim = 7 # direct control: 6 arm DOF, 1 gripper

        self.maxSteps = maxSteps
        # self.randObjPos = randObjPos
        self.observation = np.array(0)

        self.task = task
        self.learning_param = learning_param
     
        self._action_bound = 1.0 # delta limits
        action_high = np.array([self._action_bound] * self.action_dim)
        self.action_space = spaces.Box(-action_high, action_high, dtype='float32')
        self.reset()
        high = np.array([10]*self.observation.shape[0])
        self.observation_space = spaces.Box(-high, high, dtype='float32')

    def set_joint_angles(self, joint_angles):
        poses = []
        indexes = []
        forces = []

        for i, name in enumerate(self.control_joints):
            joint = self.joints[name]
            poses.append(joint_angles[i])
            indexes.append(joint.id)
            forces.append(joint.maxForce)

        pybullet.setJointMotorControlArray(
            self.ur5, indexes,
            pybullet.POSITION_CONTROL,
            targetPositions=joint_angles,
            targetVelocities=[0]*len(poses),
            positionGains=[0.05]*len(poses),
            forces=forces
        )

    def control_gripper(self, gripper_opening_angle):
        pybullet.setJointMotorControl2(
            self.ur5,
            self.joints[self.gripper_main_control_joint_name].id,
            pybullet.POSITION_CONTROL,
            targetPosition=gripper_opening_angle,
            force=self.joints[self.gripper_main_control_joint_name].maxForce,
            maxVelocity=self.joints[self.gripper_main_control_joint_name].maxVelocity)
        
        for i in range(len(self.mimic_joint_name)):
            joint = self.joints[self.mimic_joint_name[i]]
            pybullet.setJointMotorControl2(
                self.ur5, joint.id, pybullet.POSITION_CONTROL,
                targetPosition=gripper_opening_angle * self.mimic_multiplier[i],
                force=joint.maxForce,
                maxVelocity=joint.maxVelocity)

    def get_joint_angles(self):
        j = pybullet.getJointStates(self.ur5, [1,2,3,4,5,6])
        joints = [i[0] for i in j]
        return joints
    

    def check_collisions(self):
        collisions = pybullet.getContactPoints()
        if len(collisions) > 0:
            # print("[Collision detected!] {}".format(datetime.now()))
            return True
        return False


    def calculate_ik(self, position, orientation):
        quaternion = pybullet.getQuaternionFromEuler(orientation)
        # print(quaternion)
        # quaternion = (0,1,0,1)
        lower_limits = [-math.pi]*6
        upper_limits = [math.pi]*6
        joint_ranges = [2*math.pi]*6
        # rest_poses = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0]
        rest_poses = [(-0.34, -1.57, 1.80, -1.57, -1.57, 0.00)] # rest pose of our ur5 robot

        joint_angles = pybullet.calculateInverseKinematics(
            self.ur5, self.end_effector_index, position, quaternion, 
            jointDamping=[0.01]*6, upperLimits=upper_limits, 
            lowerLimits=lower_limits, jointRanges=joint_ranges, 
            restPoses=rest_poses
        )
        return joint_angles
       
        
    def get_current_pose(self):
        linkstate = pybullet.getLinkState(self.ur5, self.end_effector_index, computeForwardKinematics=True)
        position, orientation = linkstate[0], linkstate[1]
        return (position, orientation)


    def reset(self):
        self.stepCounter = 0
        self.current_task = 0
        self.obj_picked_up = False
        self.ur5_or = [0.0, 1/2*math.pi, 0.0]
        self.target_pos = self.initial_target_pos

        # pybullet.addUserDebugText('X', self.obj_pos, [0,1,0], 1) # display goal
        # if self.randObjPos:
        # self.initial_obj_pos = [0.6+random.random()*0.1, 0.1+random.random()*0.1, 0.0]
        pybullet.resetBasePositionAndOrientation(self.obj, self.initial_obj_pos, [0.,0.,0.,1.0]) # reset object pos

        # reset robot simulation and position:
        joint_angles = (-0.34, -1.57, 1.80, -1.57, -1.57, 0.00) # pi/2 = 1.5707
        self.set_joint_angles(joint_angles)

        # reset gripper:
        self.control_gripper(-0.4) # open

        # step simualator:
        for i in range(100):
            pybullet.stepSimulation()

        # get obs and return:
        self.getExtendedObservation()
        return self.observation
    
    
    def step(self, action):
        action = np.array(action)
        arm_action = action[0:self.action_dim-1].astype(float) # dX, dY, dZ - range: [-1,1]
        gripper_action = action[self.action_dim-1].astype(float) # gripper - range: [-1=closed,1=open]

        # simualted gripping:
        # if self.obj_picked_up and gripper_action < -0.1:
        if self.current_task == 1 or self.current_task == 2:
            # object follows the arm tool tip:
            object_pos = self.get_current_pose()[0] # XYZ, no angles
            pybullet.resetBasePositionAndOrientation(self.obj, object_pos, [0.,0.,0.,1.0])

        # get current position:
        if self.useIK:
            cur_p = self.get_current_pose()
        else:
            cur_p = self.get_joint_angles()

        # add delta position:
        new_p = np.array(cur_p[0]) + arm_action
        
        # actuate:
        if self.useIK:
            joint_angles = self.calculate_ik(new_p, self.ur5_or) # XYZ and angles set to zero
        else:
            joint_angles = new_p

        self.set_joint_angles(joint_angles)

        # operate gripper: close = 0.4, open = -0.4, 2.5 to scale to std=1, nn max value
        gripper_action = np.clip(gripper_action/2.5, -0.4, 0.4)
        self.control_gripper(gripper_action)

        
        # step simualator:
        for i in range(self.actionRepeat):
            pybullet.stepSimulation()
            if self.renders: time.sleep(1./240.)
        
        self.getExtendedObservation()
        reward = self.compute_reward() # call this after getting obs!
        done = self.my_task_done()

        info = {}
        # info = {'is_success': False}
        # if self.terminated == self.task:
        #     info['is_success'] = True

        self.stepCounter += 1

        return self.observation, reward, done, info


    # observations are: arm (tip/tool) position, arm acceleration, ...
    def getExtendedObservation(self):
        # sensor values:
        # js = self.get_joint_angles()
        self.tool_pos,_ = self.get_current_pose() # XYZ, no angles
        self.obj_pos,_ = pybullet.getBasePositionAndOrientation(self.obj)
        self.observation = np.array(np.concatenate((self.tool_pos, self.obj_pos)))

        # we define tasks as: 0-reach obj, 1-lift ojb, 2-move to target, 3-drop obj
        self.goal_pos = self.obj_pos
        if self.current_task == 2: # reach target pos
            self.goal_pos = self.target_pos
            pybullet.addUserDebugText('X', self.goal_pos, [0,1,0], 1) # display goal



    def my_task_done(self):
        # NOTE: need to call compute_reward before this to check termination!
        c = (self.current_task == self.task+1 or self.stepCounter > self.maxSteps)
        return c


    def compute_reward(self):
        reward = np.zeros(1)

        self.target_dist = goal_distance(np.array(self.tool_pos), 
                                        np.array(self.goal_pos))
        # print(self.target_dist)

        # check approach velocity:
        # tv = self.tool.getVelocity()
        # approach_velocity = np.sum(tv)

        # print(approach_velocity)
        # input()

        reward += -self.target_dist * 10

        # task 0,2: reach object/target:
        if self.current_task == 0 or self.current_task == 2:
            if self.target_dist < self.learning_param:# and approach_velocity < 0.05:
                if self.current_task == 0:
                    self.obj_picked_up = True
                    # print('Successful object reach')
                if self.current_task == 2:
                    self.obj_picked_up = False
                    # print('Successful target reach')
                self.current_task += 1
        
        # task 1,3: lift up:
        if self.current_task == 1 or self.current_task == 3:
            if self.tool_pos[2] > 0.3:# and approach_velocity < 0.05:
                # if self.current_task == 1:
                    # print('Successful picked up!')
                # if self.current_task == 3:
                    # print('Successful drop!')
                self.current_task += 1
                # print('Successful!')

        # penalize if it tries to go lower than desk / platform collision:
        # if grip_trans[1] < self.desired_goal[1]-0.08: # lower than position of object!
            # reward[i] += -1
            # print('Penalty: lower than desk!')

        # check collisions:
        if self.check_collisions(): 
            reward += -1
            # print('Collision!')

        # print(target_dist, reward)
        # input()

        return reward