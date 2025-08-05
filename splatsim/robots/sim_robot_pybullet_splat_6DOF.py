import pickle
import threading
import time
from typing import Any, Dict, Optional

import mujoco
import mujoco.viewer
import numpy as np
import zmq
from dm_control import mjcf

from splatsim.robots.robot import Robot

assert mujoco.viewer is mujoco.viewer
import pybullet as p


import urdf_models.models_data as md
from pybullet_planning.interfaces.robots.collision import pairwise_collision
import random



###### Splat related imports ######
import copy

import torch
from gaussian_splatting.scene import Scene
import os
from tqdm import tqdm
from os import makedirs
from gaussian_renderer import render
import torchvision
from gaussian_splatting.utils.general_utils import safe_state
from argparse import ArgumentParser
from gaussian_splatting.arguments import ModelParams, PipelineParams, get_combined_args
from gaussian_splatting.gaussian_renderer import GaussianModel
# from gaussian_splatting.utils_fk import *
# from gaussian_splatting.utils_fk import compute_transformation
from splatsim.utils.utils_fk import compute_transformation
from gaussian_splatting.scene.cameras import Camera
#import RGB2SH
from gaussian_splatting.utils.sh_utils import eval_sh, RGB2SH


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
from gaussian_splatting.utils.system_utils import searchForMaxIteration

import einops
import pickle
import numpy as np
import cv2

import subprocess

#import namedtuple
from collections import namedtuple


####################################

ROBOT_TRANSFORMATION = np.array([
    [0.091979, 0.040193, -0.202176, 0.771204],
    [0.205922, -0.007912, 0.09211, -0.335536],
    [0.009315, -0.221975, -0.039892, 0.520633],
    [0, 0, 0, 1]
])



class ZMQServerThread(threading.Thread):
    def __init__(self, server):
        super().__init__()
        self._server = server

    def run(self):
        self._server.serve()

    def terminate(self):
        self._server.stop()


class ZMQRobotServer:
    """A class representing a ZMQ server for a robot."""

    def __init__(self, robot: Robot, host: str = "127.0.0.1", port: int = 5556):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        """Serve the robot state and commands over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                message = self._socket.recv()
                request = pickle.loads(message)

                # Call the appropriate method based on the request
                method = request.get("method")
                args = request.get("args", {})
                result: Any
                if method == "num_dofs":
                    result = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "get_observations":
                    result = self._robot.get_observations()
                else:
                    result = {"error": "Invalid method"}
                    print(result)
                    raise NotImplementedError(
                        f"Invalid method: {method}, {args, result}"
                    )

                self._socket.send(pickle.dumps(result))
            except zmq.error.Again:
                print("Timeout in ZMQLeaderServer serve")
                # Timeout occurred, check if the stop event is set

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()
        self._context.term()


class PybulletRobotServer:
    def __init__(
        self,
        urdf_path: str = './submodules/pybullet-playground-wrapper/pybullet_playground/urdf/sisbot.urdf',
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
        use_gripper: bool = True,
    ):
        
        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)
        self.pybullet_client = p
        self.urdf_path = urdf_path
        self._num_joints = 7
        self.client_id = self.pybullet_client.connect(p.GUI)
        self.pybullet_client.setAdditionalSearchPath("../gaussian-splatting/pybullet-playground/urdf/pybullet_ur5_gripper/urdf")

        # flags = self.pybullet_client.URDF_USE_IMPLICIT_CYLINDER
        self.dummy_robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, -0.1], useFixedBase=True)


 

        self.use_gripper = use_gripper
        if self.use_gripper:
            self.setup_gripper()
        # else:
        #     self.setup_spatula()
        #     pass

        # self.pybullet_client.resetBasePositionAndOrientation(self.dummy_robot, [0, 0, -0.1], [0, 0, 0, 1])
        self.joint_signs = [1, 1, -1, 1, 1, 1, 1]
        self.offsets = [np.pi/2, 0, 0, 0, 0, 0, 0]

        model_lib = md.model_lib()
        # objectid = self.pybullet_client.loadURDF(model_lib['potato_chip_1'], [0.5, 0.15, 0])
        # x = random.uniform(0.4, 0.6)
        # y = random.uniform(-0.3, 0.3)
        x = random.uniform(0.2, 0.6)
        y = random.uniform(-0.3, 0.3)
        # random euler angles for the orientation of the object
        euler_z =  random.uniform(-np.pi, np.pi)
        # random quaternion for the orientation of the object
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, 0])

        models_lib = md.model_lib()
        self.object_name_list = ['plastic_orange', 'plate']
        self.splat_object_name_list = ['plastic_orange', 'plate']
        self.randomize_object_positions = [True, True]
        self.randomize_object_rotations = [False, False]
        self.rotation_values = [[0, 0], [0, 0], ]
        self.use_fixed_base = [False,  False, True]
        global_scaling_list = [0.95, 1, 0.8]
        self.urdf_object_list = []
        for object_name in range(len(self.object_name_list)):
            if self.object_name_list[object_name] in models_lib.model_name_list:
                object_loaded = self.pybullet_client.loadURDF(models_lib[self.object_name_list[object_name]], [x, y, 0.0], quat, useFixedBase=self.use_fixed_base[object_name])
                self.urdf_object_list.append(object_loaded)
            else:
                x = 0.5
                y = 0.4
                z = 0.005
                euler = [0, 0, 0]
                quat = self.pybullet_client.getQuaternionFromEuler(euler)
                object_path = '/home/nomaan/Desktop/corl24/virtual_objects/' + self.object_name_list[object_name] + '/object.urdf'
                object_loaded = self.pybullet_client.loadURDF(object_path, [x, y, z], quat, globalScaling=1, useFixedBase=self.use_fixed_base[object_name])
                self.urdf_object_list.append(object_loaded)


        #reset the box position
        # self.pybullet_client.resetBasePositionAndOrientation(self.urdf_object_list[0], [0.5, -0.15, 0.1], p.getQuaternionFromEuler([0, 0, 0]))
                
        self.pybullet_client.resetBasePositionAndOrientation(self.urdf_object_list[-1], [0.5, -0.3, 0.07], p.getQuaternionFromEuler([0, 0, np.pi/2]))


        #add gravity
        self.pybullet_client.setGravity(0, 0, -9.81)
        
        #add plane
        import pybullet_data
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = self.pybullet_client.loadURDF("plane.urdf", [0, 0, -0.013])

        #change the friction of the plane
        self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.1))

        #set time step
        self.pybullet_client.setTimeStep(1/240)
        # self.pybullet_client.setRealTimeSimulation(0)

        #reset friction of all the objects
        for i in range(len(self.urdf_object_list)):
            self.pybullet_client.changeDynamics(self.urdf_object_list[i], -1, lateralFriction=30.5)


        # self.robot = self.pybullet_client.loadURDF("../gaussian-splatting/pybullet-playground/urdf/sisbot.urdf", [0, 0, -0.1], useFixedBase=True, flags=flags)
        # #make robot invisible
        # self.pybullet_client.changeVisualShape(self.robot, -1, rgbaColor=[0, 0, 0, 0])
        # self.pybullet_client.resetBasePositionAndOrientation(self.robot, [0, 0, -0.1], [0, 0, 0, 1])
        # #make robot not collide with anything
        # ## disable collision between dummy_robot and everything
        # for i in range(self.pybullet_client.getNumJoints(self.robot)):
        #     self.pybullet_client.setCollisionFilterGroupMask(self.robot, i, 0, 0)

        # for i in range(self.pybullet_client.getNumJoints(self.robot)):
        #     info = self.pybullet_client.getJointInfo(self.robot, i)
        #     joint_id = info[0]
        #     joint_name = info[1].decode("utf-8")
        #     joint_type = info[2]
        #     if joint_name == "ee_fixed_joint":
        #         self.ur5e_ee_id = joint_id




        self.robot_gaussian = GaussianModel(3)
        self.T_object_gaussian = GaussianModel(3)

        #load the gaussian model for the robot
        self.robot_gaussian.load_ply("/home/jennyw2/data/output/robot_iphone/point_cloud/iteration_30000/point_cloud.ply")
        self.gaussians_backup = copy.deepcopy(self.robot_gaussian)

        self.object_gaussians = [GaussianModel(3) for _ in range(len(self.urdf_object_list))]
        for _ in range(len(self.urdf_object_list)):
            self.object_gaussians[_].load_ply("/home/jennyw2/data/output/{}/point_cloud/iteration_7000/point_cloud.ply".format(self.splat_object_name_list[_]))



        # t_gaussians_backup = copy.deepcopy(t_gaussians)
        self.object_gaussians_backup = copy.deepcopy(self.object_gaussians)
        
        self.camera = self.setup_camera()
        self.camera2 = self.setup_camera2()

        parser = ArgumentParser(description="Testing script parameters")
        self.pipeline = PipelineParams(parser)

        bg_color = [1,1,1] 
        self.background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")

        # x = random.uniform(-0.4, 0.5)
        # y_low = 0.6 - np.sqrt(1 - (x-0.1)**2 / (0.5)**2) * 0.1
        # y_high = 0.6 + np.sqrt(1 - (x-0.1)**2 / (0.5)**2) * 0.1
        # y = random.uniform(y_low, y_high)
        # self.pybullet_client.resetBasePositionAndOrientation(self.T_object, [x, y, 0], [0, 0, 0, 1])
        # self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.1))

        import yaml
        with open('/home/jennyw2/code/SplatSim/configs/object_configs/objects.yaml', 'r') as file:
            self.object_config = yaml.safe_load(file)

        self.current_gripper_action = 0

        self.total_count = 301 

        for _ in range(100):
            self.pybullet_client.stepSimulation()


    def num_dofs(self) -> int:
        return 7

    def get_joint_state(self) -> np.ndarray:
        # return self._joint_state
        joint_states = []
        num_joints = self.pybullet_client.getNumJoints(self.dummy_robot)
        for i in range(1, num_joints):
            joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])
        return np.array(joint_states)
    

    def get_joint_state_dummy(self) -> np.ndarray:
        # return self._joint_state
        num_joints = self.pybullet_client.getNumJoints(self.dummy_robot)
        joint_states = []
        for i in range(1, num_joints):
            joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])
        return np.array(joint_states)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )

        for _ in range(100):
            for i in range(1, 7):
                self.pybullet_client.setJointMotorControl2(self.dummy_robot, i, p.POSITION_CONTROL, targetPosition=joint_state[i-1], force=250, maxVelocity=0.2) 

            # for i in range(1, 7):
            #     self.pybullet_client.resetJointState(self.dummy_robot, i, joint_state[i-1])
            if self.use_gripper:
                # move_gripper(0) means close, move_gripper(0.084) means open
                # joint_state[-1] == 1 means close, joint_state[-1] == 0 means open
                print("moving gripper to", (1-joint_state[-1])*0.085)
                self.move_gripper((1-joint_state[-1])*0.085)
                self.current_gripper_action = joint_state[-1]


            self.pybullet_client.stepSimulation()



        # self.total_count += 1
        # target_apple_pos = [0.5129530465428657, 0.10310958170994994, 0.235239846197727]

        # #check if the apple is close to the target position
        # if np.linalg.norm(np.array(self.pybullet_client.getBasePositionAndOrientation(self.urdf_object_list[0])[0]) - np.array(target_apple_pos)) < 0.1:
        #     self.total_count =301
            

        # if self.total_count > 300:
        #     #reset the apple position
        #     x = random.uniform(0.2, 0.6)
        #     y = random.uniform(-0.4, 0.4)
        #     self.pybullet_client.resetBasePositionAndOrientation(self.urdf_object_list[0], [x, y, 0.1], p.getQuaternionFromEuler([0, 0, 0]))
        #     self.pybullet_client.resetBasePositionAndOrientation(self.urdf_object_list[1], [random.uniform(0.2, 0.6), random.uniform(-0.4, 0.4), 0.1], p.getQuaternionFromEuler([0, 0, 0]))

        #     #reset the robot to the initial position
        #     for i in range(len(self.initial_joint_state)):
        #         self.pybullet_client.resetJointState(self.dummy_robot, i, self.initial_joint_state[i])
        #     self.total_count = 0





    def __parse_joint_info__(self):
        numJoints = p.getNumJoints(self.dummy_robot)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):
            info = p.getJointInfo(self.dummy_robot, i)
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
                self.controllable_joints.append(jointID)
                self.pybullet_client.setJointMotorControl2(self.dummy_robot, jointID, self.pybullet_client.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)

    def setup_gripper(self):
            self.__parse_joint_info__()
            self.gripper_range = [0, 0.085]
            
            mimic_parent_name = 'finger_joint'
            mimic_children_names = {'right_outer_knuckle_joint': 1,
                                    'left_inner_knuckle_joint': 1,
                                    'right_inner_knuckle_joint': 1,
                                    'left_inner_finger_joint': -1,
                                    'right_inner_finger_joint': -1}
            # self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)

            self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
            self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if joint.name in mimic_children_names}

            for joint_id, multiplier in self.mimic_child_multiplier.items():
                c = self.pybullet_client.createConstraint(self.dummy_robot, self.mimic_parent_id,
                                    self.dummy_robot, joint_id,
                                    jointType=self.pybullet_client.JOINT_GEAR,
                                    jointAxis=[0, 1, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0])
                self.pybullet_client.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)  # Note: the mysterious `erp` is of EXTREME importance
    
    def get_link_pose(self, body, link):
        result = self.pybullet_client.getLinkState(body, link)
        return result[4], result[5]
    
    def move_gripper(self, open_length):
        # open_length = np.clip(open_length, *self.gripper_range)
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation
        # Control the mimic gripper joint(s)
        p.setJointMotorControl2(self.dummy_robot, self.mimic_parent_id, self.pybullet_client.POSITION_CONTROL, targetPosition=open_angle,
                                force=20, maxVelocity=self.joints[self.mimic_parent_id].maxVelocity)


    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass


    def get_observations(self, image2=True) -> Dict[str, np.ndarray]:
        joint_positions = self.get_joint_state()
        joint_positions_dummy = self.get_joint_state_dummy()
        joint_velocities = np.array([self.pybullet_client.getJointState(self.dummy_robot, i)[1] for i in range(7)])

        dummy_ee_pos, dummy_ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0], self.pybullet_client.getLinkState(self.dummy_robot, 6)[1]
        #get the euler angles from the quaternion
        dummy_ee_euler = self.pybullet_client.getEulerFromQuaternion(dummy_ee_quat)

        #get quaternion from euler angles
        dummy_ee_quat_reconstruct = self.pybullet_client.getQuaternionFromEuler(dummy_ee_euler)

        #print the euler angles and the reconstructed quaternion
        self.current_gripper_state = self.get_current_gripper_state() / 0.8

        
        #combine the position and euler angles and self.current_gripper_state to get the state
        state = np.concatenate([dummy_ee_pos, dummy_ee_euler, [self.current_gripper_state]])
        action = np.concatenate([dummy_ee_pos, dummy_ee_euler, [self.current_gripper_action]])

        


        # Target object position and orientation

        observations = {"joint_positions": joint_positions[:7],
            "all_joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_positions_dummy": joint_positions_dummy,
            "state": state,
            "action": action,}
        
        observations["gripper_position"] = [self.current_gripper_state]
        
        for i in range(len(self.urdf_object_list)):
            object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.urdf_object_list[i])
            observations[self.splat_object_name_list[i]+ "_position"] = object_pos
            observations[self.splat_object_name_list[i] + "_orientation"] = object_quat


        image = self.get_image_observation(data=observations, )
        if image2:
            image2 = self.get_image_observation(data=observations, camera='camera2')
        else:
            image2 = None

        cv2.imshow('camera1', cv2.cvtColor(cv2.resize(image,(1200, 900) ), cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)

        observations["base_rgb"] = image
        observations["wrist_rgb"] = image2


        return observations
    
    def get_image_observation(self, data, camera='camera1') -> Dict[str, np.ndarray]:
        if camera == 'camera1':
            cur_joint = self.get_joint_state_dummy()
            cur_joint = [0] + cur_joint.tolist()
            cur_joint = np.array(cur_joint)

            #add 0 at front of cur_joint
            cur_joint = [0] + cur_joint.tolist()
            # cur_joint = [0, 0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
            cur_joint = np.array(cur_joint)

            cur_object_position_list = []
            cur_object_rotation_list = []

            for object_name in self.splat_object_name_list:
                cur_object_position = np.array(data[object_name + '_position'])
                cur_object_position_list.append(torch.from_numpy(cur_object_position).to(device='cuda').float())
                cur_object_rotation = np.array(data[object_name + '_orientation'])
                # cur_object_rotation = np.roll(cur_object_rotation, 1)
                cur_object_rotation_list.append(torch.from_numpy(cur_object_rotation).to(device='cuda').float())

            
            
            transformations_list = self.get_transfomration_list(cur_joint)
            segmented_list, xyz = get_segmented_indices(self.gaussians_backup)

            xyz, rot, opacity, shs_featrest, shs_dc  = transform_means(self.gaussians_backup, xyz, segmented_list, transformations_list)
            # xyz_cube, rot_cube, opacity_cube, scales_cube, shs_dc_cube, sh_rest_cube = place_object(gaussians_backup, pos=torch.from_numpy(cur_object).to(device='cuda').float(), rotation=torch.from_numpy(curr_rotation).to(device='cuda').float())
            # xyz_obj, rot_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj = transform_object(t_gaussians, pos=cur_position, quat=cur_rotation)
            xyz_obj_list = []
            rot_obj_list = []
            opacity_obj_list = []
            scales_obj_list = []
            features_dc_obj_list = []
            features_rest_obj_list = []


            for i in range(len(self.urdf_object_list)):
                xyz_obj, rot_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj = transform_object(self.object_gaussians_backup[i], object_config=self.object_config[self.splat_object_name_list[i]], pos=cur_object_position_list[i], quat=cur_object_rotation_list[i])
                xyz_obj_list.append(xyz_obj)
                rot_obj_list.append(rot_obj)
                opacity_obj_list.append(opacity_obj)
                scales_obj_list.append(scales_obj)
                features_dc_obj_list.append(features_dc_obj)
                features_rest_obj_list.append(features_rest_obj)
            
            with torch.no_grad():
                # gaussians.active_sh_degree = 0
                self.robot_gaussian._xyz = torch.cat([xyz] + xyz_obj_list, dim=0)
                self.robot_gaussian._rotation = torch.cat([rot] + rot_obj_list, dim=0)
                self.robot_gaussian._opacity = torch.cat([opacity] + opacity_obj_list, dim=0)
                self.robot_gaussian._features_rest = torch.cat([shs_featrest] + features_rest_obj_list, dim=0)
                self.robot_gaussian._features_dc = torch.cat([shs_dc] +features_dc_obj_list, dim=0)
                self.robot_gaussian._scaling = torch.cat([self.gaussians_backup._scaling] +scales_obj_list, dim=0)
        
        if camera == 'camera1':
            rendering = render(self.camera, self.robot_gaussian, self.pipeline, self.background)["render"]
        else:
            rendering = render(self.camera2, self.robot_gaussian, self.pipeline, self.background)["render"]

        # t_gaussians = copy.deepcopy(t_gaussians_backup)
        self.object_gaussians = copy.deepcopy(self.object_gaussians_backup)
    

        # convert into numpy
        rendering = rendering.detach().cpu().numpy()

        # convert to hxwxc from cxhxw
        rendering = np.transpose(rendering, (1, 2, 0))
        
        #convert to 0-255
        rendering = (rendering * 255).astype(np.uint8)

       

        #show the image
        # cv2.imshow(camera, cv2.cvtColor(cv2.resize(rendering,(1200, 900) ), cv2.COLOR_BGR2RGB))
        # cv2.waitKey(1)

        
        #save the image
        return rendering
        

    def setup_camera2(self):
        uid = 0
        colmap_id = 1
        R = torch.from_numpy(np.array([[-0.98784567,  0.00125165,  0.15543282],
                        [0.1153457,   0.67620402,  0.72762868],
                        [ -0.10419356,  0.73671335, -0.66812959]])).float().numpy()
        
        T = torch.Tensor([ 1.04674738, -0.96049824,  2.03845016] ).float().numpy()



        
        # T = torch.Tensor([ 0.09347542+0.5, -0.74648806+0.6,  5.57444971+0.2] ).float()


        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        gt_mask_alpha = None

        image_width = 640
        image_height = 480  
        image_name = 'test'
        image = torch.zeros((3, image_height, image_width)).float()

        #order is  colmap_id, R, T, FoVx, FoVy, image, gt_alpha_mask, image_name, uid,

        camera = Camera(colmap_id, R, T, FoVx, FoVy, image, gt_mask_alpha, image_name, uid)

        return camera
    
    def setup_camera(self):
        uid = 0
        colmap_id = 1
        R = torch.from_numpy(np.array([[1.27679278e-01, -4.36057591e-01,  8.90815233e-01],
                        [6.15525303e-02,  8.99918716e-01,  4.31691546e-01],
                        [ -9.89903676e-01, -2.86133428e-04,  1.41741420e-01]])).float().numpy()
        
        T = torch.Tensor([ -1.88933526, -0.6446558,   2.98276143] ).float().numpy()



        
        # T = torch.Tensor([ 0.09347542+0.5, -0.74648806+0.6,  5.57444971+0.2] ).float()


        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        gt_mask_alpha = None

        image_width = 640*2
        image_height = 480*2
        image_name = 'test'
        image = torch.zeros((3, image_height, image_width)).float()

        #order is  colmap_id, R, T, FoVx, FoVy, image, gt_alpha_mask, image_name, uid,

# self, resolution, colmap_id, R, T, FoVx, FoVy, depth_params, image, invdepthmap,
#                  image_name, uid,
#                  trans=np.array([0.0, 0.0, 0.0]), scale=1.0, data_device = "cuda",
#                  train_test_exp = False, is_test_dataset = False, is_test_view = False
#                  ):
        camera = Camera(colmap_id, R, T, FoVx, FoVy, image, gt_mask_alpha, image_name, uid)

        return camera


    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        initial_joint_state = [0, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, 0.0, 0.7999999999999996, 0.0, -0.8000070728762431, 0.0, 0.7999947291384548, 0.799996381456464, 0.0, -0.799988452159267, 0.0, 0.7999926186486127]
        self.initial_joint_state = initial_joint_state
        joint_signs = [1, 1, 1, 1, 1, 1]
        for i in range(len(self.initial_joint_state)):
            self.pybullet_client.resetJointState(self.dummy_robot, i, initial_joint_state[i])

        self.initial_joints = []
        for joint_index in range(19):
            joint_info = self.pybullet_client.getJointInfo(self.dummy_robot, joint_index)
            joint_name = joint_info[1].decode("utf-8")  
            link_state = self.pybullet_client.getLinkState(self.dummy_robot, joint_index, computeForwardKinematics=True)
            self.initial_joints.append(link_state)
        
        #get end effector position and orientation
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0], self.pybullet_client.getLinkState(self.dummy_robot, 6)[1]
        self.iniital_ee_quat = ee_quat
        

        while True:
            self.pybullet_client.stepSimulation()
            pass
            # self.get_observations()
            # for i in range(len(self.initial_joint_state)):
            #     self.pybullet_client.resetJointState(self.dummy_robot, i, initial_joint_state[i]) 
            # time.sleep(1/240)

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None: 
        self.stop()


    def get_current_gripper_state(self):
        return self.pybullet_client.getJointState(self.dummy_robot, self.mimic_parent_id)[0]


    def get_transfomration_list(self, new_joint_poses):  
        # for i in range(1, len(self.initial_joint_state)):
        #     self.pybullet_client.resetJointState(self.dummy_robot, i-1, new_joint_poses[i-1])
        
        new_joints = []
        for joint_index in range(19):
            joint_info = p.getJointInfo(self.dummy_robot, joint_index)
            joint_name = joint_info[1].decode("utf-8") 
            link_state = p.getLinkState(self.dummy_robot, joint_index, computeForwardKinematics=True)
            new_joints.append(link_state)
            
            
        transformations_list = []
        for joint_index in range(19):
            input_1 = (self.initial_joints[joint_index][0][0], self.initial_joints[joint_index][0][1], self.initial_joints[joint_index][0][2], np.array(self.initial_joints[joint_index][1]))
            input_2 = (new_joints[joint_index][0][0], new_joints[joint_index][0][1], new_joints[joint_index][0][2], np.array(new_joints[joint_index][1]))
            r_rel, t = compute_transformation(input_1, input_2)
            r_rel = torch.from_numpy(r_rel).to(device='cuda').float()
            t = torch.from_numpy(t).to(device='cuda').float()
            
            transformations_list.append((r_rel, t))

        return transformations_list




def transform_means(pc, xyz, segmented_list, transformations_list):

    #takes in already transformed means and transforms other properties

    # Trans = torch.tensor([[0.144820347428, 0.019638715312, -0.143120646477, -0.240426957607],
    #                         [0.141510024667, 0.021471565589, 0.146136879921, 0.408296585083],
    #                         [0.029053352773, -0.202473223209, 0.001615444897, 0.492784976959],
    #                         [0, 0, 0, 1]]).to(device=xyz.device) # shape (4, 4)
    
    ### fill the transformation matrix here
    #     #    -0.171274 0.028960 -0.021428 0.532716
# 0.023970 0.013890 -0.172815 -0.322159
# -0.026895 -0.172049 -0.017558 0.371520
# 0.000000 0.000000 0.000000 1.000000

    Trans = torch.tensor([[-0.171274, 0.028960, -0.021428, 0.532716],
                            [0.023970, 0.013890, -0.172815, -0.322159],
                            [-0.026895, -0.172049, -0.017558, 0.371520],
                            [0, 0, 0, 1]]).to(device=xyz.device) # shape

    scale_robot = torch.pow(torch.linalg.det(Trans[:3, :3]), 1/3)
    rotation_matrix = Trans[:3, :3] / scale_robot
    translation = Trans[:3, 3]
    inv_transformation_matrix = torch.inverse(Trans)
    inv_rotation_matrix = inv_transformation_matrix[:3, :3] 
    inv_translation = inv_transformation_matrix[:3, 3]
    
    # rot = copy.deepcopy(pc.get_rotation)
    rot = pc.get_rotation
    opacity = pc.get_opacity_raw
    # shs = pc.get_features
    with torch.no_grad():
        # shs_dc = pc._features_dc
        # shs_featrest = pc._features_rest
        shs_dc = copy.deepcopy(pc._features_dc)
        shs_featrest = copy.deepcopy(pc._features_rest)

    for joint_index in range(7):
        r_rel, t = transformations_list[joint_index]
        segment = segmented_list[joint_index]
        transformed_segment = torch.matmul(r_rel, xyz[segment].T).T + t
        xyz[segment] = transformed_segment
        
        # Defining rotation matrix for the covariance
        rot_rotation_matrix = (inv_rotation_matrix*scale_robot) @ r_rel @ rotation_matrix
        
        tranformed_rot = rot[segment]  
        tranformed_rot = o3.quaternion_to_matrix(tranformed_rot) ### --> zyx    
        
        transformed_rot = rot_rotation_matrix  @ tranformed_rot # shape (N, 3, 3)
        
        transformed_rot = o3.matrix_to_quaternion(transformed_rot)
        
        rot[segment] = transformed_rot

        #transform the shs features
        shs_feat = shs_featrest[segment]
        shs_dc_segment = shs_dc[segment]
        shs_feat = transform_shs(shs_feat, rot_rotation_matrix)
        # print('shs_feat : ', shs_feat.shape)
        with torch.no_grad():
            shs_featrest[segment] = shs_feat
        # shs_dc[segment] = shs_dc_segment
        # shs_featrest[segment] = torch.zeros_like(shs_featrest[segment])
    # cnt = 7
    # for joint_index in [8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 12]:
    #     r_rel, t = transformations_list[joint_index]
    #     segment = segmented_list[cnt]
    #     transformed_segment = torch.matmul(r_rel, xyz[segment].T).T + t
    #     xyz[segment] = transformed_segment
        
    #     # Defining rotation matrix for the covariance
    #     rot_rotation_matrix = (inv_rotation_matrix*scale_robot) @ r_rel @ rotation_matrix
        
    #     tranformed_rot = rot[segment]  
    #     tranformed_rot = o3.quaternion_to_matrix(tranformed_rot) ### --> zyx    
        
    #     transformed_rot = rot_rotation_matrix  @ tranformed_rot # shape (N, 3, 3)
        
    #     transformed_rot = o3.matrix_to_quaternion(transformed_rot)
        
    #     rot[segment] = transformed_rot

    #     #transform the shs features
    #     shs_feat = shs_featrest[segment]
    #     shs_dc_segment = shs_dc[segment]
    #     shs_feat = transform_shs(shs_feat, rot_rotation_matrix)
    #     # print('shs_feat : ', shs_feat.shape)
    #     with torch.no_grad():
    #         shs_featrest[segment] = shs_feat
    #     # shs_dc[segment] = shs_dc_segment
    #     # shs_featrest[segment] = torch.zeros_like(shs_featrest[segment])
    #     cnt += 1
    

        
    #transform_back
    xyz = torch.matmul(inv_rotation_matrix, xyz.T).T + inv_translation
    
        
    return xyz, rot, opacity, shs_featrest, shs_dc



def transform_object(pc, object_config, pos = torch.tensor([0, 0.3, 0.09]).to(device='cuda').float(), quat = torch.tensor([1, 0, 0, 0]).to(device='cuda').float()):
            
#   Trans = #     -0.171274 0.028960 -0.021428 0.532716
# 0.023970 0.013890 -0.172815 -0.322159
# -0.026895 -0.172049 -0.017558 0.371520
# 0.000000 0.000000 0.000000 1.000000
    

    
#     0.399489 0.125515 0.082090 0.059245
# -0.122380 0.407827 -0.028004 0.006866
# -0.086694 0.002674 0.417805 -0.122231
# 0.000000 0.000000 0.000000 1.000000
    
    Trans_canonical = torch.from_numpy(np.array(object_config['transformation']['matrix'])).to(device=pc.get_xyz.device).float() # shape (4, 4)

    
    
    rotation_matrix_c = Trans_canonical[:3, :3]
    translation_c = Trans_canonical[:3, 3]
    scale_obj = torch.pow(torch.linalg.det(rotation_matrix_c), 1/3)

    Trans_robot = torch.from_numpy(ROBOT_TRANSFORMATION).to(device=pc.get_xyz.device).float() # shape (4, 4)
    
    # Trans_robot = torch.tensor([[-0.171274, 0.028960, -0.021428, 0.532716],
    #                         [0.023970, 0.013890, -0.172815, -0.322159],
    #                         [-0.026895, -0.172049, -0.017558, 0.371520],
    #                         [0, 0, 0, 1]]).to(device=pc.get_xyz.device) # shape (4, 4)

    
    rotation_matrix_r = Trans_robot[:3, :3]
    scale_r = torch.pow(torch.linalg.det(rotation_matrix_r), 1/3)

    translation_r = Trans_robot[:3, 3]

    inv_transformation_r = torch.inverse(Trans_robot)
    inv_rotation_matrix_r = inv_transformation_r[:3, :3]
    inv_translation_r = inv_transformation_r[:3, 3]
    inv_scale = torch.pow(torch.linalg.det(inv_rotation_matrix_r), 1/3)

    # print('scale_obj : ', scale_obj)
    # print('inv_scale : ', inv_scale)
    
    xyz_obj = pc.get_xyz
    rotation_obj = pc.get_rotation
    opacity_obj = pc.get_opacity_raw
    scales_obj = pc.get_scaling
    scales_obj = scales_obj * scale_obj * inv_scale 
    # scales_obj = scales_obj * 1

    scales_obj = torch.log(scales_obj)

    with torch.no_grad():
        features_dc_obj = copy.deepcopy(pc._features_dc)
        features_rest_obj = copy.deepcopy(pc._features_rest)
    
    #transform the object to the canonical frame
    xyz_obj = torch.matmul(rotation_matrix_c, xyz_obj.T).T + translation_c
    
    
    rot_rotation_matrix = ( inv_rotation_matrix_r/inv_scale) @ o3.quaternion_to_matrix(quat)  @  (rotation_matrix_c/scale_obj)
    rotation_obj_matrix = o3.quaternion_to_matrix(rotation_obj)
    rotation_obj_matrix = rot_rotation_matrix @ rotation_obj_matrix 
    rotation_obj = o3.matrix_to_quaternion(rotation_obj_matrix) 
    
    
    # aabb = ((-0.10300000149011612, -0.17799999701976776, -0.0030000000000000027), (0.10300000149011612, 0.028000000372529033, 0.022999999552965167))
    aabb = object_config['aabb']['bounding_box']
    #segment according to axis aligned bounding box
    segmented_indices = ((xyz_obj[:, 0] > aabb[0][0]) & (xyz_obj[:, 0] < aabb[1][0]) & (xyz_obj[:, 1] > aabb[0][1] ) & (xyz_obj[:, 1] < aabb[1][1]) & (xyz_obj[:, 2] > aabb[0][2] ) & (xyz_obj[:, 2] < aabb[1][2]))
    

    #offset the object by the position and rotation
    xyz_obj = torch.matmul(o3.quaternion_to_matrix(quat), xyz_obj.T).T + pos
    # xyz_obj = xyz_obj + pos
    
    xyz_obj = torch.matmul(inv_rotation_matrix_r, xyz_obj.T).T + inv_translation_r

    xyz_obj = xyz_obj[segmented_indices]
    rotation_obj = rotation_obj[segmented_indices]
    opacity_obj = opacity_obj[segmented_indices]
    scales_obj = scales_obj[segmented_indices]
    # cov3D_obj = cov3D_obj[segmented_indices]
    features_dc_obj = features_dc_obj[segmented_indices]
    features_rest_obj = features_rest_obj[segmented_indices]
    features_rest_obj= transform_shs( features_rest_obj, rot_rotation_matrix)
    # features_rest_obj = torch.zeros_like(features_rest_obj)
    
    return xyz_obj, rotation_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj


def transform_shs(shs_feat, rotation_matrix):

    ## rotate shs
    P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]) # switch axes: yzx -> xyz
    permuted_rotation_matrix = np.linalg.inv(P) @ rotation_matrix.cpu().numpy() @ P
    rot_angles = o3._rotation.matrix_to_angles(torch.from_numpy(permuted_rotation_matrix).to(device=shs_feat.device).float())
    
    rot_angles = [a.detach().cpu() for a in rot_angles]  # convert to numpy for wigner_D computation

    # Construction coefficient
    D_1 = o3.wigner_D(1, rot_angles[0], - rot_angles[1], rot_angles[2])
    D_2 = o3.wigner_D(2, rot_angles[0], - rot_angles[1], rot_angles[2])
    D_3 = o3.wigner_D(3, rot_angles[0], - rot_angles[1], rot_angles[2])

    D_1 = D_1.to(device=shs_feat.device)
    D_2 = D_2.to(device=shs_feat.device)
    D_3 = D_3.to(device=shs_feat.device)

    #rotation of the shs features
    one_degree_shs = shs_feat[:, 0:3]
    one_degree_shs = einops.rearrange(one_degree_shs, 'n shs_num rgb -> n rgb shs_num')
    one_degree_shs = einsum(
            D_1,
            one_degree_shs,
            "... i j, ... j -> ... i",
        )
    one_degree_shs = einops.rearrange(one_degree_shs, 'n rgb shs_num -> n shs_num rgb')
    shs_feat[:, 0:3] = one_degree_shs

    two_degree_shs = shs_feat[:, 3:8]
    two_degree_shs = einops.rearrange(two_degree_shs, 'n shs_num rgb -> n rgb shs_num')
    two_degree_shs = einsum(
            D_2,
            two_degree_shs,
            "... i j, ... j -> ... i",
        )
    two_degree_shs = einops.rearrange(two_degree_shs, 'n rgb shs_num -> n shs_num rgb')
    shs_feat[:, 3:8] = two_degree_shs

    three_degree_shs = shs_feat[:, 8:15]
    three_degree_shs = einops.rearrange(three_degree_shs, 'n shs_num rgb -> n rgb shs_num')
    three_degree_shs = einsum(
            D_3,
            three_degree_shs,
            "... i j, ... j -> ... i",
        )
    three_degree_shs = einops.rearrange(three_degree_shs, 'n rgb shs_num -> n shs_num rgb')
    shs_feat[:, 8:15] = three_degree_shs

    return shs_feat


        

def get_segmented_indices(pc):
    # empty torch cache
    torch.cuda.empty_cache()
    means3D = pc.get_xyz # 3D means shape (N, 3)
    
    # Defining a cube in Gaussian space to segment out the robot
    xyz = pc.get_xyz # shape (N, 3)

  
    #find indices where x is between -2 and 2, y is between -2 and 2, z is between -2 and 2
    # x_indices = ((xyz[:, 0] > -2) & (xyz[:, 0] < 20) & (xyz[:, 1] > -1.3) & (xyz[:, 1] < 1.3) & (xyz[:, 2] > -2.4) & (xyz[:, 2] < 2.4))
    
#     -0.171274 0.028960 -0.021428 0.532716
# 0.023970 0.013890 -0.172815 -0.322159
# -0.026895 -0.172049 -0.017558 0.371520
# 0.000000 0.000000 0.000000 1.000000


    Trans = torch.tensor([[-0.171274, 0.028960, -0.021428, 0.532716],
                            [0.023970, 0.013890, -0.172815, -0.322159],
                            [-0.026895, -0.172049, -0.017558, 0.371520],
                            [0, 0, 0, 1]]).to(device=means3D.device) # shape (4, 4)
    
    #define a transformation matrix according to 90 degree rotation about z axis
    temp_matrix = torch.tensor([[0, -1, 0, 0],
                                [1, 0, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]]).to(device=means3D.device).float() # shape (4, 4)
    
    Trans = torch.matmul(temp_matrix, Trans)
    
    R = Trans[:3, :3]
    translation = Trans[:3, 3]
    
    
    points = copy.deepcopy(means3D)
    #transform the points to the new frame
    points = torch.matmul(R, points.T).T + translation
    
    
    centers = torch.tensor([[0, 0, 0.0213], [-0.0663-0.00785, 0 , .0892], [-0.0743, 0, .5142], [-0.0743 +0.0174 -0.00785, 0.39225, .5142], [-0.0743 +0.0174-0.0531, 0.04165+0.39225+0.00785, .5142], [-0.0743 +0.0174-0.0531, 0.04165+0.39225+0.0531 , .5142 -0.04165-0.00785]]) # length = 6
    centers = centers.to(device=xyz.device)
    segmented_points = []
    
    # Box condition
    box_condition = ((points[:, 0] > -0.25) * (points[:, 0] < 0.2) * (points[:, 1] > -0.3) * (points[:, 1] < 0.6) * (points[:, 2] > 0.0) * (points[:, 2] < 0.6))
    
    
    # Segment Base
    condition = torch.where((points[:, 2] < centers[0, 2]) * box_condition)[0]
    segmented_points.append(condition)
    
    # Segment Link 1
    condition = torch.where(((points[:, 2] > centers[0, 2])*(points[:, 0] > centers[1, 0])* (points[:, 2] < 0.2)) * box_condition
                    )[0]
    segmented_points.append(condition)
    
    # Segment Link 2
    condition1 = torch.where(((points[:,0] < centers[1,0]) * (points[:,2] > centers[0,2]) * (points[:,2] < 0.3) * (points[:,1] < 0.3))*box_condition)[0]
    condition2 = torch.where(((points[:,0] < centers[2,0]) * (points[:, 2] >= 0.3) * (points[:, 1] < 0.1))*box_condition)[0]
    condition = torch.cat([condition1, condition2])
    segmented_points.append(condition)
    
    # Segment Link 3
    condition1 = torch.where(((points[:,0] > centers[2,0]) * (points[:,1] > (centers[2,1] - 0.1)) * (points[:,1] < 0.3) * (points[:,2] > 0.4))*box_condition)[0]
    condition2 = torch.where(((points[:, 0] > centers[3, 0]) * (points[:, 1] >= 0.3) * (points[:, 2] > 0.4))*box_condition)[0]
    condition = torch.cat([condition1, condition2])
    
    segmented_points.append(condition)
    
    # Segment Link 4
    condition = torch.where(((points[:, 0] < centers[3, 0]) * (points[:, 1] > 0.25) * (points[:,1] < centers[4, 1]) * (points[:,2] > 0.3))*box_condition)[0]

    segmented_points.append(condition)
    
    # Segment Link 5
    condition = torch.where(((points[:, 0] < centers[3, 0]) * (points[:,1] > centers[4, 1]) * (points[:, 2] > centers[5, 2]))*box_condition)[0]
    segmented_points.append(condition)

    # Segment Link 6
    # condition = torch.where(((points[:, 0] < centers[3, 0]) * (points[:,1] > centers[4, 1]) * (points[:, 2] < centers[5, 2]))*box_condition)[0]
    condition = torch.where(((points[:, 0] < centers[3, 0]+0.2) * (points[:,1] > centers[4, 1]) * (points[:, 2] < centers[5, 2]) * (points[:, 2] > 0.4))*box_condition)[0]
    segmented_points.append(condition)


    #undo the temporary transformation
    points = torch.matmul(torch.inverse(temp_matrix)[:3, :3], points.T).T + torch.inverse(temp_matrix)[:3, 3]


    #load labels.npy
    # labels = np.load('/home/nomaan/Desktop/corl24/ocean_backup/gaussian-splatting/labels.npy')
    # labels = torch.from_numpy(labels).to(device=xyz.device).long()

    # condition = (points[:, 2] > 0.2) & (points[:, 2] < 0.5) & (points[:, 1] < 0.2) & (points[:, 1] > 0.) & (points[:, 0] < 0.6) & (points[:, 0] > -0.)

    condition = (points[:, 2] > 0.2) & (points[:, 2] < 0.4) & (points[:, 1] < 0.2) & (points[:, 1] > 0.) & (points[:, 0] < 0.6) & (points[:, 0] > -0.)
    condition = torch.where(condition)[0]

    segmented_points.append(condition)


    # segmented_points.append(condition[labels== 1])
    # segmented_points.append(condition[labels== 2])
    # segmented_points.append(condition[labels== 3])
    # segmented_points.append(condition[labels== 4])
    # segmented_points.append(condition[labels== 5])
    # segmented_points.append(condition[labels== 6])
    # segmented_points.append(condition[labels== 7])
    # segmented_points.append(condition[labels== 8])
    # segmented_points.append(condition[labels== 9])
    # segmented_points.append(condition[labels== 10])
    # segmented_points.append(condition[labels== 11])


    
    return segmented_points, points
