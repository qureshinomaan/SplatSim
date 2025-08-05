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
from gaussian_splatting.gaussian_renderer import render
import torchvision
from gaussian_splatting.utils.general_utils import safe_state
from argparse import ArgumentParser
from gaussian_splatting.arguments import ModelParams, PipelineParams, get_combined_args
from gaussian_splatting.gaussian_renderer import GaussianModel
from splatsim.utils.utils_fk import *
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
        # urdf_path: str = '../gaussian-splatting/pybullet-playground/urdf/sisbot.urdf',
        urdf_path: str = '../gaussian-splatting/pybullet-playground/urdf/sisbot.urdf',
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
        self.pybullet_client.connect(p.GUI)
        self.pybullet_client.setAdditionalSearchPath("../gaussian-splatting/pybullet-playground/urdf/pybullet_ur5_gripper/urdf")

        flags = self.pybullet_client.URDF_USE_IMPLICIT_CYLINDER
        self.robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, -0.1], useFixedBase=True, flags=flags)
        self.dummy_robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, -0.1], useFixedBase=True, flags=flags)


        ## disable collision between dummy_robot and everything
        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            self.pybullet_client.setCollisionFilterGroupMask(self.robot, i, 0, 0)



        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            info = self.pybullet_client.getJointInfo(self.robot, i)
            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            if joint_name == "ee_fixed_joint":
                self.ur5e_ee_id = joint_id

        self.use_gripper = use_gripper
        if self.use_gripper:
            self.setup_gripper()
        # else:
        #     self.setup_spatula()
        #     pass

        # self.pybullet_client.resetBasePositionAndOrientation(self.robot, [0, 0, -0.1], [0, 0, 0, 1])
        # self.pybullet_client.resetBasePositionAndOrientation(self.dummy_robot, [0, 0, -0.1], [0, 0, 0, 1])
        self.joint_signs = [1, 1, -1, 1, 1, 1, 1]
        self.offsets = [np.pi/2, 0, 0, 0, 0, 0, 0]

        model_lib = md.model_lib()
        # objectid = self.pybullet_client.loadURDF(model_lib['potato_chip_1'], [0.5, 0.15, 0])

         # x = random.uniform(0.4, 0.6)
        # y = random.uniform(-0.3, 0.3)
        x = 0.5 + random.uniform(-0.03, 0.03)
        y = -0.03 + random.uniform(-0.01, 0.01)
        # random euler angles for the orientation of the object
        euler_z =  random.uniform(-np.pi, np.pi)
        # random quaternion for the orientation of the object
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])

        T_object = self.pybullet_client.loadURDF('../gaussian-splatting/pybullet-playground/urdf/T_object/urdf_T.urdf', [x, y, 0], quat, globalScaling=0.93)
        self.T_object = T_object

        # load another object without collision
        self.target_location = [0.5+0.0, 0.-0.03, -0.01395]
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, np.pi+np.pi/4])
        T_object2 = self.pybullet_client.loadURDF('../gaussian-splatting/pybullet-playground/urdf/T_object/urdf_T.urdf', self.target_location, quat, globalScaling=0.93, useFixedBase=True)
        self.T_object2 = T_object2
        #disable collision for T_object2
        collisionFilterGroup = 0
        collisionFilterMask = 0
        p.setCollisionFilterGroupMask(T_object2, -1, collisionFilterGroup, collisionFilterMask)



        #make the T_object2 transparent and red
        self.pybullet_client.changeVisualShape(T_object2, -1, rgbaColor=[1, 0, 0, 0.5])

        #make robot transparent
        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            self.pybullet_client.changeVisualShape(self.robot, i, rgbaColor=[0, 0, 0, 0.0])

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




        self.robot_gaussian = GaussianModel(3)
        self.T_object_gaussian = GaussianModel(3)

        #load the gaussian model for the robot
        self.robot_gaussian.load_ply("/home/nomaan/Desktop/corl24/ocean_backup/gaussian-splatting/output/new_ur/point_cloud/iteration_100000/point_cloud.ply")
        self.gaussians_backup = copy.deepcopy(self.robot_gaussian)
        self.T_object_gaussian.load_ply("/home/nomaan/Desktop/corl24/ocean_backup/gaussian-splatting/output/new_ur/point_cloud/iteration_100000/point_cloud.ply")
        
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


    def num_dofs(self) -> int:
        return 7

    def get_joint_state(self) -> np.ndarray:
        # return self._joint_state
        joint_states = []
        for i in range(1, 8):
            joint_states.append(self.pybullet_client.getJointState(self.robot, i)[0])
        return np.array(joint_states)
    

    def get_joint_state_dummy(self) -> np.ndarray:
        # return self._joint_state
        joint_states = []
        for i in range(1, 8):
            joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])
        return np.array(joint_states)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )


        for i in range(1, 7):
            # self.pybullet_client.resetJointState(self.robot, i, joint_state[i-1])
            self.pybullet_client.setJointMotorControl2(self.robot, i, p.POSITION_CONTROL, targetPosition=joint_state[i-1], force=100) 

    
        #get end effector position and orientation
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.robot, 7)[0], self.pybullet_client.getLinkState(self.robot, 7)[1]

        #get current end effector position and orientation of the dummy robot
        ee_pos_dummy, ee_quat_dummy = self.pybullet_client.getLinkState(self.dummy_robot, 7)[0], self.pybullet_client.getLinkState(self.dummy_robot, 7)[1]

        dummy_ee_pos = [ee_pos[0], ee_pos[1], 0.265]
        

        # get ee command
        self.ee_command = np.array([dummy_ee_pos[0] - ee_pos_dummy[0], dummy_ee_pos[1] - ee_pos_dummy[1]])

        dummy_ee_quat = ee_quat
        dummy_joint_pos = self.pybullet_client.calculateInverseKinematics(self.dummy_robot, 6, dummy_ee_pos, self.iniital_ee_quat, 
            residualThreshold=0.00001, maxNumIterations=1000, 
            lowerLimits=[self.initial_joint_state[k] - np.pi/2 for k in range(6)],
            upperLimits=[self.initial_joint_state[k] + np.pi/2 for k in range(6)],
            jointRanges=[12.566, 12.566, 6.282, 12.566, 12.566, 12.566],
            restPoses=[0* np.pi, -0.5* np.pi, 0.5* np.pi, -0.5* np.pi, -0.5* np.pi, 0]
            )
        dummy_joint_pos = list(dummy_joint_pos)
        
        for i in range(1, 7):
            # self.pybullet_client.resetJointState(self.dummy_robot, i, dummy_joint_pos[i-1])
            self.pybullet_client.setJointMotorControl2(self.dummy_robot, i, p.POSITION_CONTROL, targetPosition=dummy_joint_pos[i-1], force=100)
            self.pybullet_client.stepSimulation()


        if self.use_gripper:
            self.move_gripper(0.0)

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
                                force=self.joints[self.mimic_parent_id].maxForce, maxVelocity=self.joints[self.mimic_parent_id].maxVelocity)


    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self, image2=False) -> Dict[str, np.ndarray]:
        joint_positions = self.get_joint_state()
        joint_positions_dummy = self.get_joint_state_dummy()
        joint_velocities = np.array([self.pybullet_client.getJointState(self.dummy_robot, i)[1] for i in range(7)])

        dummy_ee_pos, dummy_ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 7)[0], self.pybullet_client.getLinkState(self.dummy_robot, 7)[1]
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.robot, 7)[0], self.pybullet_client.getLinkState(self.robot, 7)[1]

        gripper_pos = self.pybullet_client.getJointState(self.dummy_robot, 7)[0]

        # T_object position and orientation
        object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.T_object)

        # Target object position and orientation
        object_pos2, object_quat2 = self.pybullet_client.getBasePositionAndOrientation(self.T_object2)

        image = self.get_image_observation()
        if image2:
            image2 = self.get_image_observation(camera='camera2')
        else:
            image2 = None


        #create dict for camera information
        camera_info = {
            "T" : self.camera.T,
            "R" : self.camera.R,
            "FoVx" : self.camera.FoVx,
            "FoVy" : self.camera.FoVy,
            "image_width" : self.camera.image_width,
            "image_height" : self.camera.image_height,
        }

        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_positions_dummy": joint_positions_dummy,
            "state": np.concatenate([dummy_ee_pos]),
            "action": np.concatenate([ee_pos]),
            "gripper_position": gripper_pos,
            "object_position": object_pos,
            "object_orientation": object_quat, 
            "target_position": object_pos2,
            "target_orientation": object_quat2,
            "base_rgb": image,
            "wrist_rgb": image2,
            "ee_pos_quat": np.concatenate([ee_pos, ee_quat]),
            "camera_info" : camera_info,
        }
    
    def get_image_observation(self, camera='camera1') -> Dict[str, np.ndarray]:
        if camera == 'camera1':
            cur_joint = self.get_joint_state_dummy()
            cur_joint = [0] + cur_joint.tolist()
            cur_joint = np.array(cur_joint)

            #get object position and orientation
            cur_position = np.array(self.pybullet_client.getBasePositionAndOrientation(self.T_object)[0])
            cur_position = torch.from_numpy(cur_position).float().cuda()
            cur_rotation = np.array(self.pybullet_client.getBasePositionAndOrientation(self.T_object)[1])
            cur_rotation = np.roll(cur_rotation, 1)
            cur_rotation = torch.from_numpy(cur_rotation).float().cuda()   

            transformations_list = self.get_transfomration_list(cur_joint)
            segmented_list, xyz = get_segmented_indices(self.gaussians_backup)
            xyz, rot, opacity, shs_featrest, shs_dc  = transform_means(self.gaussians_backup, xyz, segmented_list, transformations_list)
            # xyz_cube, rot_cube, opacity_cube, scales_cube, shs_dc_cube, sh_rest_cube = place_object(gaussians_backup, pos=torch.from_numpy(cur_object).to(device='cuda').float(), rotation=torch.from_numpy(curr_rotation).to(device='cuda').float())
            xyz_obj, rot_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj = transform_object(self.T_object_gaussian, pos=cur_position, quat=cur_rotation)
            
            with torch.no_grad():
                    # gaussians.active_sh_degree = 0
                    self.robot_gaussian._xyz = torch.cat([xyz, xyz_obj], dim=0)
                    self.robot_gaussian._rotation = torch.cat([rot, rot_obj], dim=0)
                    self.robot_gaussian._opacity = torch.cat([opacity, opacity_obj], dim=0)
                    self.robot_gaussian._features_rest = torch.cat([shs_featrest, features_rest_obj], dim=0)
                    self.robot_gaussian._features_dc = torch.cat([shs_dc, features_dc_obj], dim=0)
                    self.robot_gaussian._scaling = torch.cat([self.gaussians_backup._scaling, scales_obj], dim=0)
        
        if camera == 'camera1':
            rendering = render(self.camera, self.robot_gaussian, self.pipeline, self.background)["render"]
        else:
            rendering = render(self.camera2, self.robot_gaussian, self.pipeline, self.background)["render"]

        # t_gaussians = copy.deepcopy(t_gaussians_backup)

        # convert into numpy
        rendering = rendering.detach().cpu().numpy()

        # convert to hxwxc from cxhxw
        rendering = np.transpose(rendering, (1, 2, 0))
        
        #convert to 0-255
        rendering = (rendering * 255).astype(np.uint8)

        #make the image sharper
        kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        rendering = cv2.filter2D(rendering, -1, kernel)
       

        #show the image
        cv2.imshow(camera, cv2.cvtColor(rendering, cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)

        
        #save the image
        return rendering
        

    def setup_camera2(self):
        uid = 0
        colmap_id = 1
        R = torch.from_numpy(np.array([[-0.98784567,  0.00125165,  0.15543282],
                        [0.1153457,   0.67620402,  0.72762868],
                        [ -0.10419356,  0.73671335, -0.66812959]])).float()
        
        T = torch.Tensor([ 1.04674738, -0.96049824,  2.03845016] ).float()



        
        # T = torch.Tensor([ 0.09347542+0.5, -0.74648806+0.6,  5.57444971+0.2] ).float()


        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        gt_mask_alpha = None

        image_width = 629 
        image_height = 472 
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
                        [ -9.89903676e-01, -2.86133428e-04,  1.41741420e-01]])).float()
        
        T = torch.Tensor([ -1.88933526, -0.6446558,   2.98276143] ).float()



        
        # T = torch.Tensor([ 0.09347542+0.5, -0.74648806+0.6,  5.57444971+0.2] ).float()


        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        gt_mask_alpha = None

        image_width = 629 //2
        image_height = 472 //2
        image_name = 'test'
        image = torch.zeros((3, image_height, image_width)).float()

        #order is  colmap_id, R, T, FoVx, FoVy, image, gt_alpha_mask, image_name, uid,

        camera = Camera(colmap_id, R, T, FoVx, FoVy, image, gt_mask_alpha, image_name, uid)

        return camera


    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        initial_joint_state = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        self.initial_joint_state = initial_joint_state
        joint_signs = [1, 1, 1, 1, 1, 1]
        for i in range(1, 7):
            self.pybullet_client.resetJointState(self.robot, i, initial_joint_state[i-1]*joint_signs[i-1])

        self.initial_joints = []
        for joint_index in range(8):
            joint_info = self.pybullet_client.getJointInfo(self.robot, joint_index)
            joint_name = joint_info[1].decode("utf-8")  
            link_state = self.pybullet_client.getLinkState(self.robot, joint_index, computeForwardKinematics=True)
            self.initial_joints.append(link_state)
        
        #get end effector position and orientation
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.robot, 6)[0], self.pybullet_client.getLinkState(self.robot, 6)[1]
        self.iniital_ee_quat = ee_quat


        #set the initial position of the dummy robot using inverse kinematics
        dummy_ee_pos = [ee_pos[0], ee_pos[1], 0.265]
        dummy_joint_pos = self.pybullet_client.calculateInverseKinematics(self.robot, 6, dummy_ee_pos, self.iniital_ee_quat, 
            residualThreshold=0.00001, maxNumIterations=1000, 
            lowerLimits=[initial_joint_state[k] - np.pi/2 for k in range(6)],
            upperLimits=[initial_joint_state[k] + np.pi/2 for k in range(6)],
            jointRanges=[12.566, 12.566, 6.282, 12.566, 12.566, 12.566],
            restPoses=[0* np.pi, -0.5* np.pi, 0.5* np.pi, -0.5* np.pi, -0.5* np.pi, 0]
            )
        
        dummy_joint_pos = list(dummy_joint_pos)
        
        for i in range(1, 7):
            self.pybullet_client.resetJointState(self.dummy_robot, i, dummy_joint_pos[i-1])

        

        #get end effector position and orientation
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 7)[0], self.pybullet_client.getLinkState(self.dummy_robot, 7)[1]
        self.last_obs = self.get_joint_state()

        while True:
            # self.pybullet_client.stepSimulation()

            #get T_object position and orientation
            object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.T_object)
            #get T_object2 position and orientation
            object_pos2, object_quat2 = self.pybullet_client.getBasePositionAndOrientation(self.T_object2)

            #distance between T_object and T_object2
            distance = np.linalg.norm(np.array(object_pos) - np.array(object_pos2))

            #orientation difference between T_object and T_object2
            orientation_diff = np.linalg.norm(np.array(object_quat) - np.array(object_quat2))

            # self.get_observations(image2=False)

            if distance < 0.02 and orientation_diff < 0.024:
                print('Object reached')
                print('orientation_diff: ', orientation_diff)
                time.sleep(3)
                #reset T_object position and orientation
                x = random.uniform(-0.4, 0.5)
                y_low = 0.6 - np.sqrt(1 - (x-0.1)**2 / (0.5)**2) * 0.1
                y_high = 0.6 + np.sqrt(1 - (x-0.1)**2 / (0.5)**2) * 0.1
                y = random.uniform(y_low, y_high)
                self.pybullet_client.resetBasePositionAndOrientation(self.T_object, [x, y, 0], [0, 0, 0, 1])
                self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.1))

                x = random.uniform(0.4, 0.6)
                y = random.uniform(-0.3, 0.3)
                # x = 0.5 + random.uniform(-0.03, 0.03)
                # y = -0.03 + random.uniform(-0.01, 0.01)
                # random euler angles for the orientation of the object
                euler_z =  random.uniform(-np.pi, np.pi)
                # random quaternion for the orientation of the object
                quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])

                self.pybullet_client.resetBasePositionAndOrientation(self.T_object, [x, y, 0], quat)

                # load another object without collision
                self.target_location = [0.5+0.0, 0.-0.03, -0.01395]
                quat = self.pybullet_client.getQuaternionFromEuler([0, 0, np.pi+np.pi/4])
               
                #disable collision for T_object2
                collisionFilterGroup = 0
                collisionFilterMask = 0
                p.setCollisionFilterGroupMask(self.T_object2, -1, collisionFilterGroup, collisionFilterMask)


                #randomize the friction of the plane
                self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.5))

            time.sleep(1/240)

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None: 
        self.stop()


    def get_transfomration_list(self, new_joint_poses):

            
        new_joints = []
        for joint_index in range(8):
            joint_info = p.getJointInfo(self.dummy_robot, joint_index)
            joint_name = joint_info[1].decode("utf-8") 
            link_state = p.getLinkState(self.dummy_robot, joint_index, computeForwardKinematics=True)
            new_joints.append(link_state)
            
            
        transformations_list = []
        for joint_index in range(8):
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
        
    #transform_back
    xyz = torch.matmul(inv_rotation_matrix, xyz.T).T + inv_translation
    
        
    return xyz, rot, opacity, shs_featrest, shs_dc


def transform_object(pc, pos = torch.tensor([0, 0.3, 0.09]).to(device='cuda').float(), quat = torch.tensor([1, 0, 0, 0]).to(device='cuda').float()):
            
#   Trans = #     -0.171274 0.028960 -0.021428 0.532716
# 0.023970 0.013890 -0.172815 -0.322159
# -0.026895 -0.172049 -0.017558 0.371520
# 0.000000 0.000000 0.000000 1.000000
    
    ### canonical 
    ### -0.132477 0.034095 -0.121401 0.035303
# 0.123114 -0.003108 -0.135219 0.329979
# -0.027270 -0.179663 -0.020700 0.264398
# 0.000000 0.000000 0.000000 1.000000

    Trans_canonical = torch.tensor([[-0.132477, 0.034095, -0.121401, 0.035303],
                            [0.123114, -0.003108, -0.135219, 0.329979],
                            [-0.027270, -0.179663, -0.020700, 0.264398],
                            [0, 0, 0, 1]]).to(device=pc.get_xyz.device) # shape (4, 4)
    #canonical 0.099509 -0.000807 0.037517 -0.017219
# -0.035263 -0.038370 0.092707 -0.177059
# 0.012832 -0.099183 -0.036170 0.180745
# 0.000000 0.000000 0.000000 1.000000
    
    # Trans_canonical = torch.tensor([[0.099509, -0.000807, 0.037517, -0.017219],
    #                         [-0.035263, -0.038370, 0.092707, -0.177059],
    #                         [0.012832, -0.099183, -0.036170, 0.180745],
    #                         [0, 0, 0, 1]]).to(device=pc.get_xyz.device) # shape (4, 4)



    
    
    rotation_matrix_c = Trans_canonical[:3, :3]
    translation_c = Trans_canonical[:3, 3]
    scale_obj = torch.pow(torch.linalg.det(rotation_matrix_c), 1/3)
    
    Trans_robot = torch.tensor([[-0.171274, 0.028960, -0.021428, 0.532716],
                            [0.023970, 0.013890, -0.172815, -0.322159],
                            [-0.026895, -0.172049, -0.017558, 0.371520],
                            [0, 0, 0, 1]]).to(device=pc.get_xyz.device) # shape (4, 4)

    
    rotation_matrix_r = Trans_robot[:3, :3]
    scale_r = torch.pow(torch.linalg.det(rotation_matrix_r), 1/3)

    translation_r = Trans_robot[:3, 3]

    inv_transformation_r = torch.inverse(Trans_robot)
    inv_rotation_matrix_r = inv_transformation_r[:3, :3]
    inv_translation_r = inv_transformation_r[:3, 3]
    inv_scale = torch.pow(torch.linalg.det(inv_rotation_matrix_r), 1/3)
    
    xyz_obj = pc.get_xyz
    rotation_obj = pc.get_rotation
    opacity_obj = pc.get_opacity_raw
    scales_obj = pc.get_scaling
    scales_obj = scales_obj * scale_obj * inv_scale 
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
    
    
    aabb = ((-0.10300000149011612, -0.17799999701976776, -0.0030000000000000027), (0.10300000149011612, 0.028000000372529033, 0.022999999552965167))
    aabb = ((-0.10300000149011612, -0.17799999701976776, -0.0030000000000000027), (0.10300000149011612, 0.028000000372529033, 0.04299999910593033))
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
    
    # Construction coefficient
    D_1 = o3.wigner_D(1, rot_angles[0], - rot_angles[1], rot_angles[2])
    D_2 = o3.wigner_D(2, rot_angles[0], - rot_angles[1], rot_angles[2])
    D_3 = o3.wigner_D(3, rot_angles[0], - rot_angles[1], rot_angles[2])

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
    condition = torch.where(((points[:, 0] < centers[3, 0]+0.2) * (points[:,1] > centers[4, 1]) * (points[:, 2] < centers[5, 2]))*box_condition)[0]
    segmented_points.append(condition)


    #undo the temporary transformation
    points = torch.matmul(torch.inverse(temp_matrix)[:3, :3], points.T).T + torch.inverse(temp_matrix)[:3, 3]

    
    return segmented_points, points
