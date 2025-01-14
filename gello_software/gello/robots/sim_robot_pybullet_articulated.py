import pickle
import threading
import time
from typing import Any, Dict, Optional

import mujoco
import mujoco.viewer
import numpy as np
import zmq
from dm_control import mjcf

from gello.robots.robot import Robot

assert mujoco.viewer is mujoco.viewer
import pybullet as p


import urdf_models.models_data as md
from pybullet_planning.interfaces.robots.collision import pairwise_collision
import random
from collections import namedtuple
import math
import torch
from scipy.spatial.transform import Rotation as R

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
        self.dummy_robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, -0.1], useFixedBase=True, flags=flags)




        for i in range(self.pybullet_client.getNumJoints(self.dummy_robot)):
            info = self.pybullet_client.getJointInfo(self.dummy_robot, i)
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
        euler_z =  0
        # random quaternion for the orientation of the object
        quat = self.pybullet_client.getQuaternionFromEuler([0, euler_z,0])


        models_lib = md.model_lib()
        self.object_name_list = [ 'glasses']
        self.x_list = [[0.5, -0.0], [0.5, -0.0], [0.5, -0.0], [0.5, 0], [0.5, 0.0], [0.5, 0.0], [0.5, 0.0]]

        self.urdf_object_list = []
        #load the articulated object at 0.0, 0.0, 0.0
        self.urdf_object_list.append(self.pybullet_client.loadURDF('/home/nomaan/Desktop/corl24/ocean_backup/splat/articulated_objects/articulated_object_urdfs/101297/mobility.urdf', [0.0, 0.0, 0.0], quat, globalScaling=0.33, useFixedBase=True))

        #record the pose of each link of the object
        self.link_poses = []
        for i in range(len(self.urdf_object_list)):
            num_links = self.pybullet_client.getNumJoints(self.urdf_object_list[i])
            overall_poses = []
            for j in range(num_links):
                overall_poses.append(self.pybullet_client.getLinkState(self.urdf_object_list[i], j))
            self.link_poses.append(overall_poses)


        #transfer the object to random position
        x = random.uniform(0.4, 0.6)
        y = random.uniform(-0.3, 0.3)
        self.pybullet_client.resetBasePositionAndOrientation(self.urdf_object_list[0], [x, y, 0], quat)
        
        #change the friction of the object
        for T_object in self.urdf_object_list:  
            self.pybullet_client.changeDynamics(T_object, -1, lateralFriction=16.1)
            #change rolling friction
            

        #add gravity
        self.pybullet_client.setGravity(0, 0, -9.81)
        
        #add plane
        import pybullet_data
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = self.pybullet_client.loadURDF("plane.urdf", [0, 0, -0.022])

        #place a wall in -0.4 at x axis using plane.urdf
        # wall is perpendicular to the plane
        quat = self.pybullet_client.getQuaternionFromEuler([ 0, np.pi/2,0])
        self.wall = self.pybullet_client.loadURDF("plane.urdf", [-0.4, 0, 0.0], quat)

        ##add a spher at 0.4, 0.5 0.01 without urdf 
        self.sphere = self.pybullet_client.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1], specularColor=[0.4, .4, 0], visualFramePosition=[0.4, 0.6, 0.01])
        self.sphere_id = self.pybullet_client.createMultiBody(baseMass=0, baseVisualShapeIndex=self.sphere)

        ## add stage 
        self.stage = 0 

        #change the friction of the plane
        # self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.1))

        #set time step
        self.pybullet_client.setTimeStep(1/240)
        # self.pybullet_client.setRealTimeSimulation(0)

        #current gripper state 
        self.current_gripper_action = 0

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
        joint_states = []
        num_joints = self.pybullet_client.getNumJoints(self.dummy_robot)
        for i in range(1, num_joints):
            joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])
        return np.array(joint_states)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )

        for i in range(1, 7):
            # self.pybullet_client.resetJointState(self.robot, i, joint_state[i-1])
            self.pybullet_client.setJointMotorControl2(self.dummy_robot, i, p.POSITION_CONTROL, targetPosition=joint_state[i-1], force=250) 

        if self.use_gripper:
            self.move_gripper((1-joint_state[-1])*0.085)

            self.current_gripper_action = joint_state[-1]

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
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
        
        for i in range(len(self.urdf_object_list)):
            object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.urdf_object_list[i])
            observations[self.object_name_list[i] + "_position"] = object_pos
            observations[self.object_name_list[i] + "_orientation"] = object_quat

        #we also want to give articulated information
        #the way to do that is to give the transformation of each link in the initial frame
        for i in range(len(self.urdf_object_list)):
            #first find the new link poses
            new_link_poses = []
            for j in range(len(self.link_poses[i])):
                new_link_poses.append(self.pybullet_client.getLinkState(self.urdf_object_list[i], j))
            #then find the transformation of each link in the initial frame
            #note that we 
            for j in range(len(new_link_poses)):
                initial_link_pose = self.link_poses[i][j]
                new_link_pose = new_link_poses[j]
                
                initial_link_pos, initial_link_quat = initial_link_pose[0], initial_link_pose[1]
                new_link_pos, new_link_quat = new_link_pose[0], new_link_pose[1]

                # Create transformation matrices for both initial and new poses
                initial_transform = np.eye(4)
                initial_transform[:3, :3] = np.array(self.pybullet_client.getMatrixFromQuaternion(initial_link_quat)).reshape(3, 3)
                initial_transform[:3, 3] = np.array(initial_link_pos)

                new_transform = np.eye(4)
                new_transform[:3, :3] = np.array(self.pybullet_client.getMatrixFromQuaternion(new_link_quat)).reshape(3, 3)
                new_transform[:3, 3] = np.array(new_link_pos)

                # Calculate transformation from initial to current frame
                # This will convert points from initial frame to current frame: p_current = transform @ p_initial
                transform = new_transform @ np.linalg.inv(initial_transform)

                # Store the transformation in observations
                link_key = f"{self.object_name_list[i]}_link_{j}_transform"
                observations[link_key] = transform

        return observations

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        initial_joint_state = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        self.initial_joint_state = initial_joint_state
        joint_signs = [1, 1, 1, 1, 1, 1]
        for i in range(1, 7):
            self.pybullet_client.resetJointState(self.dummy_robot, i, initial_joint_state[i-1]*joint_signs[i-1])
            self.move_gripper(0.0)




        #print joint angles 
        joint_states = []
        for i in range(1, 19):
            joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])


        while True:
            # self.get_camera_image_from_end_effector()
            self.pybullet_client.stepSimulation()

            #get static camera image
            # self.get_camera_static()

            #get object position
            object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.urdf_object_list[0])

            #create transformation matrix from the object position and orientation
            object_transformation = np.eye(4)
            object_transformation[:3, :3] = np.array(self.pybullet_client.getMatrixFromQuaternion(object_quat)).reshape(3, 3)
            object_transformation[:3, 3] = np.array(object_pos)

            #get the end effector position and orientation
            ee_pos, ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0], self.pybullet_client.getLinkState(self.dummy_robot, 6)[1]
            ee_transformation = np.eye(4)
            ee_transformation[:3, :3] = np.array(self.pybullet_client.getMatrixFromQuaternion(ee_quat)).reshape(3, 3)
            ee_transformation[:3, 3] = np.array(ee_pos)

            #get the transformation of end effector in the object frame
            ee_transformation_object_frame = np.linalg.inv(object_transformation) @ ee_transformation


            
            
            #get the distance of the object from the sphere
            distance = np.linalg.norm(np.array(object_pos) - np.array([0.4, 0.6, 0.05]))

            #if the object is close to the sphere, reset the object position
            if distance < 0.05:
                #turn sphere green
                self.pybullet_client.changeVisualShape(self.sphere_id, -1, rgbaColor=[0, 1, 0, 1])

                time.sleep(5)
                x = random.uniform(.1, 0.8)
                y = random.uniform(-0.6, 0.6)
                # random euler angles for the orientation of the object
                euler_z =  random.uniform(0, 0)
                # random quaternion for the orientation of the object
                quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])
                self.pybullet_client.resetBasePositionAndOrientation(self.urdf_object_list[0], [x, y, 0], quat)

                #turn sphere red
                self.pybullet_client.changeVisualShape(self.sphere_id, -1, rgbaColor=[1, 0, 0, 1])

            time.sleep(1/240)

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None: 
        self.stop()


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
            self.pybullet_client.changeConstraint(c, gearRatio=-multiplier, maxForce=1000, erp=1)  # Note: the mysterious `erp` is of EXTREME importance

    def get_link_pose(self, body, link):
        result = self.pybullet_client.getLinkState(body, link)
        return result[4], result[5]
    
    def move_gripper(self, open_length):
        # open_length = np.clip(open_length, *self.gripper_range)
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation
        # Control the mimic gripper joint(s)
        p.setJointMotorControl2(self.dummy_robot, self.mimic_parent_id, self.pybullet_client.POSITION_CONTROL, targetPosition=open_angle,
                                force=20, maxVelocity=self.joints[self.mimic_parent_id].maxVelocity)


    def get_current_gripper_state(self):
        return self.pybullet_client.getJointState(self.dummy_robot, self.mimic_parent_id)[0]
    
    def get_camera_image_from_end_effector(self):

        cam_fov = 90
        near_plane = 0.01
        far_plane = 100
        # Get the pose of the end effector
        end_effector_state = self.pybullet_client.getLinkState(self.dummy_robot, 8)
        end_effector_pos = end_effector_state[4]
        end_effector_orn = end_effector_state[5]

        # Convert the quaternion orientation to a rotation matrix
        rot_matrix = self.pybullet_client.getMatrixFromQuaternion(end_effector_orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Define the camera position relative to the end effector
        camera_position = np.array([0, 0, 0.1])  # Adjust as needed
        camera_position_world = end_effector_pos + rot_matrix.dot(camera_position)

        # Define the camera view direction
        camera_target_position = np.array([0, 0, 1])  # Adjust as needed
        camera_target_position_world = end_effector_pos + rot_matrix.dot(camera_target_position)

        # Compute the view matrix
        view_matrix = self.pybullet_client.computeViewMatrix(camera_position_world, camera_target_position_world, [0, 0, 1])

        # Compute the projection matrix
        projection_matrix = self.pybullet_client.computeProjectionMatrixFOV(cam_fov, 1.0, near_plane, far_plane)

        # Get the camera image
        width, height, rgb_img, depth_img, seg_img = self.pybullet_client.getCameraImage(320, 240, view_matrix, projection_matrix, flags=self.pybullet_client.ER_NO_SEGMENTATION_MASK)

        return rgb_img
    

    def get_camera_static(self):

        cam_fov = 90
        near_plane = 0.01
        far_plane = 100

        # Camera parameters in splat frame
        R_splat = torch.from_numpy(np.array([[1.27679278e-01, -4.36057591e-01,  8.90815233e-01],
                        [6.15525303e-02,  8.99918716e-01,  4.31691546e-01],
                        [ -9.89903676e-01, -2.86133428e-04,  1.41741420e-01]])).float()

        T_splat = torch.Tensor([ -1.88933526, -0.6446558,   2.98276143] ).float()

        # Transformation matrix from splat frame to pybullet frame
        transform = np.array([[-0.171274, 0.028960, -0.021428, 0.532716],
                            [0.023970, 0.013890, -0.172815, -0.322159],
                            [-0.026895, -0.172049, -0.017558, 0.371520],
                            [0, 0, 0, 1]])
        

        # Convert R_splat to numpy and add the translation vector to form a 4x4 matrix
        scale = np.power(np.linalg.det(R_splat), 1/3)
        camera_matrix_splat = np.eye(4)
        camera_matrix_splat[:3, :3] = R_splat.numpy()
        camera_matrix_splat[:3, 3] = T_splat.numpy()

        transform[:3,:3] = transform[:3,:3] / scale

        # Apply the transformation matrix from splat frame to pybullet frame
        camera_matrix_pybullet = np.dot(transform, camera_matrix_splat)

        # Extract rotation and translation from the resulting matrix
        R_pybullet = camera_matrix_pybullet[:3, :3]
        T_pybullet = camera_matrix_pybullet[:3, 3]
        T_pybullet[1] = 0

        # Convert rotation matrix to quaternion
        rotation_pybullet = R.from_matrix(R_pybullet)
        quaternion_pybullet = rotation_pybullet.as_quat()  # [x, y, z, w]

        # PyBullet camera placement
        camera_target_position = T_pybullet + R_pybullet[:, 2]  # Target is along the camera's forward vector

        view_matrix = self.pybullet_client.computeViewMatrix(
            cameraEyePosition=T_pybullet.tolist(),
            cameraTargetPosition=(camera_target_position).tolist(),  # Target is along the camera's forward vector
            cameraUpVector=([0,0,1])
        )

        #add a sphere at the camera position
        self.pybullet_client.resetBasePositionAndOrientation(self.sphere_id, T_pybullet.tolist(), quaternion_pybullet.tolist())


        # Compute the projection matrix
        projection_matrix = self.pybullet_client.computeProjectionMatrixFOV(cam_fov, 1.0, near_plane, far_plane)



        # Get the camera image
        width, height, rgb_img, depth_img, seg_img = self.pybullet_client.getCameraImage(
            1280, 960, view_matrix, projection_matrix, flags=self.pybullet_client.ER_NO_SEGMENTATION_MASK)
        
        return rgb_img
