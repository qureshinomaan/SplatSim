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
        self.robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, -0.1], useFixedBase=True, flags=flags)
        self.dummy_robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, -0.1], useFixedBase=True, flags=flags)

        ## disable collision between dummy_robot and everything
        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            self.pybullet_client.setCollisionFilterGroupMask(self.robot, i, 0, 0)

        ## enable collision between dummy_robot and everything but robot
        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            self.pybullet_client.setCollisionFilterGroupMask(self.dummy_robot, i, 0, 1)


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

        #make T_object orange
        self.pybullet_client.changeVisualShape(T_object, -1, rgbaColor=[1, 0.5, 0, 1])

        # load another object without collision
        self.target_location = [0.5+0.0, 0.-0.03, -0.02895]
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, np.pi+np.pi/4])
        T_object2 = self.pybullet_client.loadURDF('../gaussian-splatting/pybullet-playground/urdf/T_object/urdf_T.urdf', self.target_location, quat, globalScaling=0.93, useFixedBase=True)
        self.T_object2 = T_object2
        #disable collision for T_object2
        collisionFilterGroup = 0
        collisionFilterMask = 0
        p.setCollisionFilterGroupMask(T_object2, -1, collisionFilterGroup, collisionFilterMask)



        #make the T_object2 transparent and black
        self.pybullet_client.changeVisualShape(T_object2, -1, rgbaColor=[0, 0, 0, 0.01])

        #make robot transparent
        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            self.pybullet_client.changeVisualShape(self.robot, i, rgbaColor=[0, 0, 0, 0.0])

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

        self.total_images_count = 0 

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
            self.pybullet_client.setJointMotorControl2(self.robot, i, p.POSITION_CONTROL, targetPosition=joint_state[i-1], force=250) 

    
        #get end effector position and orientation
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.robot, 7)[0], self.pybullet_client.getLinkState(self.robot, 7)[1]

        #get current end effector position and orientation of the dummy robot
        ee_pos_dummy, ee_quat_dummy = self.pybullet_client.getLinkState(self.dummy_robot, 7)[0], self.pybullet_client.getLinkState(self.dummy_robot, 7)[1]

        #set the initial position of the dummy robot using inverse kinematics
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
        
        
        for _ in range(1):
            for i in range(1, 7):
                # self.pybullet_client.resetJointState(self.dummy_robot, i, dummy_joint_pos[i-1])
                self.pybullet_client.setJointMotorControl2(self.dummy_robot, i, p.POSITION_CONTROL, targetPosition=dummy_joint_pos[i-1], force=100)
            self.pybullet_client.stepSimulation()
            time.sleep(1/240)

        self.get_camera_static()
        

        error = np.linalg.norm(np.array(dummy_joint_pos)[:6] - np.array(self.get_joint_state_dummy())[:6])
        step = 0

        # while error > 0.05:
        #     for i in range(1, 7):
        #         # self.pybullet_client.resetJointState(self.dummy_robot, i, dummy_joint_pos[i-1])
        #         self.pybullet_client.setJointMotorControl2(self.dummy_robot, i, p.POSITION_CONTROL, targetPosition=dummy_joint_pos[i-1], force=100)
        #     error = np.linalg.norm(np.array(dummy_joint_pos)[:6] - np.array(self.get_joint_state_dummy())[:6])
        #     self.pybullet_client.stepSimulation()
        #     time.sleep(1/240)
        #     step += 1
        if self.use_gripper:
            self.move_gripper(0.0)

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self.get_joint_state()
        joint_positions_dummy = self.get_joint_state_dummy()
        joint_velocities = np.array([self.pybullet_client.getJointState(self.dummy_robot, i)[1] for i in range(7)])

        dummy_ee_pos, dummy_ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0], self.pybullet_client.getLinkState(self.dummy_robot, 6)[1]
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.robot, 6)[0], self.pybullet_client.getLinkState(self.robot, 6)[1]

        gripper_pos = self.pybullet_client.getJointState(self.dummy_robot, 7)[0]

        # T_object position and orientation
        object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.T_object)

        # Target object position and orientation
        object_pos2, object_quat2 = self.pybullet_client.getBasePositionAndOrientation(self.T_object2)

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
            "ee_pos_quat": ee_quat,
        }


    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        initial_joint_state = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        self.initial_joint_state = initial_joint_state
        joint_signs = [1, 1, 1, 1, 1, 1]
        for i in range(1, 7):
            self.pybullet_client.resetJointState(self.robot, i, initial_joint_state[i-1]*joint_signs[i-1])
            print('here')
            self.move_gripper(0.0)



        #print joint states
        for i in range(1, 20):
            print(self.pybullet_client.getJointState(self.robot, i)[0])
        # exit()


        #print link names of robot 
        # for i in range(self.pybullet_client.getNumJoints(self.robot)):
        #     info = self.pybullet_client.getJointInfo(self.robot, i)
        #     print(info[1].decode("utf-8"))
        # exit()
        
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
        ee_pos, ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0], self.pybullet_client.getLinkState(self.dummy_robot, 6)[1]
        self.last_obs = self.get_joint_state()



        while True:
            self.pybullet_client.stepSimulation()

            
            if self.stage == 0:

                #get T_object position and orientation
                object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.T_object)
                #get T_object2 position and orientation
                object_pos2, object_quat2 = self.pybullet_client.getBasePositionAndOrientation(self.T_object2)

                #distance between T_object and T_object2
                distance = np.linalg.norm(np.array(object_pos)[:2] - np.array(object_pos2)[:2])

                #orientation difference between T_object and T_object2
                orientation_diff = np.linalg.norm(np.array(object_quat) - np.array(object_quat2))


                if distance < 0.02 and orientation_diff < 0.02:
                    self.stage = 1
                    #turn the sphere green
                    self.pybullet_client.changeVisualShape(self.sphere_id, -1, rgbaColor=[0, 1, 0, 1])
            
            if self.stage == 1:

                object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.T_object)
                #get T_object2 position and orientation
                object_pos2, object_quat2 = self.pybullet_client.getBasePositionAndOrientation(self.T_object2)

                #distance between T_object and T_object2
                distance = np.linalg.norm(np.array(object_pos)[:2] - np.array(object_pos2)[:2])

                #orientation difference between T_object and T_object2
                orientation_diff = np.linalg.norm(np.array(object_quat) - np.array(object_quat2))



                #find the distance of the end effector to the sphere [0.4, 0.5, 0.01]
                ee_pos = self.pybullet_client.getLinkState(self.robot, 7)[0]
                distance2 = np.linalg.norm(np.array(ee_pos)[:2] - np.array([0.4, 0.6]))

                if distance > 0.02 or orientation_diff > 0.02:
                    self.stage = 0
                    #turn the sphere red
                    self.pybullet_client.changeVisualShape(self.sphere_id, -1, rgbaColor=[1, 0, 0, 1])

                elif distance2 < 0.05:

                    #turn the sphere red
                    self.pybullet_client.changeVisualShape(self.sphere_id, -1, rgbaColor=[1, 0, 0, 1])


                    time.sleep(5)

                    #reset T_object position and orientation
                    x = random.uniform(.3, 0.7)
                    y = random.uniform(-0.35, 0.35)
                    # x = 0.5 + random.uniform(-0.2, 0.03)
                    # y = -0.03 + random.uniform(-0.5, -0.2)
                     # random euler angles for the orientation of the object
                    euler_z =  random.uniform(-np.pi, np.pi)
                    # random quaternion for the orientation of the object
                    quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])
                    self.pybullet_client.resetBasePositionAndOrientation(self.T_object, [x, y, 0], quat)
                    
                    #reset the stage
                    self.stage = 0  



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
            640, 480, view_matrix, projection_matrix, flags=self.pybullet_client.ER_NO_SEGMENTATION_MASK)
        

        #save the camera image
        import cv2
        rgb_img = np.reshape(rgb_img, (height, width, 4))
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGBA2RGB)
        self.total_images_count += 1
        cv2.imwrite(f"stream/image_{self.total_images_count}.png", cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))
        
        
        return rgb_img