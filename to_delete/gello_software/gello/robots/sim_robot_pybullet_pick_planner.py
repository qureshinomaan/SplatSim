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
from pybullet_planning import plan_joint_motion, get_movable_joints, set_joint_positions
import random
from collections import namedtuple
import math

import os
import pickle


class ZMQServerThread(threading.Thread):
    def __init__(self, server):
        super().__init__()
        self._server = server

    def run(self):
        self._server.serve()

    def terminate(self):
        self._server.stop()

def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles.
    
    Parameters:
    R (numpy.ndarray): A 3x3 rotation matrix.
    
    Returns:
    tuple: A tuple containing the Euler angles (roll, pitch, yaw) in radians.
    """
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])  # Roll
        y = math.atan2(-R[2, 0], sy)      # Pitch
        z = math.atan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return x, y, z


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
        x = random.uniform(0.2, 0.7)
        y = random.uniform(-0.4, 0.4)
        # random euler angles for the orientation of the object
        euler_z =  random.uniform(-np.pi, np.pi)
        euler_z = 0
        # random quaternion for the orientation of the object
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])


        models_lib = md.model_lib()
        self.object_name_list = ['plastic_apple']
        self.urdf_object_list = []
        for object_name in self.object_name_list:
            object_loaded = self.pybullet_client.loadURDF(models_lib[object_name], [x, y, 0], quat, globalScaling=1, useFixedBase=False)
            self.urdf_object_list.append(object_loaded)


        self.banana_grasp_pose = np.array([[-0.25341801,  0.09241874,  0.96293203, -0.05715429],
                                        [-0.96604641,  0.02761772, -0.25688828, -0.0386821 ],
                                        [-0.05033528, -0.99533715,  0.08228196,  0.24903046],
                                        [ 0.,          0.,          0.,          1.        ]])
        

        self.apple_grasp_pose = np.array([[-0.12515046, -0.0412762,   0.99127879,  0.00471373],
                                [-0.98896543, -0.07464537, -0.12796658,  0.01413896],
                                [ 0.07927635, -0.99635553, -0.03147883,  0.27105228],
                                [ 0. ,         0. ,         0.,          1.        ]])
        


        #change the friction of the object
        for T_object in self.urdf_object_list:  
            self.pybullet_client.changeDynamics(T_object, -1, lateralFriction=10)

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

        #trajectory path
        self.path = '/home/nomaan/bc_data/gello/'

        self.trajectory_count = 0

        #step simulation
        for i in range(100):
            self.pybullet_client.stepSimulation()
            # time.sleep(1/240)

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

        #get iniital ee_pos and ee_quat
        initial_ee_pos, initial_ee_quat = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0], self.pybullet_client.getLinkState(self.dummy_robot, 6)[1]

        #print joint angles 
        joint_states = []
        for i in range(1, 19):
            joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])



        initial_gripper_random_state = random.uniform(0, 0.08)
        for i in range(10000):
            #set to initial joint state
            for i in range(1, 7):
                self.pybullet_client.resetJointState(self.dummy_robot, i, initial_joint_state[i-1]*joint_signs[i-1])

            self.move_gripper(initial_gripper_random_state)
            self.pybullet_client.stepSimulation()
            # time.sleep(1/240)

        while True:
            # self.get_camera_image_from_end_effector()

            #randomly reset the object position and orientation
            x = random.uniform(0.1, 0.7)
            y = random.uniform(-0.6, 0.6)
            # random euler angles for the orientation of the object
            euler_z =  random.uniform(0, 0)
            # random quaternion for the orientation of the object
            quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])

            for object_id in self.urdf_object_list:
                self.pybullet_client.resetBasePositionAndOrientation(object_id, [x, y, 0], quat)

            
            # for i in range(100):
            #     self.pybullet_client.stepSimulation()
            #     # time.sleep(1/240)
            #     self.move_gripper(0.0)
            #     for k in range(1, 7):
            #         self.pybullet_client.resetJointState(self.dummy_robot, k, initial_joint_state[k-1]*joint_signs[k-1])


            #generating random initial joint state using random end effector position and orientation
            # random end effector position
            random_ee_pos = np.array([random.uniform(0.2, 0.7), random.uniform(-0.4, 0.4), random.uniform(0.2, 0.6)])
            random_ee_quat = initial_ee_quat

            #joint angles using inverse kinematics
            initial_joint_positions = self.pybullet_client.calculateInverseKinematics(self.dummy_robot, 6, random_ee_pos, random_ee_quat, maxNumIterations=100000, residualThreshold=1e-10)
            

            

            self.pybullet_client.stepSimulation()

            #get object position
            object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.urdf_object_list[0])

            #create transformation matrix from the object position and orientation
            object_transformation = np.eye(4)
            object_transformation[:3, :3] = np.array(self.pybullet_client.getMatrixFromQuaternion(object_quat)).reshape(3, 3)
            object_transformation[:3, 3] = np.array(object_pos)

            #get the end effector position and orientation according to self.apple_grasp_pose
            ee_transformation =  object_transformation @ self.apple_grasp_pose

            #get pregrasp to grasp path
            pre_grasp2grasp_path, pregrasp_transformation = self.pre_grasp_to_grasp(ee_transformation)

            #get the joint positions using the inverse kinematics
            ee_pos = pregrasp_transformation[:3, 3]
            #convert transformation matrix to euler angles
            ee_euler = rotation_matrix_to_euler_angles(ee_transformation[:3, :3])
            ee_quat = self.pybullet_client.getQuaternionFromEuler(ee_euler)

            #get the joint positions using the inverse kinematics
            joint_positions = self.pybullet_client.calculateInverseKinematics(self.dummy_robot, 6, ee_pos, ee_quat, maxNumIterations=100000, residualThreshold=1e-10)
            joint_positions = list(joint_positions)

            
            for k in range(100):
                for i in range(1, 7):
                    self.pybullet_client.resetJointState(self.dummy_robot, i, initial_joint_positions[i-1]*joint_signs[i-1])

                for i in range(7, 19):
                    self.pybullet_client.resetJointState(self.dummy_robot, i, 0)
                self.move_gripper(0.08)
                self.pybullet_client.stepSimulation()


            # time.sleep(3)
            

            #compute the path from the current joint positions to the target joint positions
            ik_joints = get_movable_joints(self.dummy_robot)
            ik_joint_positions = []
            path = plan_joint_motion(self.dummy_robot, ik_joints, joint_positions, obstacles=[ self.plane, self.urdf_object_list[0]], self_collisions=False)


            #set to initial joint state
            for i in range(1, 7):
                self.pybullet_client.resetJointState(self.dummy_robot, i, initial_joint_positions[i-1]*joint_signs[i-1])

            if path is None:
                print("path not found")
                continue
            else:
                #append pregrasp to grasp path
                path = path + pre_grasp2grasp_path
                #make path+trajectory_count folder
                os.makedirs(self.path + str(self.trajectory_count), exist_ok=True)
                self.trajectory_length = 0

            k = 0
            loop_iters = 0
            current_iters = 0
            while k < len(path):
                print('k = ', k)
                print('current_iters', current_iters)
                error = 0

                self.current_gripper_action = 0

                for j in range(1, 7):
                    self.pybullet_client.setJointMotorControl2(self.dummy_robot, j, p.POSITION_CONTROL, targetPosition=path[k][j-1], force=250, maxVelocity=0.2)
                
                #get current joint positions
                joint_states = []
                for i in range(1, 7):
                    joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])
                
                error = np.linalg.norm(np.array(joint_states) - path[k][:6])

                self.move_gripper(0.085)

                if error < 1e-3:
                    k += 1
                    current_iters = 0

                
                current_iters += 1

                if current_iters > 200:
                    k += 1
                    current_iters = 0

                if loop_iters % 50 == 0:
                    self.trajectory_length += 1
                    #get observations
                    observations = self.get_observations()
                    #save the observations with trajectory length as pickle file in format 0000x.pkl
                    with open(self.path + str(self.trajectory_count) + '/' + str(self.trajectory_length).zfill(5) + '.pkl', 'wb') as f:
                        pickle.dump(observations, f)

                

                if loop_iters>10000:
                    break
                    

                self.pybullet_client.stepSimulation()
                # time.sleep(1/240.)

                loop_iters += 1

            
            print('reached the target joint positions')

            #apply force to the object
            for i in range(20):
                self.move_gripper(0.0)
                self.pybullet_client.stepSimulation()
                print('i', i)
                # time.sleep(1/240.)

                #get observations
                observations = self.get_observations()

                self.current_gripper_action = 1
                #save 
                if i % 5 == 0:
                    self.trajectory_length += 1
                    with open(self.path + str(self.trajectory_count) + '/' + str(self.trajectory_length).zfill(5) + '.pkl', 'wb') as f:
                        pickle.dump(observations, f)

                
            print('grasped the object')
            #go to pregrasp position
            grasp2pre_grasp_path = pre_grasp2grasp_path[::-1]
            k = 0
            loop_iters = 0
            current_iters = 0
            while k < len(grasp2pre_grasp_path):
                print('k = ', k)
                error = 0

                for j in range(1, 7):
                    self.pybullet_client.setJointMotorControl2(self.dummy_robot, j, p.POSITION_CONTROL, targetPosition=grasp2pre_grasp_path[k][j-1], force=250, maxVelocity=0.2)
                
                #get current joint positions
                joint_states = []
                for i in range(1, 7):
                    joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])

                for i in range(20):
                    self.move_gripper(0.0)
                
                error = np.linalg.norm(np.array(joint_states) - grasp2pre_grasp_path[k][:6])


                if error < 1e-2:
                    k += 1
                    current_iters = 0


                self.current_gripper_action = 1


                if loop_iters % 50 == 0:
                    self.trajectory_length += 1
                    #get observations
                    observations = self.get_observations()
                    #save the observations with trajectory length as pickle file in format 0000x.pkl
                    with open(self.path + str(self.trajectory_count) + '/' + str(self.trajectory_length).zfill(5) + '.pkl', 'wb') as f:
                        pickle.dump(observations, f)


                current_iters += 1
                loop_iters += 1

                if current_iters > 200:
                    k += 1
                    current_iters = 0


                if loop_iters>10000:
                    break


                self.pybullet_client.stepSimulation()
                # time.sleep(1/240.)

            #go to initial joint state with low velocity
            loop_iters = 0
            current_iters = 0
            while True:
                error = 0
                for j in range(1, 7):
                    self.pybullet_client.setJointMotorControl2(self.dummy_robot, j, p.POSITION_CONTROL, targetPosition=self.initial_joint_state[j-1], force=250, maxVelocity=0.3)
                
                #get current joint positions
                joint_states = []
                for i in range(1, 7):
                    joint_states.append(self.pybullet_client.getJointState(self.dummy_robot, i)[0])

                #keep gripper closed
                for i in range(20):
                    self.move_gripper(0.0)

                
                error = np.linalg.norm(np.array(joint_states) - initial_joint_state)

                if error < 1e-2:
                    break

                self.current_gripper_action = 1

                if loop_iters % 25 == 0:
                    self.trajectory_length += 1
                    #get observations
                    observations = self.get_observations()
                    #save the observations with trajectory length as pickle file in format 0000x.pkl
                    with open(self.path + str(self.trajectory_count) + '/' + str(self.trajectory_length).zfill(5) + '.pkl', 'wb') as f:
                        pickle.dump(observations, f)


                self.pybullet_client.stepSimulation()

                loop_iters += 1
                # time.sleep(1/240.)


            
            for i in range(5):
                for j in range(1, 7):
                    self.pybullet_client.setJointMotorControl2(self.dummy_robot, j, p.POSITION_CONTROL, targetPosition=self.initial_joint_state[j-1], force=250, maxVelocity=0.3)

                for l in range(20):
                    self.move_gripper(0.0)

                self.current_gripper_action = 1

                self.pybullet_client.stepSimulation()

                observations = self.get_observations()

                self.trajectory_length += 1
                with open(self.path + str(self.trajectory_count) + '/' + str(self.trajectory_length).zfill(5) + '.pkl', 'wb') as f:
                    pickle.dump(observations, f)

                # time.sleep(1/240.)
                


            #check z position of the object
            object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.urdf_object_list[0])
            if object_pos[2] < 0.2:
                print('object not picked')
                
                #delete the trajectory folder
                import shutil
                shutil.rmtree(self.path + str(self.trajectory_count))
                continue   

            self.trajectory_count += 1
                     
           

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


    def pre_grasp_to_grasp(self, transformation_matrix):

        #get the end effector position and orientation according to self.apple_grasp_pose
        ee_transformation =  transformation_matrix 

        #get approach vector
        approach_vector = ee_transformation[:3, :3][:, 1] / np.linalg.norm(ee_transformation[:3, :3][:, 1])

        #get the pre-grasp position
        pre_grasp_pos = ee_transformation[:3, 3] - 0.1 * approach_vector

        #pre-grasp transformation
        pre_grasp_transformation = np.eye(4)
        pre_grasp_transformation[:3, 3] = pre_grasp_pos
        pre_grasp_transformation[:3, :3] = ee_transformation[:3, :3]

        #get joint positions going from pre-grasp to grasp
        path = []
        for i in range(13):
            ee_pos = pre_grasp_pos + 0.01 * i * approach_vector
            ee_quat = self.pybullet_client.getQuaternionFromEuler(rotation_matrix_to_euler_angles(ee_transformation[:3, :3]))
            joint_positions = self.pybullet_client.calculateInverseKinematics(self.dummy_robot, 6, ee_pos, ee_quat, maxNumIterations=100000, residualThreshold=1e-10)
            path.append(joint_positions)

        return path, pre_grasp_transformation
