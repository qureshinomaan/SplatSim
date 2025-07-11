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
from pybullet_planning import plan_joint_motion, get_movable_joints, set_joint_positions
import random
from collections import namedtuple
import math

import os
import pickle
import shutil


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
        y = math.atan2(-R[2, 0], sy)  # Pitch
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
        # urdf_path: str = './pybullet-playground_2/urdf/sisbot.urdf',
        urdf_path: str = "./pybullet-playground_2/urdf/sisbot.urdf",
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
        self.pybullet_client.setAdditionalSearchPath(
            "./pybullet-playground_2/urdf/pybullet_ur5_gripper/urdf"
        )

        flags = self.pybullet_client.URDF_USE_IMPLICIT_CYLINDER
        self.dummy_robot = self.pybullet_client.loadURDF(
            self.urdf_path, [0, 0, -0.1], useFixedBase=True, flags=flags
        )

        self.skip_recording_first = 0

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
        self.offsets = [np.pi / 2, 0, 0, 0, 0, 0, 0]

        model_lib = md.model_lib()
        # objectid = self.pybullet_client.loadURDF(model_lib['potato_chip_1'], [0.5, 0.15, 0])

        # x = random.uniform(0.4, 0.6)
        # y = random.uniform(-0.3, 0.3)
        x = random.uniform(0.2, 0.7)
        y = random.uniform(-0.4, 0.4)
        # random euler angles for the orientation of the object
        euler_z = random.uniform(-np.pi, np.pi)
        # random quaternion for the orientation of the object
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, 0])

        models_lib = md.model_lib()
        self.object_name_list = ["plastic_orange", "plate"]
        self.splat_object_name_list = ["plastic_orange", "plate"]
        self.randomize_object_positions = [True, False]
        self.randomize_object_rotations = [False, True]
        self.rotation_values = [[0, 0], [-np.pi / 6, np.pi / 6]]
        self.use_fixed_base = [False, True]
        global_scaling_list = [1, 1]
        self.urdf_object_list = []
        for object_name in range(len(self.object_name_list)):
            if self.object_name_list[object_name] in models_lib.model_name_list:
                object_loaded = self.pybullet_client.loadURDF(
                    models_lib[self.object_name_list[object_name]],
                    [x, y, 0.0],
                    quat,
                    globalScaling=global_scaling_list[object_name],
                    useFixedBase=self.use_fixed_base[object_name],
                )
                self.urdf_object_list.append(object_loaded)
            else:
                object_path = (
                    "/home/nomaan/Desktop/corl24/virtual_objects/"
                    + self.object_name_list[object_name]
                    + "/object.urdf"
                )
                object_loaded = self.pybullet_client.loadURDF(
                    object_path,
                    [x, y, 0.0],
                    quat,
                    globalScaling=1,
                    useFixedBase=self.use_fixed_base[object_name],
                )

        # reset the box position
        self.pybullet_client.resetBasePositionAndOrientation(
            self.urdf_object_list[-1],
            [0.3, -0.5, 0.07],
            p.getQuaternionFromEuler([0, 0, np.pi / 2]),
        )

        # set the grasp pose for the apple and banana

        self.orange_grasp_pose = np.array(
            [
                [0.03420832, 0.29551898, 0.95472421, -0.08157158],
                [-0.82904722, 0.54187654, -0.13802362, -0.14110232],
                [-0.55813126, -0.7867899, 0.26353588, 0.20728098],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        self.banana_grasp_pose_1 = np.array(
            [
                [-0.13784676, -0.14873802, 0.97922177, 0.01055928],
                [-0.98239786, 0.14637033, -0.11606107, -0.06527538],
                [-0.12606632, -0.97798401, -0.16629659, 0.23013977],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        self.banana_grasp_pose_2 = np.array(
            [
                [0.12773567, 0.02665088, -0.99145011, 0.00692899],
                [-0.87105321, 0.481048, -0.09929316, -0.14203231],
                [0.47428884, 0.87628908, 0.08466133, -0.20627994],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        self.apple_grasp_pose = np.array(
            [
                [-0.12515046, -0.0412762, 0.99127879, 0.00471373],
                [-0.98896543, -0.07464537, -0.12796658, 0.01413896],
                [0.07927635, -0.99635553, -0.03147883, 0.27105228],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # self.strawberry_grasp_pose = np.array([[-0.19612399,  0.06661985,  0.97831344 ,-0.03194745],
        #                                 [-0.90997152, -0.38409934, -0.15626751,  0.10821076],
        #                                 [ 0.36535902, -0.92088517,  0.13595326,  0.23474673],
        #                                 [ 0.,          0. ,         0. ,         1.        ]])

        self.strawberry_grasp_pose = np.array(
            [
                [6.03600159e-04, 4.74883933e-01, 8.80048229e-01, -1.17034260e-01],
                [-7.31850150e-01, -5.99512796e-01, 3.24005810e-01, 1.57542460e-01],
                [6.81465328e-01, -6.44258999e-01, 3.47182012e-01, 1.72402069e-01],
                [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )

        self.grasp_poses = [
            self.orange_grasp_pose,
        ]

        # set the drop location for the apple and banana
        self.drop_ee_pos = [0.3, -0.5, 0.3]
        self.drop_ee_euler = [-np.pi / 2, 0, -np.pi / 2]
        self.drop_ee_quat = self.pybullet_client.getQuaternionFromEuler(
            self.drop_ee_euler
        )

        # set initial joint positions
        initial_joint_state = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        for i in range(1, 7):
            self.pybullet_client.resetJointState(
                self.dummy_robot, i, initial_joint_state[i - 1]
            )

        # limits are +-pi of the initial joint positions
        lower_limits = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
        upper_limits = [np.pi, 0, np.pi, np.pi, np.pi, np.pi]
        self.drop_ee_joint = self.pybullet_client.calculateInverseKinematics(
            self.dummy_robot,
            6,
            self.drop_ee_pos,
            self.drop_ee_quat,
            maxNumIterations=100000,
            residualThreshold=1e-10,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
        )

        print("drop_ee_joint", self.drop_ee_joint)

        # set the joint positions to the drop location

        for i in range(1, 7):
            self.pybullet_client.resetJointState(
                self.dummy_robot, i, self.drop_ee_joint[i - 1]
            )

        # change the friction of the object
        for T_object in self.urdf_object_list:
            self.pybullet_client.changeDynamics(T_object, -1, lateralFriction=1.5)
            # rolling friction
            self.pybullet_client.changeDynamics(T_object, -1, rollingFriction=0)
            inertia = p.getDynamicsInfo(T_object, -1)[2]

        # add gravity
        self.pybullet_client.setGravity(0, 0, -9.81)

        # add plane
        import pybullet_data

        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = self.pybullet_client.loadURDF("plane.urdf", [0, 0, -0.022])

        # place a wall in -0.4 at x axis using plane.urdf
        # wall is perpendicular to the plane
        quat = self.pybullet_client.getQuaternionFromEuler([0, np.pi / 2, 0])
        self.wall = self.pybullet_client.loadURDF("plane.urdf", [-0.4, 0, 0.0], quat)

        ##add a spher at 0.4, 0.5 0.01 without urdf
        # self.sphere = self.pybullet_client.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1], specularColor=[0.4, .4, 0], visualFramePosition=[0.4, 0.6, 0.01])
        # self.sphere_id = self.pybullet_client.createMultiBody(baseMass=0, baseVisualShapeIndex=self.sphere)

        ## add stage
        self.stage = 0

        # change the friction of the plane
        # self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.1))

        # set time step
        self.pybullet_client.setTimeStep(1 / 240)
        # self.pybullet_client.setRealTimeSimulation(0)

        # current gripper state
        self.current_gripper_action = 0

        # trajectory path
        self.path = "/home/jennyw2/data/bc_data/gello/"
        # get no of folders in the path
        self.trajectory_count = len(os.listdir(self.path))

        # self.trajectory_count =

        # step simulation
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
            joint_states.append(
                self.pybullet_client.getJointState(self.dummy_robot, i)[0]
            )
        return np.array(joint_states)

    def get_joint_state_dummy(self) -> np.ndarray:
        # return self._joint_state
        joint_states = []
        num_joints = self.pybullet_client.getNumJoints(self.dummy_robot)
        for i in range(1, num_joints):
            joint_states.append(
                self.pybullet_client.getJointState(self.dummy_robot, i)[0]
            )
        return np.array(joint_states)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )

        for i in range(1, 7):
            # self.pybullet_client.resetJointState(self.robot, i, joint_state[i-1])
            self.pybullet_client.setJointMotorControl2(
                self.dummy_robot,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_state[i - 1],
                force=250,
            )

        if self.use_gripper:
            self.move_gripper((1 - joint_state[-1]) * 0.085)

            self.current_gripper_action = joint_state[-1]

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self.get_joint_state()
        joint_positions_dummy = self.get_joint_state_dummy()
        joint_velocities = np.array(
            [
                self.pybullet_client.getJointState(self.dummy_robot, i)[1]
                for i in range(7)
            ]
        )

        dummy_ee_pos, dummy_ee_quat = (
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[0],
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[1],
        )
        # get the euler angles from the quaternion
        dummy_ee_euler = self.pybullet_client.getEulerFromQuaternion(dummy_ee_quat)

        # get quaternion from euler angles
        dummy_ee_quat_reconstruct = self.pybullet_client.getQuaternionFromEuler(
            dummy_ee_euler
        )

        # print the euler angles and the reconstructed quaternion
        self.current_gripper_state = self.get_current_gripper_state() / 0.8

        # combine the position and euler angles and self.current_gripper_state to get the state
        state = np.concatenate(
            [dummy_ee_pos, dummy_ee_euler, [self.current_gripper_state]]
        )
        action = np.concatenate(
            [dummy_ee_pos, dummy_ee_euler, [self.current_gripper_action]]
        )

        # Target object position and orientation

        observations = {
            "joint_positions": joint_positions[:7],
            "all_joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_positions_dummy": joint_positions_dummy,
            "state": state,
            "action": action,
        }

        for i in range(len(self.urdf_object_list)):
            (
                object_pos,
                object_quat,
            ) = self.pybullet_client.getBasePositionAndOrientation(
                self.urdf_object_list[i]
            )
            observations[self.splat_object_name_list[i] + "_position"] = object_pos
            observations[self.splat_object_name_list[i] + "_orientation"] = object_quat

        return observations

    def randomize_object_pose(self):
        collison_between_objects = True
        while collison_between_objects:
            collison_between_objects = False
            for object_id in range(len(self.urdf_object_list)):
                if self.randomize_object_positions[object_id]:
                    # randomly reset the object position and orientation
                    x = random.uniform(0.2, 0.6)
                    y = random.uniform(-0.5, 0.5)
                    # random euler angles for the orientation of the object
                    euler_z = random.uniform(
                        self.rotation_values[object_id][0],
                        self.rotation_values[object_id][1],
                    )
                    # random quaternion for the orientation of the object
                    # get object name from the object id
                    object_name = self.object_name_list[object_id]
                    if object_name == "plastic_banana":
                        flip = random.choice([0, 1])
                        if flip == 1:
                            self.grasp_poses[object_id] = self.banana_grasp_pose_2
                            quat = self.pybullet_client.getQuaternionFromEuler(
                                [0, np.pi, euler_z]
                            )
                            self.pybullet_client.resetBasePositionAndOrientation(
                                self.urdf_object_list[object_id], [x, y, 0], quat
                            )

                        else:
                            self.grasp_poses[object_id] = self.banana_grasp_pose_1
                            quat = self.pybullet_client.getQuaternionFromEuler(
                                [0, 0, euler_z]
                            )
                            self.pybullet_client.resetBasePositionAndOrientation(
                                self.urdf_object_list[object_id], [x, y, 0], quat
                            )

                    else:
                        quat = self.pybullet_client.getQuaternionFromEuler(
                            [0, 0, euler_z]
                        )
                        self.pybullet_client.resetBasePositionAndOrientation(
                            self.urdf_object_list[object_id], [x, y, 0], quat
                        )

            for object_id in range(len(self.urdf_object_list)):
                for object_id_2 in range(len(self.urdf_object_list)):
                    if object_id != object_id_2:
                        collison_between_objects_1 = pairwise_collision(
                            self.urdf_object_list[object_id],
                            self.urdf_object_list[object_id_2],
                        )
                        if collison_between_objects_1:
                            collison_between_objects = True
                            break

    def randomize_plate_and_drop_pose(self):
        # randomize plate and drop location
        # [0.3, -0.5, 0.07]
        while True:
            x = random.uniform(0.2, 0.8)
            y = random.uniform(-0.4, 0.4)
            z = 0.0

            # get obj[0] position
            (
                object_pos,
                object_quat,
            ) = self.pybullet_client.getBasePositionAndOrientation(
                self.urdf_object_list[0]
            )

            euler_z = 0
            quat = self.pybullet_client.getQuaternionFromEuler([0, 0, euler_z])

            self.pybullet_client.resetBasePositionAndOrientation(
                self.urdf_object_list[-1], [x, y, z], quat
            )

            self.drop_ee_pos = [x, y, 0.3]

            # calculate the drop ee joint
            self.drop_ee_joint = self.pybullet_client.calculateInverseKinematics(
                self.dummy_robot,
                6,
                self.drop_ee_pos,
                self.drop_ee_quat,
                maxNumIterations=100000,
            )

            # check the distance between the object and the drop location
            if np.linalg.norm(np.array(object_pos)[:2] - np.array([x, y])) > 0.2:
                break

    def get_random_ee_pose(self):
        # random end effector position
        if random.uniform(0, 1) > 0.2:
            random_ee_pos = np.array(
                [
                    random.uniform(0.2, 0.5),
                    random.uniform(-0.6, 0.6),
                    random.uniform(0.25, 0.65),
                ]
            )
        else:
            # get object position
            (
                object_pos,
                object_quat,
            ) = self.pybullet_client.getBasePositionAndOrientation(
                self.urdf_object_list[0]
            )
            random_x = random.uniform(-0.1, 0.1)
            random_y = random.uniform(-0.1, 0.1)
            random_x = 0.05 * random_x / np.abs(random_x) + random_x
            random_y = 0.05 * random_y / np.abs(random_y) + random_y
            random_ee_pos = np.array(
                [
                    object_pos[0] + random_x,
                    object_pos[1] + random_y,
                    object_pos[2] + random.uniform(0.25, 0.3),
                ]
            )
        # random_ee_pos = np.array([random.uniform(0.2, 0.5), random.uniform(-0.6, 0.6), random.uniform(0.2, 0.65)])

        # get the euler angles from the quaternion
        # get quaternion from euler angles
        random_ee_quat = self.initial_ee_quat

        return random_ee_pos, random_ee_quat

    def plan_approach_grasp_move_drop_plan(self, initial_joint_positions):
        all_paths = []

        object_order = [0]
        for object in object_order:

            # get object position
            (
                object_pos,
                object_quat,
            ) = self.pybullet_client.getBasePositionAndOrientation(
                self.urdf_object_list[object]
            )

            # create transformation matrix from the object position and orientation
            object_transformation = np.eye(4)
            object_transformation[:3, :3] = np.array(
                self.pybullet_client.getMatrixFromQuaternion(object_quat)
            ).reshape(3, 3)
            object_transformation[:3, 3] = np.array(object_pos)

            # get the end effector position and orientation according to self.apple_grasp_pose
            ee_transformation = object_transformation @ self.grasp_poses[object]

            # get pregrasp to grasp path
            pre_grasp2grasp_path, pregrasp_transformation = self.pre_grasp_to_grasp(
                ee_transformation
            )
            if pre_grasp2grasp_path is None:
                break

            # get the joint positions using the inverse kinematics
            ee_pos = pregrasp_transformation[:3, 3]
            # convert transformation matrix to euler angles
            ee_euler = rotation_matrix_to_euler_angles(ee_transformation[:3, :3])
            ee_quat = self.pybullet_client.getQuaternionFromEuler(ee_euler)

            # get the joint positions using the inverse kinematics
            joint_positions = self.pybullet_client.calculateInverseKinematics(
                self.dummy_robot,
                6,
                ee_pos,
                ee_quat,
                maxNumIterations=100000,
                residualThreshold=1e-10,
            )
            joint_positions = list(joint_positions)

            # compute the path from the current joint positions to the target joint positions
            ik_joints = get_movable_joints(self.dummy_robot)
            ik_joint_positions = []
            path = plan_joint_motion(
                self.dummy_robot,
                ik_joints,
                joint_positions,
                obstacles=[
                    self.plane,
                    self.urdf_object_list[0],
                    self.urdf_object_list[1],
                ],
                self_collisions=False,
            )

            # set the joints to the last joint positions of path
            if path is not None:
                # reset the joint positions to the initial joint positions
                for i in range(1, 7):
                    self.pybullet_client.resetJointState(
                        self.dummy_robot, i, path[0][i - 1]
                    )

            else:
                break

            if path is not None:
                all_paths.append(path)

            all_paths.append(pre_grasp2grasp_path)

            all_paths.append(pre_grasp2grasp_path[::-1])

            # set the joint angle to pre_grasp2grasp_path[0]
            for i in range(1, 7):
                self.pybullet_client.resetJointState(
                    self.dummy_robot, i, pre_grasp2grasp_path[0][i - 1]
                )

            # now plan the path from pre_grasp to intermediate position
            ee_pos = self.pybullet_client.getLinkState(self.dummy_robot, 6)[0]
            intermediate_ee_pos = [ee_pos[0], ee_pos[1], 0.4]
            intermediate_ee_quat = self.initial_ee_quat
            intermediate_joint_positions = (
                self.pybullet_client.calculateInverseKinematics(
                    self.dummy_robot,
                    6,
                    intermediate_ee_pos,
                    intermediate_ee_quat,
                    maxNumIterations=100000,
                    residualThreshold=1e-10,
                )
            )

            # compute the path from the current joint positions to the target joint positions
            path = plan_joint_motion(
                self.dummy_robot,
                ik_joints,
                intermediate_joint_positions,
                obstacles=[self.plane, self.urdf_object_list[-1]],
                self_collisions=False,
            )

            # set the joints to the last joint positions of path
            if path is not None:
                # reset the joint positions to the initial joint positions
                for i in range(1, 7):
                    self.pybullet_client.resetJointState(
                        self.dummy_robot, i, path[-1][i - 1]
                    )

                all_paths.append(path)

            # now plan the path from intermediate to drop location
            path = plan_joint_motion(
                self.dummy_robot,
                ik_joints,
                self.drop_ee_joint,
                obstacles=[self.plane, self.urdf_object_list[-1]],
                self_collisions=False,
            )

            # set the joints to the last joint positions of path
            if path is not None:
                # reset the joint positions to the initial joint positions
                for i in range(1, 7):
                    self.pybullet_client.resetJointState(
                        self.dummy_robot, i, path[-1][i - 1]
                    )

                all_paths.append(path)

                # TODO why is this here. Can it be moved outside?
                if self.skip_recording_first:
                    for i in range(1, 7):
                        self.pybullet_client.resetJointState(
                            self.dummy_robot, i, initial_joint_positions[i - 1]
                        )

        return all_paths

    def follow_paths_and_record(
        self, all_paths, initial_joint_positions, initial_joint_state, joint_signs
    ):
        for k in range(1):
            # go to first object
            self.follow_trajectory(
                all_paths[4 * k + 0], 0, gripper_velocity=0.2, threshold=0.001
            )

            self.follow_trajectory(
                all_paths[4 * k + 1], 0, gripper_velocity=0.2, threshold=0.001
            )

            # grasp the first object
            for i in range(160):
                self.move_gripper(0.084)
                self.pybullet_client.stepSimulation()

                self.current_gripper_action = 1
                observations = self.get_observations()
                # save the observations in the correct format zfill(5)
                if i % 20 == 0 and not self.skip_recording_first:
                    self.trajectory_length += 1
                    with open(
                        self.path
                        + str(self.trajectory_count).zfill(3)
                        + "/"
                        + str(self.trajectory_length).zfill(5)
                        + ".pkl",
                        "wb",
                    ) as f:
                        pickle.dump(observations, f)

            # go to pregrasp
            self.follow_trajectory(all_paths[4 * k + 2], 1)

            # go to intermediate
            self.follow_trajectory(all_paths[4 * k + 3], 1)

            # go to drop location
            self.follow_trajectory(all_paths[4 * k + 4], 1)

            # drop the object
            for i in range(160):
                self.move_gripper(0)
                self.pybullet_client.stepSimulation()

                self.current_gripper_action = 0
                observations = self.get_observations()
                # save the observations
                if i % 20 == 0 and not self.skip_recording_first:
                    self.trajectory_length += 1
                    with open(
                        self.path
                        + str(self.trajectory_count).zfill(3)
                        + "/"
                        + str(self.trajectory_length).zfill(5)
                        + ".pkl",
                        "wb",
                    ) as f:
                        pickle.dump(observations, f)

            if self.skip_recording_first:
                for i in range(100):
                    self.pybullet_client.stepSimulation()
                    # time.sleep(1/240)
                    self.move_gripper(0.0)
                    for k in range(1, 7):
                        self.pybullet_client.resetJointState(
                            self.dummy_robot,
                            k,
                            initial_joint_positions[k - 1] * joint_signs[k - 1],
                        )

        # TODO put this in all_paths
        # create a path from the drop location to the initial joint positions
        path = [
            np.array(self.drop_ee_joint[:6]) * 0.1 * (10 - i)
            + np.array(initial_joint_state[:6]) * 0.1 * (i)
            for i in range(1, 11)
        ]

        self.follow_trajectory(path, 0)

        path = [initial_joint_state for _ in range(5)]
        self.follow_trajectory(path, 0)

    def eval_trajectory_success(self):
        # check the mse of xy position of the objects with the drop location
        for i in range(len(self.urdf_object_list) - 1):
            object_pos, _ = self.pybullet_client.getBasePositionAndOrientation(
                self.urdf_object_list[i]
            )
            mse = (object_pos[0] - self.drop_ee_pos[0]) ** 2 + (
                object_pos[1] - self.drop_ee_pos[1]
            ) ** 2

            if mse > 0.03:
                print("object not placed correctly")
                return False
        return True

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        initial_joint_state = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        self.initial_joint_state = initial_joint_state
        joint_signs = [1, 1, 1, 1, 1, 1]
        for i in range(1, 7):
            self.pybullet_client.resetJointState(
                self.dummy_robot, i, initial_joint_state[i - 1] * joint_signs[i - 1]
            )
            self.move_gripper(0.0)

        # get initial ee position and orientation
        self.initial_ee_pos, self.initial_ee_quat = (
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[0],
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[1],
        )
        # print joint angles
        joint_states = []
        for i in range(1, 19):
            joint_states.append(
                self.pybullet_client.getJointState(self.dummy_robot, i)[0]
            )

        # set to initial joint state
        for i in range(10000):
            for i in range(1, 7):
                self.pybullet_client.resetJointState(
                    self.dummy_robot, i, initial_joint_state[i - 1] * joint_signs[i - 1]
                )
            self.pybullet_client.stepSimulation()

        while True:
            # self.get_camera_image_from_end_effector()
            self.randomize_object_pose()
            self.randomize_plate_and_drop_pose()

            # Let the simulation settle
            for i in range(10000):
                self.pybullet_client.stepSimulation()
                self.move_gripper(0.084)
                for k in range(1, 7):
                    self.pybullet_client.resetJointState(
                        self.dummy_robot,
                        k,
                        initial_joint_state[k - 1] * joint_signs[k - 1],
                    )
            self.pybullet_client.stepSimulation()

            # make path+trajectory_count folder
            os.makedirs(self.path + str(self.trajectory_count).zfill(3), exist_ok=True)
            self.trajectory_length = 0

            # generating random initial joint state using random end effector position and orientation
            random_ee_pos, random_ee_quat = self.get_random_ee_pose()

            # joint angles using inverse kinematics
            initial_joint_positions = self.pybullet_client.calculateInverseKinematics(
                self.dummy_robot,
                6,
                random_ee_pos,
                random_ee_quat,
                maxNumIterations=100000,
                residualThreshold=1e-10,
            )

            # reset the joint positions to the initial joint positions
            for i in range(1, 7):
                self.pybullet_client.resetJointState(
                    self.dummy_robot, i, initial_joint_positions[i - 1]
                )

            all_paths = self.plan_approach_grasp_move_drop_plan(initial_joint_positions)

            for i in range(100):
                self.pybullet_client.stepSimulation()
                # time.sleep(1/240)
                self.move_gripper(0.0)
                for k in range(1, 7):
                    self.pybullet_client.resetJointState(
                        self.dummy_robot,
                        k,
                        initial_joint_positions[k - 1] * joint_signs[k - 1],
                    )

            if len(all_paths) != 5:
                self.delete_trajectory_folder()
                continue

            self.follow_paths_and_record(
                all_paths, initial_joint_positions, initial_joint_state, joint_signs
            )

            # evaluate the success of the trajectory
            correct_trajectory = self.eval_trajectory_success()

            if correct_trajectory:
                self.trajectory_count += 1

                if self.trajectory_count > 500:
                    exit()
            else:
                self.delete_trajectory_folder()
                break

    def delete_trajectory_folder(self):
        shutil.rmtree(self.path + str(self.trajectory_count).zfill(3))

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None:
        self.stop()

    def __parse_joint_info__(self):
        numJoints = p.getNumJoints(self.dummy_robot)
        jointInfo = namedtuple(
            "jointInfo",
            [
                "id",
                "name",
                "type",
                "damping",
                "friction",
                "lowerLimit",
                "upperLimit",
                "maxForce",
                "maxVelocity",
                "controllable",
            ],
        )
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):
            info = p.getJointInfo(self.dummy_robot, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[
                2
            ]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = jointType != p.JOINT_FIXED
            if controllable:
                self.controllable_joints.append(jointID)
                self.pybullet_client.setJointMotorControl2(
                    self.dummy_robot,
                    jointID,
                    self.pybullet_client.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=0,
                )
            info = jointInfo(
                jointID,
                jointName,
                jointType,
                jointDamping,
                jointFriction,
                jointLowerLimit,
                jointUpperLimit,
                jointMaxForce,
                jointMaxVelocity,
                controllable,
            )
            self.joints.append(info)

    def setup_gripper(self):
        self.__parse_joint_info__()
        self.gripper_range = [0, 0.085]

        mimic_parent_name = "finger_joint"
        mimic_children_names = {
            "right_outer_knuckle_joint": 1,
            "left_inner_knuckle_joint": 1,
            "right_inner_knuckle_joint": 1,
            "left_inner_finger_joint": -1,
            "right_inner_finger_joint": -1,
        }
        # self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)

        self.mimic_parent_id = [
            joint.id for joint in self.joints if joint.name == mimic_parent_name
        ][0]
        self.mimic_child_multiplier = {
            joint.id: mimic_children_names[joint.name]
            for joint in self.joints
            if joint.name in mimic_children_names
        }

        for joint_id, multiplier in self.mimic_child_multiplier.items():
            c = self.pybullet_client.createConstraint(
                self.dummy_robot,
                self.mimic_parent_id,
                self.dummy_robot,
                joint_id,
                jointType=self.pybullet_client.JOINT_GEAR,
                jointAxis=[0, 1, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0],
            )
            self.pybullet_client.changeConstraint(
                c, gearRatio=-multiplier, maxForce=10, erp=1
            )  # Note: the mysterious `erp` is of EXTREME importance

    def get_link_pose(self, body, link):
        result = self.pybullet_client.getLinkState(body, link)
        return result[4], result[5]

    def move_gripper(self, open_length, velocity=2):
        # open_length = np.clip(open_length, *self.gripper_range)
        open_angle = 0.715 - math.asin(
            (open_length - 0.010) / 0.1143
        )  # angle calculation
        # Control the mimic gripper joint(s)
        p.setJointMotorControl2(
            self.dummy_robot,
            self.mimic_parent_id,
            self.pybullet_client.POSITION_CONTROL,
            targetPosition=open_angle,
            force=self.joints[self.mimic_parent_id].maxForce,
            maxVelocity=velocity,
        )

    def get_current_gripper_state(self):
        return self.pybullet_client.getJointState(
            self.dummy_robot, self.mimic_parent_id
        )[0]

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
        camera_target_position_world = end_effector_pos + rot_matrix.dot(
            camera_target_position
        )

        # Compute the view matrix
        view_matrix = self.pybullet_client.computeViewMatrix(
            camera_position_world, camera_target_position_world, [0, 0, 1]
        )

        # Compute the projection matrix
        projection_matrix = self.pybullet_client.computeProjectionMatrixFOV(
            cam_fov, 1.0, near_plane, far_plane
        )

        # Get the camera image
        (
            width,
            height,
            rgb_img,
            depth_img,
            seg_img,
        ) = self.pybullet_client.getCameraImage(
            320,
            240,
            view_matrix,
            projection_matrix,
            flags=self.pybullet_client.ER_NO_SEGMENTATION_MASK,
        )

        return rgb_img

    def pre_grasp_to_grasp(self, transformation_matrix):

        # get the end effector position and orientation according to self.apple_grasp_pose
        ee_transformation = transformation_matrix

        # get approach vector
        approach_vector = ee_transformation[:3, :3][:, 1] / np.linalg.norm(
            ee_transformation[:3, :3][:, 1]
        )

        # check the angle of the approach vector with the -ve z axis
        angle = np.arccos(np.dot(approach_vector, np.array([0, 0, -1])))
        if angle > 0.1 or angle < -0.1:
            print("angle is greater than 0.1")
            return None, None

        # check the angle of the approach vector with the y axis
        angle = np.arccos(np.dot(approach_vector, np.array([0, 1, 0])))
        if np.abs(angle - np.pi / 2) > 0.1:
            print("angle is greater than 0.1")
            return None, None

        # check the angle of the approach vector with the x axis
        angle = np.arccos(np.dot(approach_vector, np.array([1, 0, 0])))
        print(angle)

        # get the pre-grasp position
        pre_grasp_pos = ee_transformation[:3, 3] - 0.1 * approach_vector

        # pre-grasp transformation
        pre_grasp_transformation = np.eye(4)
        pre_grasp_transformation[:3, 3] = pre_grasp_pos
        pre_grasp_transformation[:3, :3] = ee_transformation[:3, :3]

        # get joint positions going from pre-grasp to grasp
        path = []
        for i in range(11):
            ee_pos = pre_grasp_pos + 0.01 * i * approach_vector
            ee_quat = self.pybullet_client.getQuaternionFromEuler(
                rotation_matrix_to_euler_angles(ee_transformation[:3, :3])
            )
            joint_positions = self.pybullet_client.calculateInverseKinematics(
                self.dummy_robot,
                6,
                ee_pos,
                ee_quat,
                maxNumIterations=100000,
                residualThreshold=1e-10,
            )
            path.append(joint_positions)

        return path, pre_grasp_transformation

    def follow_trajectory(
        self,
        path,
        gripper_pos,
        use_current_iters=True,
        gripper_velocity=2,
        threshold=1e-2,
    ):
        # Note: This function also saves observations
        k = 0
        loop_iters = 0
        current_iters = 0
        while k < len(path):
            error = 0

            for j in range(1, 7):
                self.pybullet_client.setJointMotorControl2(
                    self.dummy_robot,
                    j,
                    p.POSITION_CONTROL,
                    targetPosition=path[k][j - 1],
                    force=250,
                    maxVelocity=0.2,
                )

            # get current joint positions
            joint_states = []
            for i in range(1, 7):
                joint_states.append(
                    self.pybullet_client.getJointState(self.dummy_robot, i)[0]
                )

            error = np.linalg.norm(np.array(joint_states) - path[k][:6])

            self.move_gripper((1 - gripper_pos) * 0.084, velocity=gripper_velocity)

            if error < threshold:
                k += 1
                current_iters = 0

            current_iters += 1

            if use_current_iters:
                if current_iters > 200:
                    k += 1
                    current_iters = 0

            self.current_gripper_action = gripper_pos
            if loop_iters % 50 == 0 and not self.skip_recording_first:
                self.trajectory_length += 1
                # get observations
                observations = self.get_observations()
                # save the observations with trajectory length as pickle file in format 0000x.pkl
                with open(
                    self.path
                    + str(self.trajectory_count).zfill(3)
                    + "/"
                    + str(self.trajectory_length).zfill(5)
                    + ".pkl",
                    "wb",
                ) as f:
                    pickle.dump(observations, f)

            if loop_iters > 10000:
                break

            self.pybullet_client.stepSimulation()

            loop_iters += 1

        if not self.skip_recording_first:

            self.trajectory_length += 1
            # get observations
            observations = self.get_observations()
            # save the observations with trajectory length as pickle file in format 0000x.pkl
            with open(
                self.path
                + str(self.trajectory_count).zfill(3)
                + "/"
                + str(self.trajectory_length).zfill(5)
                + ".pkl",
                "wb",
            ) as f:
                pickle.dump(observations, f)
