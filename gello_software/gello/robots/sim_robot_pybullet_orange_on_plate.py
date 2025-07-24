import pickle
import threading
import time
from typing import Any, Dict, Optional, List
import enum
import copy
import random
from collections import namedtuple
import math
import os
import pickle
import shutil
import yaml
from argparse import ArgumentParser

import torch
import numpy as np
import mujoco
import mujoco.viewer
import zmq
from dm_control import mjcf
from gello.robots.robot import Robot

import cv2

assert mujoco.viewer is mujoco.viewer
from scene.cameras import Camera
from gaussian_renderer import render
import urdf_models.models_data as md
import pybullet as p
from pybullet_planning.interfaces.robots.collision import pairwise_collision
from pybullet_planning import plan_joint_motion, get_movable_joints, set_joint_positions
from utils.robot_splat_render_utils import (
    get_segmented_indices,
    transform_means,
    get_transfomration_list,
    transform_object,
    get_curr_link_states,
)
from gaussian_splatting.gaussian_renderer import GaussianModel
from gaussian_splatting.arguments import ModelParams, PipelineParams, Namespace
from scene import Scene


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
                print(f"Received request: {method}, {args}")
                if not self._robot.ready_to_serve:
                    result = {"error": "Robot not ready to serve"}
                    print(result)
                    self._socket.send(pickle.dumps(result))
                    continue
                if method == "num_dofs":
                    result = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "set_object_pose":
                    result = self._robot.set_object_pose(**args)
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
                pass
                # print("Timeout in ZMQLeaderServer serve")
                # Timeout occurred, check if the stop event is set

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()
        self._context.term()


class PybulletRobotServer:
    MAX_TRAJECTORY_COUNT = 500

    # Enum for serve modes
    class SERVE_MODES(enum.Enum):
        GENERATE_DEMOS = "generate_demos"
        INTERACTIVE = "interactive"

    # object_rot is only x and y. Since it's a tabletop, z is randomized
    GRASP_CONFIGS = {
        "orange": {
            "grasp_pose": np.array(
                [
                    [0.03420832, 0.29551898, 0.95472421, -0.08157158],
                    [-0.82904722, 0.54187654, -0.13802362, -0.14110232],
                    [-0.55813126, -0.7867899, 0.26353588, 0.20728098],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            "object_rot": [0, 0],
        },
        "banana1": {
            "grasp_pose": np.array(
                [
                    [-0.13784676, -0.14873802, 0.97922177, 0.01055928],
                    [-0.98239786, 0.14637033, -0.11606107, -0.06527538],
                    [-0.12606632, -0.97798401, -0.16629659, 0.23013977],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            "object_rot": [0, 0],
        },
        "banana2": {
            "grasp_pose": np.array(
                [
                    [0.12773567, 0.02665088, -0.99145011, 0.00692899],
                    [-0.87105321, 0.481048, -0.09929316, -0.14203231],
                    [0.47428884, 0.87628908, 0.08466133, -0.20627994],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            "object_rot": [0, np.pi],
        },
        "apple": {
            "grasp_pose": np.array(
                [
                    [-0.12515046, -0.0412762, 0.99127879, 0.00471373],
                    [-0.98896543, -0.07464537, -0.12796658, 0.01413896],
                    [0.07927635, -0.99635553, -0.03147883, 0.27105228],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
            "object_rot": [0, 0],
        },
        # self.strawberry_grasp_pose = np.array([[-0.19612399,  0.06661985,  0.97831344 ,-0.03194745],
        #                                 [-0.90997152, -0.38409934, -0.15626751,  0.10821076],
        #                                 [ 0.36535902, -0.92088517,  0.13595326,  0.23474673],
        #                                 [ 0.,          0. ,         0. ,         1.        ]])
        "strawberry": {
            "grasp_pose": np.array(
                [
                    [6.03600159e-04, 4.74883933e-01, 8.80048229e-01, -1.17034260e-01],
                    [-7.31850150e-01, -5.99512796e-01, 3.24005810e-01, 1.57542460e-01],
                    [6.81465328e-01, -6.44258999e-01, 3.47182012e-01, 1.72402069e-01],
                    [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
                ]
            ),
            "object_rot": [0, 0],
        },
    }

    ENV_NAME_TO_CONFIG = {
        "apple_on_plate": {
            "objects": [
                {
                    "object_name": "plastic_apple",
                    "splat_object_name": "plastic_apple",
                    "grasp_config": [GRASP_CONFIGS["apple"]],
                },
                {
                    "object_name": "plate",
                    "splat_object_name": "plate",
                    "grasp_config": [],
                },
            ]
        },
        "banana_on_plate": {
            "objects": [
                {
                    "object_name": "plastic_banana",
                    "splat_object_name": "plastic_banana",
                    "grasp_config": [
                        GRASP_CONFIGS["banana1"],
                        GRASP_CONFIGS["banana2"],
                    ],
                },
                {
                    "object_name": "plate",
                    "splat_object_name": "plate",
                    "grasp_config": [],
                },
            ]
        },
        "orange_on_plate": {
            "objects": [
                {
                    "object_name": "plastic_orange",
                    "splat_object_name": "plastic_orange",
                    "grasp_config": [GRASP_CONFIGS["orange"]],
                },
                {
                    "object_name": "plate",
                    "splat_object_name": "plate",
                    "grasp_config": [],
                },
            ]
        },
        # TODO is there a plastic strawberry?
    }

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
        use_gripper: bool = True,
        serve_mode: str = SERVE_MODES.GENERATE_DEMOS,
        env_config_name: str = "orange_on_plate",
        use_link_centers: bool = True,
        robot_name: str = "robot_iphone",
        render_camera_image: bool = True,
        cam_i: int = 254,
        object_config_path: str = "./object_configs/objects.yaml",
    ):
        self.ready_to_serve = False
        self.serve_mode = serve_mode
        self.use_link_centers = use_link_centers
        self.robot_name = robot_name
        self.env_config_name = env_config_name
        self.render_camera_image = render_camera_image
        self.cam_i = cam_i
        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)
        self.pybullet_client = p
        self.object_config_path = object_config_path
        self.grasp_poses = {}
        self.pybullet_client.connect(p.GUI)
        self.pybullet_client.setAdditionalSearchPath(
            "./pybullet-playground_2/urdf/pybullet_ur5_gripper/urdf"
        )

        with open(self.object_config_path, "r") as file:
            self.object_config = yaml.safe_load(file)

        urdf_path = self.object_config[self.robot_name]["urdf_path"][0]
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        base_position = self.object_config[self.robot_name]["base_position"][0]

        flags = self.pybullet_client.URDF_USE_IMPLICIT_CYLINDER
        self.dummy_robot = self.pybullet_client.loadURDF(
            urdf_path, useFixedBase=True, basePosition=base_position, flags=flags
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

        # self.offsets = [np.pi / 2, 0, 0, 0, 0, 0, 0]
        # This has an extra 0 at the beginning for the world joint, and then another 0 for a fixed joint, I think
        self.initial_joint_state = self.object_config[self.robot_name]["joint_states"][
            0
        ]
        # Remove the extra 0 for the world joint
        self.initial_joint_state = self.initial_joint_state[1:]
        # + 1 joint for the world joint at the beginning which will be skipped
        num_joints = self.pybullet_client.getNumJoints(self.dummy_robot)
        if len(self.initial_joint_state) > num_joints:
            print(
                f"Warning: Provided initial joint positions ({len(self.initial_joint_state)}) exceed the number of joints ({num_joints}). Truncating to {num_joints} positions."
            )
            self.initial_joint_state = self.initial_joint_state[:num_joints]
        # This is a no-op
        self.joint_signs = [1] * len(self.initial_joint_state)

        # self.initial_joint_state = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]

        # model_lib = md.model_lib()
        # objectid = self.pybullet_client.loadURDF(model_lib['potato_chip_1'], [0.5, 0.15, 0])

        x = random.uniform(0.2, 0.7)
        y = random.uniform(-0.4, 0.4)
        # random euler angles for the orientation of the object
        # euler_z = random.uniform(-np.pi, np.pi)
        # random quaternion for the orientation of the object
        quat = self.pybullet_client.getQuaternionFromEuler([0, 0, 0])

        models_lib = md.model_lib()
        self.object_name_list = list(
            map(
                lambda object_cfg: object_cfg["object_name"],
                self.ENV_NAME_TO_CONFIG[env_config_name]["objects"],
            )
        )
        self.splat_object_name_list = list(
            map(
                lambda object_cfg: object_cfg["splat_object_name"],
                self.ENV_NAME_TO_CONFIG[env_config_name]["objects"],
            )
        )
        self.grasp_configs = {
            object_cfg["object_name"]: object_cfg["grasp_config"]
            for object_cfg in self.ENV_NAME_TO_CONFIG[env_config_name]["objects"]
        }

        # self.object_name_list = ["plastic_orange", "plate"]
        # self.splat_object_name_list = ["plastic_orange", "plate"]
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

        # set the drop location for the apple and banana
        self.drop_ee_pos = [0.3, -0.5, 0.3]
        self.drop_ee_euler = [-np.pi / 2, 0, -np.pi / 2]
        self.drop_ee_quat = self.pybullet_client.getQuaternionFromEuler(
            self.drop_ee_euler
        )

        # set initial joint positions
        for i in range(1, len(self.initial_joint_state)):
            self.pybullet_client.resetJointState(
                self.dummy_robot, i, self.initial_joint_state[i - 1]
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

        for i in range(1, self.num_dofs()):
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

        # Set up gaussian splat models
        # Placeholder object of type GaussianModel
        self.robot_gaussian = GaussianModel(3)
        # self.T_object_gaussian = GaussianModel(3)

        self.object_gaussians = [
            GaussianModel(3) for _ in range(len(self.urdf_object_list))
        ]
        for i in range(len(self.urdf_object_list)):
            self.object_gaussians[i].load_ply(
                "/home/jennyw2/data/output/{}/point_cloud/iteration_7000/point_cloud.ply".format(
                    self.splat_object_name_list[i]
                )
            )

        # t_gaussians_backup = copy.deepcopy(t_gaussians)

        if self.render_camera_image:
            source_path = self.object_config[self.robot_name]["source_path"]
            if not os.path.exists(source_path):
                raise FileNotFoundError(f"Source path not found: {source_path}")

            model_path = self.object_config[self.robot_name]["model_path"]
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model path not found: {model_path}")

            parser = ArgumentParser(description="Testing script parameters")
            self.pipeline = PipelineParams(parser)
            model = ModelParams(parser, sentinel=True)
            dataset = model.extract(
                Namespace(
                    sh_degree=3,
                    # TODO get these from the object config
                    source_path=source_path,
                    model_path=model_path,
                    images="images",
                    resolution=-1,
                    white_background=False,
                    data_device="cuda",
                    eval=False,
                )
            )
            self.gaussians_backup = GaussianModel(dataset.sh_degree)
            # This loads the .ply file into self.gaussians_backup
            self.scene = Scene(
                dataset,
                self.gaussians_backup,
                load_iteration=-1,
                shuffle=False,
                num_cams=self.cam_i + 2,
            )

            bg_color = [1, 1, 1]
            self.background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")

            self.camera = self.setup_camera_from_dataset(
                cam_i=self.cam_i, use_train=True
            )
            # TODO set up camera2 as the wrist camera
            self.camera2 = self.setup_camera_from_dataset(cam_i=4, use_train=True)
        else:
            self.camera = None
            self.camera2 = None
            self.scene = None
            self.pipeline = None
            self.background = None
            self.gaussians_backup = None

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

    def set_serve_mode(self, serve_mode: SERVE_MODES) -> None:
        if not isinstance(serve_mode, self.SERVE_MODES):
            print(
                f"ERROR: Expected serve_mode to be an enum instance of SERVE_MODES, got {type(serve_mode)}"
            )
        else:
            print(f"Setting serve mode to {serve_mode}")
            self.serve_mode = serve_mode

    def set_object_pose(
        self, object_name: str, position: np.ndarray, orientation: np.ndarray
    ) -> None:
        """Set the pose of an object in the simulation."""
        if object_name not in self.splat_object_name_list:
            print(f"Object {object_name} not found in splat_object_name_list.")
            return

        object_id = self.splat_object_name_list.index(object_name)
        self.pybullet_client.resetBasePositionAndOrientation(
            self.urdf_object_list[object_id], position, orientation
        )

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self.num_dofs(), (
            f"Expected joint state of length {self.num_dofs()}, "
            f"got {len(joint_state)}."
        )

        for i in range(1, self.num_dofs()):
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

    def get_image_observation(self, data, camera="camera1") -> Dict[str, np.ndarray]:
        # Gets transformations for all links of the robot based on the current simulation
        transformations_list = get_transfomration_list(
            self.dummy_robot, self.initial_link_states
        )

        # TODO does this need to be done every time?
        # Ah. it's because xyz gets overwritten
        robot_transformation = self.object_config[self.robot_name]["transformation"][
            "matrix"
        ]
        aabb = self.object_config[self.robot_name]["aabb"]["bounding_box"]
        segmented_list, xyz = get_segmented_indices(
            self.dummy_robot,
            self.gaussians_backup,
            robot_transformation,
            aabb,
            self.robot_name,
        )

        xyz, rot, opacity, shs_featrest, shs_dc = transform_means(
            self.dummy_robot,
            self.gaussians_backup,
            xyz,
            segmented_list,
            transformations_list,
            robot_transformation,
        )

        # Transform each object splat to be in the right pose
        cur_object_position_list = []
        cur_object_rotation_list = []

        for object_name in self.splat_object_name_list:
            cur_object_position = np.array(data[object_name + "_position"])
            cur_object_position_list.append(
                torch.from_numpy(cur_object_position).to(device="cuda").float()
            )
            cur_object_rotation = np.array(data[object_name + "_orientation"])
            cur_object_rotation = np.roll(cur_object_rotation, 1)
            cur_object_rotation_list.append(
                torch.from_numpy(cur_object_rotation).to(device="cuda").float()
            )
        # xyz_cube, rot_cube, opacity_cube, scales_cube, shs_dc_cube, sh_rest_cube = place_object(gaussians_backup, pos=torch.from_numpy(cur_object).to(device='cuda').float(), rotation=torch.from_numpy(curr_rotation).to(device='cuda').float())
        # xyz_obj, rot_obj, opacity_obj, scales_obj, features_dc_obj, features_rest_obj = transform_object(t_gaussians, pos=cur_position, quat=cur_rotation)
        xyz_obj_list = []
        rot_obj_list = []
        opacity_obj_list = []
        scales_obj_list = []
        features_dc_obj_list = []
        features_rest_obj_list = []
        for i in range(len(self.urdf_object_list)):
            (
                xyz_obj,
                rot_obj,
                opacity_obj,
                scales_obj,
                features_dc_obj,
                features_rest_obj,
            ) = transform_object(
                self.object_gaussians[i],
                object_config=self.object_config[self.splat_object_name_list[i]],
                pos=cur_object_position_list[i],
                quat=cur_object_rotation_list[i],
                robot_transformation=robot_transformation,
            )
            xyz_obj_list.append(xyz_obj)
            rot_obj_list.append(rot_obj)
            opacity_obj_list.append(opacity_obj)
            scales_obj_list.append(scales_obj)
            features_dc_obj_list.append(features_dc_obj)
            features_rest_obj_list.append(features_rest_obj)

        # Combine splats of robot and of objects
        with torch.no_grad():
            # gaussians.active_sh_degree = 0
            self.robot_gaussian._xyz = torch.cat([xyz] + xyz_obj_list, dim=0)
            self.robot_gaussian._rotation = torch.cat([rot] + rot_obj_list, dim=0)
            self.robot_gaussian._opacity = torch.cat(
                [opacity] + opacity_obj_list, dim=0
            )
            self.robot_gaussian._features_rest = torch.cat(
                [shs_featrest] + features_rest_obj_list, dim=0
            )
            self.robot_gaussian._features_dc = torch.cat(
                [shs_dc] + features_dc_obj_list, dim=0
            )
            self.robot_gaussian._scaling = torch.cat(
                [self.gaussians_backup._scaling] + scales_obj_list, dim=0
            )

        if camera == "camera1":
            rendering = render(
                self.camera, self.robot_gaussian, self.pipeline, self.background
            )["render"]
        else:
            rendering = render(
                self.camera2, self.robot_gaussian, self.pipeline, self.background
            )["render"]

        # t_gaussians = copy.deepcopy(t_gaussians_backup)

        # convert into numpy
        rendering = rendering.detach().cpu().numpy()

        # convert to hxwxc from cxhxw
        rendering = np.transpose(rendering, (1, 2, 0))

        # convert to 0-255
        rendering = (rendering * 255).astype(np.uint8)

        # show the image
        # resize the image to 640x480
        cv2.imshow(
            camera, cv2.cvtColor(cv2.resize(rendering, (640, 480)), cv2.COLOR_BGR2RGB)
        )
        cv2.waitKey(1)

        # save the image
        return rendering

    def setup_camera2(self):
        uid = 0
        colmap_id = 1
        R = (
            torch.from_numpy(
                np.array(
                    [
                        [-0.98784567, 0.00125165, 0.15543282],
                        [0.1153457, 0.67620402, 0.72762868],
                        [-0.10419356, 0.73671335, -0.66812959],
                    ]
                )
            )
            .float()
            .numpy()
        )

        T = torch.Tensor([1.04674738, -0.96049824, 2.03845016]).float().numpy()

        # T = torch.Tensor([ 0.09347542+0.5, -0.74648806+0.6,  5.57444971+0.2] ).float()

        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        gt_mask_alpha = None

        image_width = 640
        image_height = 480
        image_name = "test"
        image = torch.zeros((3, image_height, image_width)).float()

        # order is  colmap_id, R, T, FoVx, FoVy, image, gt_alpha_mask, image_name, uid,

        camera = Camera(
            colmap_id, R, T, FoVx, FoVy, image, gt_mask_alpha, image_name, uid
        )

        return camera

    def setup_camera_from_dataset(self, cam_i, use_train=True):
        if use_train:
            camera = self.scene.getTrainCameras()[cam_i]
        else:
            camera = self.scene.getTestCameras()[cam_i]
        return camera

    def setup_camera(self):

        uid = 0
        colmap_id = 1
        R = (
            torch.from_numpy(
                np.array(
                    [
                        [1.27679278e-01, -4.36057591e-01, 8.90815233e-01],
                        [6.15525303e-02, 8.99918716e-01, 4.31691546e-01],
                        [-9.89903676e-01, -2.86133428e-04, 1.41741420e-01],
                    ]
                )
            )
            .float()
            .numpy()
        )

        T = torch.Tensor([-1.88933526, -0.6446558, 2.98276143]).float().numpy()

        # T = torch.Tensor([ 0.09347542+0.5, -0.74648806+0.6,  5.57444971+0.2] ).float()

        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        gt_mask_alpha = None

        image_width = 640 * 2
        image_height = 480 * 2
        image_name = "test"
        image = torch.zeros((3, image_height, image_width)).float()

        # order is  colmap_id, R, T, FoVx, FoVy, image, gt_alpha_mask, image_name, uid,

        # self, resolution, colmap_id, R, T, FoVx, FoVy, depth_params, image, invdepthmap,
        #                  image_name, uid,
        #                  trans=np.array([0.0, 0.0, 0.0]), scale=1.0, data_device = "cuda",
        #                  train_test_exp = False, is_test_dataset = False, is_test_view = False
        #                  ):
        camera = Camera(
            colmap_id, R, T, FoVx, FoVy, image, gt_mask_alpha, image_name, uid
        )

        return camera

    def get_observations(self, generate_wrist_rgb=False) -> Dict[str, np.ndarray]:
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
        if self.use_gripper:
            self.current_gripper_state = self.get_current_gripper_state() / 0.8
        else:
            self.current_gripper_state = 0.0

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

        observations["gripper_position"] = [self.current_gripper_state]

        for i in range(len(self.urdf_object_list)):
            (
                object_pos,
                object_quat,
            ) = self.pybullet_client.getBasePositionAndOrientation(
                self.urdf_object_list[i]
            )
            observations[self.splat_object_name_list[i] + "_position"] = object_pos
            observations[self.splat_object_name_list[i] + "_orientation"] = object_quat

        if self.render_camera_image:
            image = self.get_image_observation(
                data=observations,
            )
            if generate_wrist_rgb:
                image2 = self.get_image_observation(data=observations, camera="camera2")
            else:
                image2 = None

            # cv2.imshow('camera1', cv2.cvtColor(cv2.resize(image,(1200, 900) ), cv2.COLOR_BGR2RGB))
            # cv2.waitKey(1)

            observations["base_rgb"] = image
            observations["wrist_rgb"] = image2

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
                    grasp_config = random.choice(self.grasp_configs[object_name])
                    self.grasp_poses[object_id] = grasp_config["grasp_pose"]
                    object_rot = grasp_config["object_rot"]
                    quat = self.pybullet_client.getQuaternionFromEuler(
                        [object_rot[0], object_rot[1], euler_z]
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

    def randomize_ee_pose(self):
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
        for i in range(1, self.num_dofs()):
            self.pybullet_client.resetJointState(
                self.dummy_robot, i, initial_joint_positions[i - 1]
            )
        # TODO possibly randomize gripper state here, too
        # Though that might have to edit initial_joint_positions
        return initial_joint_positions

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
                # Note: Doesn't reset gripper open/close state
                for i in range(1, self.num_dofs()):
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
            # Note: doesn't affect gripper open/close state
            for i in range(1, self.num_dofs()):
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
                # Note: doesn't affect gripper open/close state
                for i in range(1, self.num_dofs()):
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
                # Note: doesn't affect gripper open/close state
                for i in range(1, self.num_dofs()):
                    self.pybullet_client.resetJointState(
                        self.dummy_robot, i, path[-1][i - 1]
                    )

                all_paths.append(path)

                # TODO why is this here. Can it be moved outside?
                if self.skip_recording_first:
                    for i in range(1, self.num_dofs()):
                        self.pybullet_client.resetJointState(
                            self.dummy_robot, i, initial_joint_positions[i - 1]
                        )

        return all_paths

    def follow_paths_and_record(self, all_paths, initial_joint_positions, joint_signs):
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
                self.open_gripper()
                self.pybullet_client.stepSimulation()

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
                self.open_gripper()
                self.pybullet_client.stepSimulation()

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
                    self.close_gripper()
                    for k in range(1, self.num_dofs()):
                        self.pybullet_client.resetJointState(
                            self.dummy_robot,
                            k,
                            initial_joint_positions[k - 1] * joint_signs[k - 1],
                        )

        # TODO put this in all_paths
        # create a path from the drop location to the initial joint positions
        path = [
            np.array(self.drop_ee_joint[:6]) * 0.1 * (10 - i)
            + np.array(self.initial_joint_state[:6]) * 0.1 * (i)
            for i in range(1, 11)
        ]

        self.follow_trajectory(path, 0)

        path = [self.initial_joint_state for _ in range(5)]
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

    def open_gripper(self):
        """Open the gripper."""
        self.move_gripper(0.084)
        self.current_gripper_action = 1

    def close_gripper(self):
        """Close the gripper."""
        self.move_gripper(0.0)
        self.current_gripper_action = 0

    def plan_execute_record_trajectory(self, initial_joint_positions, joint_signs):
        # Returns whether it was a success

        self.trajectory_length = 0

        # make path+trajectory_count folder
        os.makedirs(self.path + str(self.trajectory_count).zfill(3), exist_ok=True)

        all_paths = self.plan_approach_grasp_move_drop_plan(initial_joint_positions)

        for i in range(100):
            self.pybullet_client.stepSimulation()
            # time.sleep(1/240)
            self.close_gripper()
            for k in range(1, self.num_dofs()):
                self.pybullet_client.resetJointState(
                    self.dummy_robot,
                    k,
                    initial_joint_positions[k - 1] * joint_signs[k - 1],
                )

        if len(all_paths) != 5:
            self.delete_trajectory_folder()
            return False

        self.follow_paths_and_record(all_paths, initial_joint_positions, joint_signs)

        # evaluate the success of the trajectory
        correct_trajectory = self.eval_trajectory_success()

        if correct_trajectory:
            return True
        else:
            self.delete_trajectory_folder()
            return False

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()

        # Prepare for teleport by removing forces
        for i in range(len(self.initial_joint_state)):
            self.pybullet_client.setJointMotorControl2(
                self.dummy_robot, i, self.pybullet_client.VELOCITY_CONTROL, force=0
            )
        # Reset joint states by teleporting
        for i in range(1, len(self.initial_joint_state)):
            self.pybullet_client.resetJointState(
                self.dummy_robot,
                i,
                self.initial_joint_state[i - 1] * self.joint_signs[i - 1],
            )
        self.initial_link_states = get_curr_link_states(
            self.dummy_robot, self.use_link_centers
        )

        # get end effector position and orientation
        ee_pos, ee_quat = (
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[0],
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[1],
        )
        self.iniital_ee_quat = ee_quat

        for i in range(1, self.num_dofs()):
            self.pybullet_client.resetJointState(
                self.dummy_robot,
                i,
                self.initial_joint_state[i - 1] * self.joint_signs[i - 1],
            )
            self.close_gripper()

        # get initial ee position and orientation
        self.initial_ee_pos, self.initial_ee_quat = (
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[0],
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[1],
        )
        # print joint angles
        joint_states = []
        for i in range(1, len(self.initial_joint_state)):
            joint_states.append(
                self.pybullet_client.getJointState(self.dummy_robot, i)[0]
            )

        # set to initial joint state
        for i in range(10000):
            for i in range(1, len(self.initial_joint_state)):
                self.pybullet_client.resetJointState(
                    self.dummy_robot,
                    i,
                    self.initial_joint_state[i - 1] * self.joint_signs[i - 1],
                )
            self.pybullet_client.stepSimulation()

        print("Ready to serve.")
        self.ready_to_serve = True

        while True:
            if self.serve_mode == self.SERVE_MODES.INTERACTIVE:
                self.pybullet_client.stepSimulation()
                time.sleep(1 / 240)
            elif self.serve_mode == self.SERVE_MODES.GENERATE_DEMOS:
                # self.get_camera_image_from_end_effector()
                self.randomize_object_pose()
                self.randomize_plate_and_drop_pose()

                # Let the simulation settle
                for i in range(10000):
                    self.pybullet_client.stepSimulation()
                    self.open_gripper()
                    for k in range(1, self.num_dofs()):
                        self.pybullet_client.resetJointState(
                            self.dummy_robot,
                            k,
                            self.initial_joint_state[k - 1] * self.joint_signs[k - 1],
                        )
                self.pybullet_client.stepSimulation()

                initial_joint_positions = self.randomize_ee_pose()

                success = self.plan_execute_record_trajectory(
                    initial_joint_positions, self.joint_signs
                )
                if success:
                    self.trajectory_count += 1

                if self.trajectory_count > self.MAX_TRAJECTORY_COUNT:
                    print(
                        f"Exiting record_demos mode because max trajectory count of {self.MAX_TRAJECTORY_COUNT} was reached in folder {self.path}"
                    )
                    self.set_serve_mode(self.SERVE_MODES.INTERACTIVE)
            else:
                raise ValueError(f"Unknown serve mode {self.serve_mode}. ")

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
            # "finger_joint": 1, # TODO: is this left_outer_knuckle_joint?
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
        if not self.use_gripper:
            return
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

            for j in range(1, self.num_dofs()):
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
            for i in range(1, self.num_dofs()):
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
