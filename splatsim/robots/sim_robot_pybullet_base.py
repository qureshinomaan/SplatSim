import pickle
import threading
import time
from typing import Any, Dict, Optional, List
import enum
import random
from collections import namedtuple
import math
import os
import pickle
import shutil
import yaml
from argparse import ArgumentParser

from dataclasses import dataclass, field
import numpy as np
import threading
import queue


import torch
import numpy as np
import mujoco
import mujoco.viewer
import zmq
from splatsim.robots.robot import Robot

import cv2
from torchvision.transforms.functional import to_pil_image

assert mujoco.viewer is mujoco.viewer
from gaussian_splatting.scene.cameras import Camera
from gaussian_renderer import render
import urdf_models.models_data as md
import pybullet as p
from pybullet_planning.interfaces.robots.collision import pairwise_collision
import pybullet_data
from splatsim.utils.robot_splat_render_utils import (
    get_segmented_indices,
    transform_means,
    get_transfomration_list,
    transform_object,
    get_curr_link_states,
)
from gaussian_splatting.gaussian_renderer import GaussianModel
from gaussian_splatting.arguments import ModelParams, PipelineParams, Namespace
from gaussian_splatting.scene import Scene

from splatsim.utils.transform_utils import rotation_matrix_to_euler_angles


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
                # print(f"Received request: {method}, {args}")
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


class GripperState(str, enum.Enum):
    OPEN = 1
    CLOSE = 0


@dataclass
class PathSegment:
    path_type: str = field(init=False)


@dataclass
class TrajectoryPathSegment(PathSegment):
    path: np.ndarray
    gripper_pos: float
    gripper_velocity: float = 0.2
    threshold: float = 1e-2

    def __post_init__(self):
        self.path_type = "trajectory"


@dataclass
class GripperPathSegment(PathSegment):
    target_state: GripperState
    num_steps: int = 160

    def __post_init__(self):
        self.path_type = "gripper"


class PybulletRobotServerBase:
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

    # TODO is there a plastic strawberry env?

    ENV_CONFIG = None  # To be set in a subclass

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
        use_gripper: bool = True,
        serve_mode: str = SERVE_MODES.GENERATE_DEMOS,
        use_link_centers: bool = True,
        robot_name: str = "robot_iphone",
        camera_names: List[str] = ["base_rgb"],
        cam_i: int = 254,
        object_config_path: str = "./configs/object_configs/objects.yaml",
    ):
        self.serve_mode = serve_mode
        self.use_link_centers = use_link_centers
        self.robot_name = robot_name
        self.camera_names = camera_names
        self.cam_i = cam_i

        # load labels.npy
        self.robot_labels = np.load(
            "./data/labels_path/" + self.robot_name + "_labels.npy"
        )
        self.robot_labels = torch.from_numpy(self.robot_labels).to(device="cuda").long()
        self.transformations_cache = None

        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)
        self.pybullet_client = p
        self.object_config_path = object_config_path
        self.grasp_poses = {}
        self.pybullet_client.connect(p.GUI)
        self.pybullet_client.setAdditionalSearchPath(
            "./submodules/pybullet-playground-wrapper/pybullet_playground/urdf/pybullet_ur5_gripper/urdf"
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
                self.ENV_CONFIG["objects"],
            )
        )
        self.splat_object_name_list = list(
            map(
                lambda object_cfg: object_cfg["splat_object_name"],
                self.ENV_CONFIG["objects"],
            )
        )
        self.grasp_configs = {
            object_cfg["object_name"]: object_cfg["grasp_config"]
            for object_cfg in self.ENV_CONFIG["objects"]
        }

        for object_name in [self.robot_name] + self.splat_object_name_list:
            transformation = np.array(
                self.object_config[self.robot_name]["transformation"]["matrix"]
            )
            self.populate_transformations_cache(object_name, transformation)

        self.randomize_object_positions = [True, False]
        self.randomize_object_rotations = [False, True]
        self.rotation_values = [[0, 0], [-np.pi / 6, np.pi / 6]]
        self.use_fixed_base = [False, True]
        global_scaling_list = [1, 1]
        self.urdf_object_list = []
        self.urdf_object_mass_list = []
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
                self.urdf_object_list.append(object_loaded)
            mass = self.pybullet_client.getDynamicsInfo(object_loaded, -1)[0]
            self.urdf_object_mass_list.append(mass)

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
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = self.pybullet_client.loadURDF("plane.urdf", [0, 0, -0.022])

        # place a wall in -0.4 at x axis using plane.urdf
        # wall is perpendicular to the plane
        quat = self.pybullet_client.getQuaternionFromEuler([0, np.pi / 2, 0])
        self.wall = self.pybullet_client.loadURDF("plane.urdf", [-0.4, 0, 0.0], quat)

        ## add stage
        self.stage = 0

        # change the friction of the plane
        # self.pybullet_client.changeDynamics(self.plane, -1, lateralFriction=random.uniform(0.2, 1.1))

        # set time step
        self.pybullet_client.setTimeStep(1 / 240)

        # current gripper state
        self.current_gripper_action = 0

        # trajectory path
        self.path = "/home/jennyw2/data/bc_data/gello/"
        # get no of folders in the path
        self.trajectory_count = len(os.listdir(self.path))

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
        for i, splat_name in enumerate(self.splat_object_name_list):
            ply_path = self.object_config[splat_name]["ply_path"]
            self.object_gaussians[i].load_ply(ply_path)

        # t_gaussians_backup = copy.deepcopy(t_gaussians)

        if "base_rgb" in self.camera_names:
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
                    depths="",
                    resolution=-1,
                    white_background=False,
                    train_test_exp=False,
                    data_device="cuda",
                    eval=False,
                )
            )
            self.gaussians_backup = GaussianModel(dataset.sh_degree)
            # This loads the .ply file into self.gaussians_backup
            self.cam_scale = (
                2  # A scale of 2 produces a smaller image than a scale of 1
            )
            self.scene = Scene(
                dataset,
                self.gaussians_backup,
                load_iteration=-1,
                shuffle=False,
                resolution_scales=[self.cam_scale],
                train_cam_indices=[self.cam_i],
                test_cam_indices=[
                    0
                ],  # Even tho we're not using this, make sure to load at max 1 camera for memory purposes
            )

            bg_color = [1, 1, 1]
            self.background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")

            self.base_camera = self.setup_camera_from_dataset(
                cam_i=self.cam_i, use_train=True
            )
        else:
            self.base_camera = None
            self.scene = None
            self.pipeline = None
            self.background = None
            self.gaussians_backup = None

        self.wrist_camera_link_index = None
        if "wrist_rgb" in self.camera_names:
            # Get the index of the wrist_camera_link
            if "wrist_camera_link_name" in self.object_config[self.robot_name]:
                wrist_camera_link_name = self.object_config[self.robot_name][
                    "wrist_camera_link_name"
                ]
                num_joints = p.getNumJoints(self.dummy_robot)
                for i in range(num_joints):
                    info = p.getJointInfo(self.dummy_robot, i)
                    if info[12].decode("utf-8") == wrist_camera_link_name:
                        self.wrist_camera_link_index = i
                        break
                if self.wrist_camera_link_index is None:
                    raise ValueError(
                        f"Cannot find wrist camera link name {wrist_camera_link_name}"
                    )
            else:
                raise ValueError(
                    f"wrist_camera_link_name attribute not defined in object config of robot {self.robot_name}, yet wrist camera was requested"
                )

        self.frame_queue = queue.Queue(maxsize=2)  # small size to avoid huge lag

        def display_frame_worker():
            while True:
                camera_name, frame = self.frame_queue.get()
                if frame is None:
                    break
                # format + show
                frame = np.transpose(frame, (1, 2, 0))  # CxHxW -> HxWxC
                frame = (frame * 255).astype(np.uint8)
                cv2.imshow(camera_name, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                cv2.waitKey(1)

        self.display_frame_thread = threading.Thread(
            target=display_frame_worker, daemon=True
        )
        self.display_frame_thread.start()

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
        self,
        object_name: str,
        position: np.ndarray,
        orientation: np.ndarray,
        use_gravity: bool = True,
    ) -> None:
        """Set the pose of an object in the simulation."""
        if object_name not in self.splat_object_name_list:
            print(f"Object {object_name} not found in splat_object_name_list.")
            return

        object_id = self.splat_object_name_list.index(object_name)
        self.pybullet_client.resetBasePositionAndOrientation(
            self.urdf_object_list[object_id], position, orientation
        )

        if not use_gravity:
            # Make the object static so that it doesn't move
            self.pybullet_client.changeDynamics(
                self.urdf_object_list[object_id], -1, mass=0
            )
        else:
            self.pybullet_client.changeDynamics(
                self.urdf_object_list[object_id],
                -1,
                mass=self.urdf_object_mass_list[object_id],
            )

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self.num_dofs(), (
            f"Expected joint state of length {self.num_dofs()}, "
            f"got {len(joint_state)}."
        )

        for i in range(1, self.num_dofs()):
            self.pybullet_client.setJointMotorControl2(
                self.dummy_robot,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_state[i - 1],
                force=1000,
                # force=250,
            )

        if self.use_gripper:
            self.move_gripper((1 - joint_state[-1]) * 0.085)
            self.current_gripper_action = joint_state[-1]

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def populate_transformations_cache(self, object_name, object_transformation):
        if self.transformations_cache is None:
            self.transformations_cache = {}
        object_transformation = (
            torch.tensor(object_transformation).to(device="cuda").float()
        )
        object_transformation_scale = torch.pow(
            torch.linalg.det(object_transformation[:3, :3]), 1 / 3
        )
        object_transformation_inv = torch.inverse(object_transformation)
        object_transformation_inv_scale = torch.pow(
            torch.linalg.det(object_transformation_inv[:3, :3]), 1 / 3
        )
        self.transformations_cache[object_name] = {
            "transformation": object_transformation,
            "transformation_scale": object_transformation_scale,
            "inv_transformation": object_transformation_inv,
            "inv_transformation_scale": object_transformation_inv_scale,
        }

    def get_wrist_camera(self):
        if self.wrist_camera_link_index is None:
            print("WARNING: No wrist camera index found")
            return None

        uid = 0
        colmap_id = 1

        # Get the pose of the wrist_camera_link
        link_state = p.getLinkState(
            self.dummy_robot,
            self.wrist_camera_link_index,
            computeForwardKinematics=True,
        )

        # robot_transformation = np.array(
        #     self.object_config[self.robot_name]["transformation"]["matrix"]
        # )
        if self.transformations_cache is None:
            robot_transformation = torch.tensor(
                self.object_config[self.robot_name]["transformation"]["matrix"],
                device="cuda",
            )
            robot_transformation_inv = torch.linalg.inv(robot_transformation)
        else:
            robot_transformation = self.transformations_cache[self.robot_name][
                "transformation"
            ]
            robot_transformation_inv = self.transformations_cache[self.robot_name][
                "inv_transformation"
            ]

        T = torch.tensor(
            link_state[0], device=robot_transformation.device
        ).float()  # xyz position in world frame
        quat = link_state[1]
        R = (
            torch.tensor(
                p.getMatrixFromQuaternion(quat), device=robot_transformation.device
            )
            .reshape(3, 3)
            .float()
        )
        Trans_cam_world = torch.eye(4, device=R.device)
        Trans_cam_world[:3, :3] = R
        Trans_cam_world[:3, 3] = T

        robot_transformation[:3, 3] = robot_transformation[:3, 3]
        Trans_cam_splat = torch.matmul(robot_transformation_inv, Trans_cam_world)

        FoVx = 1.375955594372348
        FoVy = 1.1025297299614814

        image_width = 640
        image_height = 480
        image_name = "wrist_rgb"
        image = torch.zeros((3, image_height, image_width)).float()

        # Original camera-to-world transform
        R_cw = Trans_cam_splat[:3, :3]
        T_cw = Trans_cam_splat[:3, 3]
        scale = torch.pow(torch.linalg.det(R_cw[:3, :3]), 1 / 3)
        R_cw = R_cw / scale
        T_cw = T_cw / scale
        Trans_cam_splat_wo_scale = torch.eye(4, device=R_cw.device)
        Trans_cam_splat_wo_scale[:3, :3] = R_cw
        Trans_cam_splat_wo_scale[:3, 3] = T_cw

        # Convert to world-to-camera
        Rt_wc = torch.linalg.inv(Trans_cam_splat_wo_scale)
        T_wc = Rt_wc[:3, 3]

        resolution = (image_width, image_height)
        depth_params = None
        invdepthmap = None

        # I really don't understand why this combination of rotation and translation matrices fixes calibration...
        camera = Camera(
            resolution,
            colmap_id,
            R_cw.detach().cpu().numpy(),
            T_wc.detach().cpu().numpy(),
            FoVx,
            FoVy,
            depth_params,
            to_pil_image(image),
            invdepthmap,
            # gt_mask_alpha,
            image_name,
            uid,
            scale=scale.detach().cpu().numpy(),
        )

        return camera

    def prep_image_rendering(self, data) -> Dict[str, np.ndarray]:
        start = time.time()
        # Gets transformations for all links of the robot based on the current simulation
        transformations_list = get_transfomration_list(
            self.dummy_robot, self.initial_link_states
        )
        end = time.time()
        print(f"get transformation list: {end - start}")

        # TODO does this need to be done every time?
        # Ah. it's because xyz gets overwritten
        start = time.time()
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
            robot_labels=self.robot_labels,
            transformations_cache=self.transformations_cache,
        )
        end = time.time()
        print(f"get segmented indices: {end - start}")

        start = time.time()
        xyz, rot, opacity, shs_featrest, shs_dc = transform_means(
            self.dummy_robot,
            self.gaussians_backup,
            xyz,
            segmented_list,
            transformations_list,
            robot_transformation,
            robot_name=self.robot_name,
            transformations_cache=self.transformations_cache,
        )
        end = time.time()
        print(f"transform means: {end - start}")

        # Transform each object splat to be in the right pose
        cur_object_position_list = []
        cur_object_rotation_list = []

        for object_name in self.splat_object_name_list:
            cur_object_position_list.append(
                torch.tensor(data[object_name + "_position"], device="cuda").float()
            )
            cur_object_rotation_list.append(
                torch.tensor(data[object_name + "_orientation"], device="cuda").float()
            )
        start = time.time()
        xyz_obj_list = []
        rot_obj_list = []
        opacity_obj_list = []
        scales_obj_list = []
        features_dc_obj_list = []
        features_rest_obj_list = []
        for i in range(len(self.urdf_object_list)):
            object_name = self.splat_object_name_list[i]
            (
                xyz_obj,
                rot_obj,
                opacity_obj,
                scales_obj,
                features_dc_obj,
                features_rest_obj,
            ) = transform_object(
                self.object_gaussians[i],
                object_config=self.object_config[object_name],
                pos=cur_object_position_list[i],
                quat=cur_object_rotation_list[i],
                robot_transformation=robot_transformation,
                object_name=object_name,
                robot_name=self.robot_name,
                transformations_cache=self.transformations_cache,
            )
            xyz_obj_list.append(xyz_obj)
            rot_obj_list.append(rot_obj)
            opacity_obj_list.append(opacity_obj)
            scales_obj_list.append(scales_obj)
            features_dc_obj_list.append(features_dc_obj)
            features_rest_obj_list.append(features_rest_obj)
        end = time.time()
        print(f"transform objects: {end - start}")

        # Combine splats of robot and of objects
        start = time.time()
        with torch.no_grad():
            # gaussians.active_sh_degree = 0
            self.robot_gaussian._xyz = torch.cat(
                [xyz] + xyz_obj_list,
                dim=0,
            )
            self.robot_gaussian._rotation = torch.cat(
                [rot] + rot_obj_list,
                dim=0,
            )
            self.robot_gaussian._opacity = torch.cat(
                [opacity] + opacity_obj_list,
                dim=0,
            )
            self.robot_gaussian._features_rest = torch.cat(
                [shs_featrest] + features_rest_obj_list,
                dim=0,
            )
            self.robot_gaussian._features_dc = torch.cat(
                [shs_dc] + features_dc_obj_list,
                dim=0,
            )
            self.robot_gaussian._scaling = torch.cat(
                [self.gaussians_backup._scaling] + scales_obj_list,
                dim=0,
            )
        end = time.time()
        print(f"concatenate scene and object points: {end - start}")

    def render_image(self, camera_name):
        # TODO to save compute, you only need to create the splat once, then it can be rendered w/ different cameras
        if camera_name == "base_rgb":
            camera = self.base_camera
        elif camera_name == "wrist_rgb":
            camera = self.get_wrist_camera()
            if camera is None:
                return None
        else:
            raise ValueError(f"Unknown camera name {camera_name}")

        start = time.time()
        rendering = render(camera, self.robot_gaussian, self.pipeline, self.background)[
            "render"
        ]
        end = time.time()
        print(f"render splat: {end - start}")

        start = time.time()
        rendering_cpu = torch.empty_like(rendering, device="cpu", pin_memory=True)
        rendering_cpu.copy_(rendering.detach(), non_blocking=True)
        self.frame_queue.put((camera_name, rendering_cpu.numpy()))
        try:
            self.frame_queue.put_nowait((camera_name, rendering_cpu.numpy()))

        except queue.Full:
            pass  # skip frame if queue is full
        print(f"show image: {end - start}")

        # save the image
        return rendering

    def setup_camera_from_dataset(self, cam_i, use_train=True):
        # Assume that self.cam_train_indices and self.cam_test_indices have already singled out
        # the camera of interest. Return the first camera in the list
        if use_train:
            camera = self.scene.getTrainCameras(scale=self.cam_scale)[0]
        else:
            camera = self.scene.getTestCameras(scale=self.cam_scale)[0]
        return camera

    def get_current_ee_pose(self):
        dummy_ee_pos, dummy_ee_quat = (
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[0],
            self.pybullet_client.getLinkState(self.dummy_robot, 6)[1],
        )
        return dummy_ee_pos, dummy_ee_quat

    def get_current_object_pose(self, object_name=None, object_id=None):
        if object_name is not None:
            if object_name not in self.splat_object_name_list:
                raise ValueError(
                    f"Object name '{object_name}' not found when querying its pose."
                )
            queried_object_id = self.splat_object_name_list.index(object_name)
            if object_id is not None:
                assert object_id == queried_object_id
            object_id = queried_object_id
        elif object_id is None:
            raise ValueError("No object_name or object_id given!")

        body_id = self.urdf_object_list[object_id]
        pos, quat = self.pybullet_client.getBasePositionAndOrientation(body_id)
        return pos, quat

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self.get_joint_state()
        joint_positions_dummy = self.get_joint_state_dummy()
        joint_velocities = np.array(
            [
                self.pybullet_client.getJointState(self.dummy_robot, i)[1]
                for i in range(7)
            ]
        )

        dummy_ee_pos, dummy_ee_quat = self.get_current_ee_pose()
        # get the euler angles from the quaternion
        dummy_ee_euler = self.pybullet_client.getEulerFromQuaternion(dummy_ee_quat)

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
            "ee_pos_quat": dummy_ee_quat,
            "state": state,
            "action": action,
        }

        # gripper_position is for gello integration. It's a shame that it intersects with self.splat_object_name_list convetion
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

        start = time.time()

        self.prep_image_rendering(data=observations)
        for camera_name in self.camera_names:
            observations[camera_name] = self.render_image(camera_name=camera_name)
        for camera_name in ["base_rgb", "wrist_rgb"]:
            if camera_name not in observations:
                observations[camera_name] = None

        end = time.time()
        duration = end - start
        print(f"get_observations took {duration} seconds ({1/duration} fps)")
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

    def follow_paths_and_record(self, all_paths: List[PathSegment]):
        for path_segment in all_paths:
            if isinstance(path_segment, TrajectoryPathSegment):
                self.follow_trajectory_and_record(
                    path=path_segment.path,
                    gripper_pos=path_segment.gripper_pos,
                    gripper_velocity=path_segment.gripper_velocity,
                    threshold=path_segment.threshold,
                    use_current_iters=True,  # Seems to always be default true
                )
            elif isinstance(path_segment, GripperPathSegment):
                if path_segment.target_state == GripperState.OPEN:
                    self.open_gripper_and_record(num_steps=path_segment.num_steps)
                elif path_segment.target_state == GripperState.CLOSE:
                    self.close_gripper_and_record(num_steps=path_segment.num_steps)
                else:
                    raise ValueError(
                        f"Unknown GripperPathSegment.target_state value {path_segment.target_state}"
                    )
            else:
                raise ValueError(f"Unknown path segment type {type(path_segment)}")

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
        self.current_gripper_action = GripperState.OPEN  # 1

    def close_gripper(self):
        """Close the gripper."""
        self.move_gripper(0.0)
        self.current_gripper_action = GripperState.CLOSE  # 0

    def plan_execute_record_trajectory(self, initial_joint_positions, joint_signs):
        # Returns whether it was a success

        self.trajectory_length = 0

        # make path+trajectory_count folder
        trajectory_folder = self.path + str(self.trajectory_count).zfill(3)
        print("Generating trajectory in folder:", trajectory_folder)
        os.makedirs(trajectory_folder, exist_ok=True)

        all_paths = self.plan_given_this_state(initial_joint_positions)

        for i in range(100):
            self.pybullet_client.stepSimulation()
            self.open_gripper()
            for k in range(1, self.num_dofs()):
                self.pybullet_client.resetJointState(
                    self.dummy_robot,
                    k,
                    initial_joint_positions[k - 1] * joint_signs[k - 1],
                )

        if len(all_paths) == 0:
            self.delete_trajectory_folder()
            return False

        self.follow_paths_and_record(all_paths)

        # evaluate the success of the trajectory
        correct_trajectory = self.eval_trajectory_success()

        if correct_trajectory:
            return True
        else:
            self.delete_trajectory_folder()
            return False

    def serve(self) -> None:
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
        ee_pos, ee_quat = self.get_current_ee_pose()
        self.iniital_ee_quat = ee_quat

        for i in range(1, self.num_dofs()):
            self.pybullet_client.setJointMotorControl2(
                self.dummy_robot,
                i,
                p.VELOCITY_CONTROL,
                targetPosition=self.initial_joint_state[i - 1]
                * self.joint_signs[i - 1],
                force=250,
                maxVelocity=0.2,
            )
        self.close_gripper()

        # get initial ee position and orientation
        self.initial_ee_pos, self.initial_ee_quat = self.get_current_ee_pose()
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

        # start the zmq server
        self._zmq_server_thread.start()

        print("Ready to serve.")

        while True:
            self.serve_loop()

    def serve_loop():
        raise NotImplementedError()

    def plan_given_this_state(self, initial_joint_positions):
        raise NotImplementedError()

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

    def close_gripper_and_record(self, num_steps=160):
        for i in range(num_steps):
            self.close_gripper()
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

    def open_gripper_and_record(self, num_steps=160):
        for i in range(num_steps):
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

    def follow_trajectory_and_record(
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

    def shutdown(self):
        # Say to shut down
        self.frame_queue.put((None, None))
        self.display_frame_thread.join(timeout=2.0)  # wait up to 2 seconds
        if self.display_frame_thread.is_alive():
            print("⚠ Display thread did not exit in time.")
