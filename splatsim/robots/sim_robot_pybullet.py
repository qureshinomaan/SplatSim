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
        urdf_path: str = '../gaussian-splatting/pybullet-playground/urdf/sisbot_pusher.urdf',
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
        use_gripper: bool = False,
    ):
        
        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)
        self.pybullet_client = p
        self.urdf_path = urdf_path
        self._num_joints = 7
        self.pybullet_client.connect(p.GUI)
        self.pybullet_client.setAdditionalSearchPath("../gaussian-splatting/pybullet-playground/urdf")
        self.robot = self.pybullet_client.loadURDF(self.urdf_path, [0, 0, 0], useFixedBase=True)
        for i in range(self.pybullet_client.getNumJoints(self.robot)):
            info = self.pybullet_client.getJointInfo(self.robot, i)
            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            if joint_name == "ee_fixed_joint":
                self.ur5e_ee_id = joint_id

        self.use_gripper = use_gripper
        # if self.use_gripper:
        #     self.setup_gripper()
        # else:
        #     self.setup_spatula()
        #     pass

        self.pybullet_client.resetBasePositionAndOrientation(self.robot, [0, 0, -0.1], [0, 0, 0, 1])
        self.joint_signs = [1, 1, -1, 1, 1, 1, 1]
        self.offsets = [np.pi/2, 0, 0, 0, 0, 0, 0]

        model_lib = md.model_lib()
        # objectid = self.pybullet_client.loadURDF(model_lib['potato_chip_1'], [0.5, 0.15, 0])

        T_object = self.pybullet_client.loadURDF('../gaussian-splatting/pybullet-playground/urdf/T_object/urdf_T.urdf', [0.5, 0.4, 0])
        # load another object without collision
        T_object2 = self.pybullet_client.loadURDF('../gaussian-splatting/pybullet-playground/urdf/T_object/urdf_T.urdf', [0.7, 0.5, 0.0], useFixedBase=True)
        #disable collision for T_object2
        collisionFilterGroup = 0
        collisionFilterMask = 0
        p.setCollisionFilterGroupMask(T_object2, -1, collisionFilterGroup, collisionFilterMask)



        #make the T_object2 transparent and red
        self.pybullet_client.changeVisualShape(T_object2, -1, rgbaColor=[1, 0, 0, 0.5])

        # #creating a small sphere
        # sphere = self.pybullet_client.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])
        # sphere_id = self.pybullet_client.createMultiBody(baseMass=0, baseInertialFramePosition=[0, 0, 0], baseVisualShapeIndex=sphere)

        #add gravity
        self.pybullet_client.setGravity(0, 0, -9.81)
        
        #add plane
        import pybullet_data
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane = self.pybullet_client.loadURDF("plane.urdf")

        #set time step
        self.pybullet_client.setTimeStep(1/240)

    def num_dofs(self) -> int:
        return 7

    def get_joint_state(self) -> np.ndarray:
        # return self._joint_state
        joint_states = []
        for i in range(1, 8):
            joint_states.append(self.pybullet_client.getJointState(self.robot, i)[0])
        return np.array(joint_states)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )


        for i in range(1, 7):
            # self.pybullet_client.resetJointState(self.robot, i, joint_state[i-1])
            self.pybullet_client.setJointMotorControl2(self.robot, i, p.POSITION_CONTROL, targetPosition=joint_state[i-1], force=200)

        ## get end effector position
        ee_pos = self.pybullet_client.getLinkState(self.robot, 7)[0]
        ## check if z of end effector is less than 0.05
        collison = False
        if ee_pos[2] < 0.09:
            print("z : ", ee_pos[2])
            collison = True
        else:
            self.last_obs = self.get_joint_state()

    
        if collison:
            for i in range(1, 7):
                self.pybullet_client.resetJointState(self.robot, i, self.last_obs[i-1])

        if self.use_gripper:
            if joint_state[-1] > 0.7:
                self.close_gripper()
            else:
                self.open_gripper()

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self.get_joint_state()
        joint_velocities = np.array([self.pybullet_client.getJointState(self.robot, i)[1] for i in range(7)])
        ee_pos, ee_quat = self.pybullet_client.getBasePositionAndOrientation(self.robot)
        gripper_pos = self.pybullet_client.getJointState(self.robot, 7)[0]

        # T_object position and orientation
        object_pos, object_quat = self.pybullet_client.getBasePositionAndOrientation(self.T_object)

        # Target object position and orientation
        object_pos2, object_quat2 = self.pybullet_client.getBasePositionAndOrientation(self.T_object2)


        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "ee_pos_quat": np.concatenate([ee_pos, ee_quat]),
            "gripper_position": gripper_pos,
            "object_position": object_pos,
            "object_orientation": object_quat, 
            "object_position2": object_pos2,
            "object_orientation2": object_quat2
        }

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        initial_joint_state = [0, -1.57, 1.57, -1.57, -1.57, 0]
        joint_signs = [1, 1, 1, 1, 1, 1]
        for i in range(1, 7):
            self.pybullet_client.resetJointState(self.robot, i, initial_joint_state[i-1]*joint_signs[i-1])
        self.last_obs = self.get_joint_state()

        while True:
            self.pybullet_client.stepSimulation()
            time.sleep(1/240)

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None: 
        self.stop()


    def setup_gripper(self):
        """Load end-effector: gripper"""
        ee_position, _ = self.get_link_pose(self.robot, self.ur5e_ee_id)
        self.ee = self.pybullet_client.loadURDF(
            "Assets/ur5e/gripper/robotiq_2f_85.urdf",
            ee_position,
            self.pybullet_client.getQuaternionFromEuler((-np.pi / 2, 0, 0)),
        )
        self.ee_tip_offset = 0.15
        self.gripper_angle_open = 0.03
        self.gripper_angle_close = 0.8
        self.gripper_angle_close_threshold = 0.7
        self.gripper_mimic_joints = {
            "left_inner_finger_joint": -1,
            "left_inner_knuckle_joint": -1,
            "right_outer_knuckle_joint": -1,
            "right_inner_finger_joint": -1,
            "right_inner_knuckle_joint": -1,
        }
        for i in range(self.pybullet_client.getNumJoints(self.ee)):
            info = self.pybullet_client.getJointInfo(self.ee, i)
            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            if joint_name == "finger_joint":
                self.gripper_main_joint = joint_id
            elif joint_name == "dummy_center_fixed_joint":
                self.ee_tip_id = joint_id
            elif (
                joint_name == "left_inner_finger_pad_joint"
                or joint_name == "right_inner_finger_pad_joint"
            ):
                self.pybullet_client.changeDynamics(self.ee, joint_id, lateralFriction=1)
            elif joint_type == p.JOINT_REVOLUTE:
                self.gripper_mimic_joints[joint_name] = joint_id
                # Keep the joints static
                self.pybullet_client.setJointMotorControl2(
                    self.ee, joint_id, self.pybullet_client.VELOCITY_CONTROL, targetVelocity=0, force=0
                )
        self.ee_constraint = self.pybullet_client.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=self.ur5e_ee_id,
            childBodyUniqueId=self.ee,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=(0, 0, 1),
            parentFramePosition=(0, 0, 0),
            childFramePosition=(0, 0, -0.02),
            childFrameOrientation=p.getQuaternionFromEuler((0, -np.pi / 2, 0)),
        )
        self.pybullet_client.enableJointForceTorqueSensor(self.ee, self.gripper_main_joint, 1)

        # Set up mimic joints in robotiq gripper: left
        c = self.pybullet_client.createConstraint(
            self.ee,
            self.gripper_main_joint,
            self.ee,
            self.gripper_mimic_joints["left_inner_finger_joint"],
            jointType=p.JOINT_GEAR,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        self.pybullet_client.changeConstraint(c, gearRatio=1, erp=0.5, maxForce=800)
        c = self.pybullet_client.createConstraint(
            self.ee,
            self.gripper_main_joint,
            self.ee,
            self.gripper_mimic_joints["left_inner_knuckle_joint"],
            jointType=p.JOINT_GEAR,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        self.pybullet_client.changeConstraint(c, gearRatio=-1, erp=0.5, maxForce=800)
        # Set up mimic joints in robotiq gripper: right
        c = self.pybullet_client.createConstraint(
            self.ee,
            self.gripper_mimic_joints["right_outer_knuckle_joint"],
            self.ee,
            self.gripper_mimic_joints["right_inner_finger_joint"],
            jointType=p.JOINT_GEAR,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        self.pybullet_client.changeConstraint(c, gearRatio=1, erp=0.5, maxForce=800)
        c = self.pybullet_client.createConstraint(
            self.ee,
            self.gripper_mimic_joints["right_outer_knuckle_joint"],
            self.ee,
            self.gripper_mimic_joints["right_inner_knuckle_joint"],
            jointType=p.JOINT_GEAR,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        self.pybullet_client.changeConstraint(c, gearRatio=-1, erp=0.5, maxForce=800)
        # Set up mimic joints in robotiq gripper: connect left and right
        c = self.pybullet_client.createConstraint(
            self.ee,
            self.gripper_main_joint,
            self.ee,
            self.gripper_mimic_joints["right_outer_knuckle_joint"],
            jointType=p.JOINT_GEAR,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        self.pybullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=100)

    def get_link_pose(self, body, link):
        result = self.pybullet_client.getLinkState(body, link)
        return result[4], result[5]
    

    def _move_gripper(self, target_angle, speed=0.01, timeout=3, max_force=5, is_slow=False):
        t0 = time.time()
        count = 0
        max_count = 3
        current_angle = self.pybullet_client.getJointState(self.ee, self.gripper_main_joint)[0]

        if is_slow:
            self.pybullet_client.setJointMotorControl2(
                self.ee,
                self.gripper_main_joint,
                p.POSITION_CONTROL,
                targetPosition=target_angle,
                maxVelocity=0.5,
                force=1,
            )
            self.pybullet_client.setJointMotorControl2(
                self.ee,
                self.gripper_mimic_joints["right_outer_knuckle_joint"],
                p.POSITION_CONTROL,
                targetPosition=target_angle,
                maxVelocity=0.5,
                force=1,
            )
            for _ in range(500):
                self.pybullet_client.stepSimulation()
        self.pybullet_client.setJointMotorControl2(
            self.ee,
            self.gripper_main_joint,
            p.POSITION_CONTROL,
            targetPosition=target_angle,
            maxVelocity=10,
            force=5,
        )
        self.pybullet_client.setJointMotorControl2(
            self.ee,
            self.gripper_mimic_joints["right_outer_knuckle_joint"],
            p.POSITION_CONTROL,
            targetPosition=target_angle,
            maxVelocity=10,
            force=5,
        )



    def setup_spatula(self):
        """Load end-effector: spatula"""
        ee_position, _ = self.get_link_pose(self.robot, self.ur5e_ee_id)
        ee_position = [ee_position[0], ee_position[1], ee_position[2] ]
        self.ee = self.pybullet_client.loadURDF(
            # "Assets/ur5e/spatula/spatula-base.urdf",
            "../gaussian-splatting/pybullet-playground/urdf/pusher.urdf",
            ee_position,
            p.getQuaternionFromEuler((-np.pi / 2, 0, 0)),
        )
        self.ee_tip_offset = (
            0.12  # tip_distance: the add-on distance to the tip from ur5e ee joint.
        )
        self.ee_tip_id = 0  # id of tip_link
        self.ee_constraint = self.pybullet_client.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=self.ur5e_ee_id,
            childBodyUniqueId=self.ee,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=(0, 0, 0),
            parentFramePosition=(0, 0, 0),
            childFramePosition=(0, -0.0, -0.058),
            childFrameOrientation=p.getQuaternionFromEuler((0, -np.pi / 2, np.pi/2)),
        )

    def open_gripper(self):
        self._move_gripper(self.gripper_angle_open, speed=0.01)

    def close_gripper(self):
        self._move_gripper(self.gripper_angle_close, speed=0.01, is_slow=True)