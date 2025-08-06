import time

import torch
import numpy as np
import mujoco
import mujoco.viewer
import zmq

assert mujoco.viewer is mujoco.viewer
import pybullet as p
from pybullet_planning import plan_joint_motion, get_movable_joints, set_joint_positions

from splatsim.robots.sim_robot_pybullet_base import PybulletRobotServerBase
from splatsim.utils.transform_utils import rotation_matrix_to_euler_angles


class AppleSearchPybulletRobotServer(PybulletRobotServerBase):
    ENV_CONFIG_NAME = "apple_search"

    ENV_CONFIG = {
        "objects": [
            {
                "object_name": "plastic_apple",
                "splat_object_name": "plastic_apple",
                "grasp_config": [PybulletRobotServerBase.GRASP_CONFIGS["apple"]],
            },
            # TODO add obstacle objects that may or may not be in view
            # {
            #     "object_name": "plate",
            #     "splat_object_name": "plate",
            #     "grasp_config": [],
            # },
        ]
    }

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.goal_ee_pos = None
        self.goal_ee_quat = None

    def plan_given_this_state(self, initial_joint_positions):
        all_paths = []

        ik_joints = get_movable_joints(self.dummy_robot)
        goal_joint_positions = self.pybullet_client.calculateInverseKinematics(
            self.dummy_robot,
            6,
            self.goal_ee_pos,
            self.goal_ee_quat,
            maxNumIterations=100000,
            residualThreshold=1e-10,
        )

        path = plan_joint_motion(
            self.dummy_robot,
            ik_joints,
            goal_joint_positions,
            # TODO add more obstacles here
            obstacles=[self.plane, self.urdf_object_list[-1]],
            self_collisions=False,
        )
        all_paths.append(path)

        # TODO why is this here. Can it be moved outside?
        if self.skip_recording_first:
            for i in range(1, self.num_dofs()):
                self.pybullet_client.resetJointState(
                    self.dummy_robot, i, initial_joint_positions[i - 1]
                )

        return all_paths

    def serve_loop(self) -> None:
        # To be called in the parent's serve()
        if self.serve_mode == self.SERVE_MODES.INTERACTIVE:
            self.pybullet_client.stepSimulation()
            time.sleep(1 / 240)
        elif self.serve_mode == self.SERVE_MODES.GENERATE_DEMOS:
            self.randomize_object_pose()
            initial_joint_positions = self.randomize_ee_pose()

            # Set the goal quat to be pointing towards the apple
            # TODO collision checking between robot goal pose and apple pose
            apple_pos, apple_quat = self.get_current_object_pose("plastic_apple")
            self.goal_ee_pos, self.goal_ee_quat = self.get_random_ee_pose()
            self.goal_ee_quat = self.compute_look_at_quaternion(
                self.goal_ee_pos, apple_pos, forward_axis="x"
            )

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

    def compute_look_at_quaternion(
        self, eye, target, up=np.array([0, 0, 1]), forward_axis="z"
    ):
        forward = np.array(target) - np.array(eye)
        forward /= np.linalg.norm(forward)

        if forward_axis == "z":
            right = np.cross(up, forward)
            right /= np.linalg.norm(right)
            true_up = np.cross(forward, right)
            rot_matrix = np.column_stack((right, true_up, forward))
        elif forward_axis == "x":
            up = np.array([0, 0, 1])
            right = np.cross(forward, up)
            right /= np.linalg.norm(right)
            true_up = np.cross(right, forward)
            rot_matrix = np.column_stack((forward, right, true_up))
        else:
            raise ValueError("Unsupported forward axis")

        return p.getQuaternionFromMatrix(rot_matrix.flatten().tolist())
