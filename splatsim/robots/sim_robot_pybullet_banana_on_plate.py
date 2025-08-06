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


class BananaOnPlatePybulletRobotServer(PybulletRobotServerBase):
    ENV_CONFIG_NAME = "banana_on_plate"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def plan_given_this_state(self, initial_joint_positions):
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

    def serve_loop(self) -> None:
        # To be called in the parent's serve()
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
