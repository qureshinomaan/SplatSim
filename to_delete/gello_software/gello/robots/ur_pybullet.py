from typing import Dict

import numpy as np

from splatsim.robots.robot import Robot

import pybullet as p


class URRobotPybullet(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = True):
        import rtde_control
        import rtde_receive

        try:
            print(robot_ip)
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
            p.connect(p.GUI)
            self.dummy_robot = p.loadURDF("../gaussian-splatting/pybullet-playground/urdf/sisbot.urdf", [0, 0, -0.1], useFixedBase=True)
            p.setGravity(0, 0, -9.81)
            # p.setRealTimeSimulation(1)
            p.setTimeStep(1/240)
            #set initial joint positions
            initial_joint_state = [0, -1.57, 1.57, -1.57, -1.57, 0]
            self.initial_joint_state = initial_joint_state
            joint_signs = [1, 1, 1, 1, 1, 1]
            for i in range(1, 7):
                p.resetJointState(self.dummy_robot, i, initial_joint_state[i-1]*joint_signs[i-1])
            p.stepSimulation()

            ee_pos, ee_quat = p.getLinkState(self.dummy_robot, 6)[0], p.getLinkState(self.dummy_robot, 6)[1]
            self.correct_ee_quat = ee_quat

            print('in try')
        except Exception as e:
            print(e)
            print(robot_ip)
            print('I am in except')

        self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not no_gripper:
            from splatsim.robots.robotiq_gripper import RobotiqGripper
            from splatsim.robots.custom_gripper import CustomGripper

            # self.gripper = RobotiqGripper()
            self.gripper = CustomGripper()
            # self.gripper.connect(hostname=robot_ip, port=63352)
            self.gripper.connect()
            # print("gripper connected")
            # gripper.activate()

        [print("connect") for _ in range(4)]

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        # import time

        # time.sleep(0.01)
        
        gripper_pos = self.gripper.get_current_position()
        gripper_pos = gripper_pos - 2604
        gripper_pos = gripper_pos // 1290
        #flip the gripper position
        # gripper_pos = (1 - gripper_pos)
        # print("gripper position", gripper_pos)
        # assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos 

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.r_inter.getActualQ()
        robot_joints_dummy = []
        for i in range(1, 7):
            joint = p.getJointState(self.dummy_robot, i)
            robot_joints_dummy.append(joint[0])

        robot_joints_dummy = np.array(robot_joints_dummy)

        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        # return pos
        return robot_joints_dummy

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        velocity = 0.01
        acceleration = 0.01
        dt = 1.0 / 500  # 2ms
        lookahead_time = 0.2
        gain = 100

        robot_joints_dummy = joint_state[:6]
        for i in range(1, 7):
            # p.setJointMotorControl2(self.dummy_robot, i, p.POSITION_CONTROL, targetPosition=robot_joints_dummy[i-1])
            p.resetJointState(self.dummy_robot, i, robot_joints_dummy[i-1])

        dummy_ee_pos, dummy_ee_quat = p.getLinkState(self.dummy_robot, 6)[0], p.getLinkState(self.dummy_robot, 6)[1]

        ee_pose = [dummy_ee_pos[0], dummy_ee_pos[1], 0.265]

        dummy_joint_pos = p.calculateInverseKinematics(self.dummy_robot, 6, ee_pose , self.correct_ee_quat,
            residualThreshold=0.00001, maxNumIterations=1000, 
            lowerLimits=[self.initial_joint_state[k] - np.pi/2 for k in range(6)],
            upperLimits=[self.initial_joint_state[k] + np.pi/2 for k in range(6)],
            jointRanges=[12.566, 12.566, 6.282, 12.566, 12.566, 12.566],
            restPoses=[0* np.pi, -0.5* np.pi, 0.5* np.pi, -0.5* np.pi, -0.5* np.pi, 0]
            )
        
        dummy_joint_pos = list(dummy_joint_pos)
        robot_joints = dummy_joint_pos[:6]
        # robot_joints = joint_state[:6]
        t_start = self.robot.initPeriod()
        #get robot joints 
        actual_robot_joints = self.r_inter.getActualQ()
        error = np.array(robot_joints) - np.array(actual_robot_joints)
        for i in range(5):
            self.robot.servoJ(
                robot_joints, velocity, acceleration, dt, lookahead_time, gain
            )
            actual_robot_joints = self.r_inter.getActualQ()
            error = np.array(robot_joints) - np.array(actual_robot_joints)
            if np.linalg.norm(error) < 0.05:
                break

        # while np.linalg.norm(error) > 0.05:
        #     self.robot.servoJ(
        #         robot_joints, velocity, acceleration, dt, lookahead_time, gain
        #     )
        #     actual_robot_joints = self.r_inter.getActualQ()
        #     error = np.array(robot_joints) - np.array(actual_robot_joints)
        #     if np.linalg.norm(error) < 0.05:
        #         break


        if self._use_gripper:
            gripper_pos = int(joint_state[-1] * 255)
            print('gripper_pos', gripper_pos)
            # print("joint state", joint_state)
            # print("gripper position", gripper_pos)
            if gripper_pos == 255:
                trigger = True
            else:
                trigger = False
            # self.gripper.move(gripper_pos, 255, 10)
            self.gripper.move(trigger)
        self.robot.waitPeriod(t_start)
        p.stepSimulation()

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])

        #get joint positions of actual robot
        robot_joints = self.r_inter.getActualQ()
        #reset the dummy robot to the actual robot joints
        for i in range(1, 7):
            p.resetJointState(self.dummy_robot, i, robot_joints[i-1])
        
        #get the end effector position of the dummy robot
        ee_pos, ee_quat = p.getLinkState(self.dummy_robot, 6)[0], p.getLinkState(self.dummy_robot, 6)[1]

        #convert into numpy array
        ee_pos = np.array(ee_pos)

        return {
            "state" : ee_pos,
            "action" : ee_pos,
            "joint_positions": robot_joints,
            "joint_velocities": robot_joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot_ip = "192.168.1.10"
    ur = URRobotPybullet(robot_ip, no_gripper=True)
    # print(ur)
    # ur.set_freedrive_mode(True)
    joint = [0, -1.57, 1.57, -1.57, -1.57, 0, 1]
    
    ur._use_gripper = True
    
    
    print(ur.get_observations())
    for i in range(10):
        obs = ur.get_observations()
        curjoints = obs["joint_positions"]
        
        ur.command_joint_state(np.array(joint))
        


if __name__ == "__main__":
    main()
