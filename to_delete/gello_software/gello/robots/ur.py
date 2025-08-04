from typing import Dict

import numpy as np

from gello.robots.robot import Robot


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):
        import rtde_control
        import rtde_receive

        try:
            print(robot_ip)
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
            print('in try')
        except Exception as e:
            print(e)
            print(robot_ip)
            print('I am in except')

        self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not no_gripper:
            from gello.robots.robotiq_gripper import RobotiqGripper
            from gello.robots.custom_gripper import CustomGripper

            self.gripper = RobotiqGripper()
            # self.gripper = CustomGripper()
            self.gripper.connect(hostname=robot_ip, port=63352)
            # self.gripper.connect()
            print("gripper connected")
            self.gripper.activate()
            print("gripper activated")

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
        import time

        time.sleep(0.01)
        gripper_pos = self.gripper.get_current_position()
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255

    # def _get_gripper_pos(self) -> float:
    #     # import time

    #     # time.sleep(0.01)
        
    #     gripper_pos = self.gripper.get_current_position()
    #     # gripper_pos = gripper_pos - 2604
    #     # gripper_pos = gripper_pos // 1290
    #     #flip the gripper position
    #     gripper_pos = (1 - gripper_pos)
    #     print("gripper position", gripper_pos)
    #     print("gripper position", gripper_pos)
    #     assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
    #     return gripper_pos 

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.r_inter.getActualQ()
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

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

        robot_joints = joint_state[:6]
        t_start = self.robot.initPeriod()
        self.robot.servoJ(
            robot_joints, velocity, acceleration, dt, lookahead_time, gain
        )
        if self._use_gripper:
            gripper_pos = int(joint_state[-1] * 255)
            # print("joint state", joint_state)
            # print("gripper position", gripper_pos)
            if gripper_pos == 255:
                trigger = True
            else:
                trigger = False
            self.gripper.move(gripper_pos, 555, 50)
            # self.gripper.move(trigger)
        # self.robot.waitPeriod(t_start)

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
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot_ip = "192.168.0.11"
    ur = URRobot(robot_ip, no_gripper=True)
    # print(ur)
    # ur.set_freedrive_mode(True)
    joint = [1.5681470689206045, -1.068216007103522, 2.1378836578411438, -2.6390424613000025, -1.5699116232851198, -0.0018527878551533776, 1]
    
    # ur._use_gripper = True
    
    
    print(ur.get_observations())
    for i in range(10):
        obs = ur.get_observations()
        curjoints = obs["joint_positions"]
        
        ur.command_joint_state(np.array(joint))
        


if __name__ == "__main__":
    main()
