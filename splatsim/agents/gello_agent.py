import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from splatsim.agents.agent import Agent
from splatsim.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB1", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # xArm
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6, 7),
    #     joint_offsets=(
    #         2 * np.pi / 2,
    #         2 * np.pi / 2,
    #         2 * np.pi / 2,
    #         2 * np.pi / 2,
    #         -1 * np.pi / 2 + 2 * np.pi,
    #         1 * np.pi / 2,
    #         1 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, 1, 1, 1, 1, 1),
    #     gripper_config=(8, 279, 279 - 50),
    # ),
    # panda
    # "/dev/cu.usbserial-FT3M9NVB": DynamixelRobotConfig(
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            3 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
            4 * np.pi / 2,
            -2 * np.pi / 2 + 2 * np.pi,
            3 * np.pi / 2,
            4 * np.pi / 2,
        ),
        joint_signs=(1, -1, 1, 1, 1, -1, 1),
        gripper_config=(8, 195, 152),
    ),
    # Left UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            0,
            1 * np.pi / 2 + np.pi,
            np.pi / 2 + 0 * np.pi,
            0 * np.pi + np.pi / 2,
            np.pi - 2 * np.pi / 2,
            -1 * np.pi / 2 + 2 * np.pi,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 20, -22),
    ),
    # Right UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            np.pi + 0 * np.pi,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            1 * np.pi,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 286, 248),
    ),
    # Custom UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT78LMLS-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
        5*np.pi/2, 6*np.pi/2, 0*np.pi/2, 0*np.pi/2, 1*np.pi/2, 3*np.pi/2
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 110, 68),
    ),
}


class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(port=port, start_joints=start_joints)

        # Keep the joint states within the range of [-pi, pi] for good operation
        joint_state = self._robot.get_joint_state()
        norm_joint_state = ((joint_state + np.pi) % (2 * np.pi)) - np.pi
        self.joint_state_offset = joint_state - norm_joint_state
        # Assume the last joint is the gripper
        self.joint_state_offset[-1] = 0


    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        joint_state = self._robot.get_joint_state() - self.joint_state_offset
        return joint_state
