import numpy as np
import pybullet as p
import pybullet_data
from gello.agents.agent import Agent

class TextInterfaceAgent(Agent):
    def __init__(self):
        self.robot = None
        # TODO does this need to be set?
        self.joint_signs = [1] * 6
        self.default_joint = [0, -1.57, 1.57, -1.57, -1.57, 0, 1]
        self.num_joints = len(self.default_joint)
        self.last_action = np.array(self.default_joint)  # 7-DoF if needed

    def act(self, obs):
        try:
            user_input = input(f"\nEnter {self.num_joints} joint angles (radians) separated by spaces ('enter' to reuse last): ").strip()
            if user_input == "":
                angles = self.last_action
            else:
                angles = list(map(float, user_input.split()))

            if len(angles) != self.num_joints:
                print(f"⚠️ Enter exactly {self.num_joints} values.")
                angles = self.last_action
            else:
                angles = np.array(angles)
                self.last_action = angles

            return angles
        except Exception as e:
            print(f"Error: {e}")
            return self.last_action
