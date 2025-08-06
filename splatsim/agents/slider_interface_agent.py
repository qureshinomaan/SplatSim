import numpy as np
import pybullet as p
import pybullet_data
from splatsim.agents.agent import Agent

class SliderInterfaceAgent(Agent):
    def __init__(self):
        self.robot = None
        self.joint_signs = [1] * 6
        self.default_joint = [0, -1.57, 1.57, -1.57, -1.57, 0, 1]
        self.num_joints = len(self.default_joint)
        self.last_action = np.array(self.default_joint)
        self.slider_ids = []
        p.connect(p.GUI)
        self._init_sliders()

    def _init_sliders(self):
        # Create sliders for each joint
        self.slider_ids = []
        for i, val in enumerate(self.default_joint):
            slider_id = p.addUserDebugParameter(f"Joint {i+1}", -3.14, 3.14, val)
            self.slider_ids.append(slider_id)

    def act(self, obs):
        angles = []
        for slider_id in self.slider_ids:
            angle = p.readUserDebugParameter(slider_id)
            angles.append(angle)
        angles = np.array(angles)
        self.last_action = angles
        return angles