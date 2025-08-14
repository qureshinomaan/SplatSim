import numpy as np
from typing import List
import pybullet as p
import pybullet_data
from splatsim.agents.agent import Agent
import pickle
import os
from tqdm import tqdm
from gello.env import RobotEnv
import cv2

class ReplayTrajectoryAgent(Agent): 
    def __init__(self, traj_folder: str, env: RobotEnv, save_images: bool = False):
        self.robot = None
        # TODO does this need to be set?
        self.joint_signs = [1] * 6

        # env is using for setting the pose of recorded objects in the scene
        self.env = env

        self.last_action = np.array([0, 0, 0, 0, 0, 0, 1])  # 7-DoF

        self.traj_folder = traj_folder
        self.image_folder = None
        self.save_images = save_images
        self.traj_index = -1
        self.traj_subfolders = sorted(os.listdir(traj_folder))
        self.load_next_recorded_trajectory()

    def load_next_recorded_trajectory(self):
        self.traj_index += 1
        print(f"Loading trajectory {self.traj_index + 1}/{len(self.traj_subfolders)}: {self.traj_subfolders[self.traj_index]}")
        if self.save_images:
            self.image_folder = os.path.join(self.traj_folder, self.traj_subfolders[self.traj_index], 'images_1')
            os.makedirs(self.image_folder, exist_ok=True)

            # Delete existing images in the folder if any
            for filename in os.listdir(self.image_folder):
                file_path = os.path.join(self.image_folder, filename)
                if os.path.isfile(file_path):
                    os.remove(file_path)
        
        # take only those filenames which end with .pkl
        file_names = os.listdir(os.path.join(self.traj_folder, self.traj_subfolders[self.traj_index]))
        file_names.sort()

        self.trajectory_iter = iter(file_names)

    def next_trajectory_step(self):
        file = next(self.trajectory_iter, None)
        if file is None:
            if self.traj_index + 1 < len(self.traj_subfolders):
                self.load_next_recorded_trajectory()
                return self.next_trajectory_step()
            else:
                print("No more recorded trajectories.")
                return None
        if not file.endswith('.pkl'):
            print(f"Skipping non-pkl file: {file}")
            return self.next_trajectory_step()
        file_path = os.path.join(self.traj_folder, self.traj_subfolders[self.traj_index], file)
        file = open(file_path, 'rb')
        data = pickle.load(file)

        cur_joint = data['joint_positions'][:]
        cur_joint = cur_joint.tolist()
        # Add the world joint to the recorded joint state
        # cur_joint = [0] + cur_joint 
        cur_joint = np.array(cur_joint)

        object_list = [object_position_key[:-len("_position")] for object_position_key in data.keys() if object_position_key.endswith("_position")]
        # gripper_position is for gello integration. Ignore this value
        if "gripper" in object_list:
            object_list.remove("gripper")
        for object_name in object_list:
            cur_object_position = np.array(data[object_name + '_position'])
            cur_object_rotation = np.array(data[object_name + '_orientation'])
            # cur_object_rotation = np.roll(cur_object_rotation, 1)
            # Disable gravity for objects when replaying a trajectory so that there's no jittering
            self.env._robot.set_object_pose(object_name, cur_object_position, cur_object_rotation, use_gravity=False)

        return cur_joint

    def act(self, obs):
        angles = self.next_trajectory_step()
        if angles is None:
            print("No more trajectory steps available.")
            return self.last_action
        else:
            if self.save_images:
                for image_name in [image_name for image_name in obs.keys() if image_name.endswith("_rgb") and obs[image_name] is not None]:
                    frame = obs[image_name]
                    frame = np.transpose(frame.detach().cpu().numpy(), (1, 2, 0))  # CxHxW -> HxWxC
                    frame = (frame * 255).astype(np.uint8)
                    image_index = len(os.listdir(self.image_folder))
                    image_path = os.path.join(self.image_folder, f"{image_name}_{image_index:05d}.png")
                    cv2.imwrite(image_path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            self.last_action = angles
            return angles
