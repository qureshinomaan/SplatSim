import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot

import sys
sys.path.append('/home/nomaan/Desktop/corl24/main/RAFT/core')

from argparse import Namespace

from raft import RAFT

import torch

from utils import flow_viz
from utils.utils import InputPadder

import cv2
import flowiz as fz

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



class ServoingAgent(Agent):
    def __init__(
        self,
        target_image_path: str = './images/target.png',
        save_recordings: bool = True,
    ):
        #check if the target image path exists
        if not os.path.exists(target_image_path):
            raise FileNotFoundError(f"The file {target_image_path} does not exist.")

        args = Namespace(alternate_corr=False, corr_levels=4, corr_radius=4, dropout=0, mixed_precision=False, model='models/raft-chairs.pth', path='demo-frames/', small=False)

        self.flow_model = torch.nn.DataParallel(RAFT(args))
        self.flow_model.load_state_dict(torch.load('/home/nomaan/Desktop/corl24/main/RAFT/models/raft-chairs.pth'))


        self.flow_model = self.flow_model.module
        self.flow_model.to('cuda')
        self.flow_model.eval()

        self.target_image_path = target_image_path
        self.target_image = cv2.imread(self.target_image_path)
        self.target_image = cv2.cvtColor(self.target_image, cv2.COLOR_BGR2RGB)

        #change the target image to tensor
        self.target_image = torch.from_numpy(self.target_image).permute(2, 0, 1).float()
        self.padder = InputPadder(self.target_image .shape)
        self.target_image = self.padder.pad(self.target_image )[0].unsqueeze(0).to('cuda')

        dummy_depth = np.zeros((480, 640))
        getInteractionMatrix(dummy_depth)

        self.vel = torch.randn(3, 1).to('cuda')

        self.use_true_depth = True

        self.total_iters = 0

        self.save_recordings = save_recordings
        
        if self.save_recordings:
            self.target_name = os.path.splitext(os.path.basename(target_image_path))[0]
            self.output_dir = f'./recordings/{self.target_name}'
            
            # Create main directory and subdirectories
            self.subdirs = {
                'rgb': f'{self.output_dir}/rgb',
                'depth': f'{self.output_dir}/depth',
                'flow': f'{self.output_dir}/flow',
                'interaction': f'{self.output_dir}/interaction'
            }
            
            # Create all directories
            for dir_path in self.subdirs.values():
                os.makedirs(dir_path, exist_ok=True)

            #clear all files in the directories
            for dir_path in self.subdirs.values():
                for file in os.listdir(dir_path):
                    os.remove(os.path.join(dir_path, file))
            
            self.frame_count = 0


            

    def act(self, obs_dict: Dict[str, np.ndarray]) -> np.ndarray:
        print('target_image_path:', self.target_image_path)

        current_image = obs_dict["wrist_rgb"]
        #show the current image
        cv2.imshow('Current Image', current_image)
        cv2.waitKey(1)
        # current_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB)
        
        # Save recordings if enabled
        if self.save_recordings:
            # Save RGB
            cv2.imwrite(f'{self.subdirs["rgb"]}/frame_{self.frame_count:04d}.png', 
                        cv2.cvtColor(current_image, cv2.COLOR_RGB2BGR))
            
            # Save depth visualization
            depth = obs_dict["depth"]
            depth = np.clip(depth, 0, 30)
            depth_viz = (depth.squeeze(0)/np.max(depth.squeeze(0))*255).astype(np.uint8)
            cv2.imwrite(f'{self.subdirs["depth"]}/frame_{self.frame_count:04d}.png', depth_viz)

        # Now convert to tensor
        current_image = torch.from_numpy(current_image).permute(2, 0, 1).float()
        current_image = self.padder.pad(current_image)[0].unsqueeze(0).to('cuda')

        #if last image exists, load it
        flow_depth = None
        if not self.use_true_depth and os.path.exists('./images/last_image.png'):
            last_image = cv2.imread('./images/last_image.png')
            last_image = cv2.cvtColor(last_image, cv2.COLOR_BGR2RGB)
            last_image = torch.from_numpy(last_image).permute(2, 0, 1).float()
            last_image = self.padder.pad(last_image)[0].unsqueeze(0).to('cuda')
            flow_low, flow_up = self.flow_model(last_image, current_image, iters=20, test_mode=True)
            flow_depth = np.linalg.norm(flow_up.detach().cpu().numpy(), axis=1).squeeze(0)
            # flow_depth = 0.4*(1/(1+np.exp(-1/flow_depth)) - 0.5)
            flow_depth = 1/flow_depth

        depth = depth.squeeze(0) 

        

        #compute the flow
        flow_low, flow_up = self.flow_model(current_image, self.target_image, iters=60, test_mode=True)
        # Visualize the flow
        flow_up_np = flow_up[0].detach().permute(1,2,0).cpu().numpy()
        flow_viz_img = flow_viz.flow_to_image(flow_up_np)
        cv2.imshow('Flow Visualization', flow_viz_img)
        cv2.waitKey(1)

        flow_depth = depth 

        
        _, Lsx, Lsy = getInteractionMatrix(flow_depth)
        Lsx = torch.from_numpy(Lsx).float().to('cuda')
        Lsy = torch.from_numpy(Lsy).float().to('cuda')

        Lsx = Lsx.view(Lsx.shape[0] * Lsx.shape[1], 6, 1)[:, :6, :]
        Lsy = Lsy.view(Lsy.shape[0] * Lsy.shape[1], 6, 1)[:,:6,:]

        L = torch.cat((Lsx, Lsy), dim=2).transpose(1, 2)
        L = L.reshape(L.shape[0] * L.shape[1], 6)


        #L is shape (480*640, 6, 2)
        flow_up = flow_up.view(flow_up.shape[2] * flow_up.shape[3], 2, 1)
        flow_up = flow_up.reshape(flow_up.shape[0] * flow_up.shape[1], 1)


        ### one way to reshape the flow
        # flow_up = flow_up.squeeze(0).permute(1, 2, 0)
        # flow_up_1 = flow_up[:, :, 0].reshape(flow_up.shape[0] * flow_up.shape[1], 1)
        # flow_up_2 = flow_up[:, :, 1].reshape(flow_up.shape[0] * flow_up.shape[1],1)
        # flow_up = torch.cat((flow_up_1, flow_up_2), dim=0)

        #flow_up is shape (480*640, 2, 1)
        #visualize the flow
        

        # L.T @ L @ vel = L.T @ flow_up
        # vel = (L.T @ L)^-1 @ L.T @ flow_up
        vel = torch.inverse(L.transpose(0, 1) @ L + 0.00*torch.diag(L.transpose(0, 1) @ L)) @ L.transpose(0, 1) @ flow_up

        vel = vel.squeeze().detach().cpu().numpy()
        vel = vel/100.
        # vel = np.clip(vel, -0.1, 0.1)

        print('Vel:', vel)
        print('Photometric Error:', self.calculate_photometric_error(self.target_image[0].permute(1, 2, 0).detach().cpu().numpy(), current_image[0].permute(1, 2, 0).detach().cpu().numpy()))
        #show the current image
  
        #save the last image
        cv2.imwrite('./images/last_image.png', current_image[0].permute(1, 2, 0).detach().cpu().numpy())
        
        if self.total_iters > 29:
            rotation_coeff = 0.0
        else:
            rotation_coeff = 0.1

        self.total_iters += 1

        # Save recordings if enabled
        if self.save_recordings:
            # Flow visualization was already computed earlier
            flow_viz_img = flow_viz.flow_to_image(flow_up_np)
            cv2.imwrite(f'{self.subdirs["flow"]}/frame_{self.frame_count:04d}.png', flow_viz_img)
            
            # Save interaction matrix visualization
            # interaction_viz = np.sqrt(np.sum(Lsx**2 + Lsy**2, axis=2))
            # interaction_viz = (interaction_viz/np.max(interaction_viz)*255).astype(np.uint8)
            # cv2.imwrite(f'{self.subdirs["interaction"]}/frame_{self.frame_count:04d}.png', interaction_viz)
            
            self.frame_count += 1

        # return [vel[0], vel[1], vel[2], vel[3]*0, vel[4]*0, -vel[5]*0.1, []]
        #### vel[0] move right
        #### vel[1] move down
        #### vel[2] move forward
        #### vel[3] look up
        #### vel[4] look right
        #### vel[5] look down
        return [0.0, 0.0, 0.0, 0.0, rotation_coeff, 0.0, [0]]


    def calculate_photometric_error(self, target, img):
        # Ensure the target and img have the same shape
        if target.shape != img.shape:
            raise ValueError("Images must have the same dimensions.")
        
        # Photometric error: mean squared difference
        pe = np.sum((target.astype(np.float32) - img.astype(np.float32))**2) / float(target.shape[0] * target.shape[1])
        return pe


def getInteractionMatrix(d1):
    Cx = d1.shape[0] / 2
    Cy = d1.shape[1] / 2
    kx = d1.shape[0] / 2
    ky = d1.shape[1] / 2
    xyz = np.zeros([d1.shape[0],d1.shape[1],3])
    Lsx = np.zeros([d1.shape[0],d1.shape[1],6])
    Lsy = np.zeros([d1.shape[0],d1.shape[1],6])
    med = np.median(d1)
    xyz = np.fromfunction(lambda i, j, k : 0.5*(k-1)*(k-2)*(j-float(Cx))/float(kx)- k*(k-2)*(i-float(Cy))/float(ky) + 0.5*k*(k-1)*((d1[i.astype(int), j.astype(int)]==0)*med + d1[i.astype(int), j.astype(int)]), (d1.shape[0], d1.shape[1], 3), dtype=float)
    Lsx = np.fromfunction(lambda i, j, k : (k==0).astype(int) * -1/xyz[i.astype(int), j.astype(int),2] + (k==2).astype(int) * xyz[i.astype(int), j.astype(int),0]/xyz[i.astype(int), j.astype(int),2] + (k==3).astype(int) * xyz[i.astype(int),j.astype(int),0]*xyz[i.astype(int),j.astype(int),1] + (k==4).astype(int)*(-(1+xyz[i.astype(int),j.astype(int),0]**2)) +(k==5).astype(int)*xyz[i.astype(int),j.astype(int),1], (d1.shape[0], d1.shape[1], 6), dtype=float)
    Lsy = np.fromfunction(lambda i, j, k : (k==1).astype(int) * -1/xyz[i.astype(int),j.astype(int),2] + (k==2).astype(int) * xyz[i.astype(int),j.astype(int),1]/xyz[i.astype(int), j.astype(int),2] + (k==3).astype(int) * (1+xyz[i.astype(int),j.astype(int),1]**2) + (k==4).astype(int)*-xyz[i.astype(int),j.astype(int),0]*xyz[i.astype(int),j.astype(int),1] +(k==5).astype(int)* -xyz[i.astype(int),j.astype(int),0], (d1.shape[0], d1.shape[1], 6), dtype=float)
    
    return None, Lsx, Lsy


if __name__ == "__main__":
    demo_agent = DiffusionAgent(port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0")
    obs = {
        "image": np.zeros((1, 1,  3, 240, 320)),
        "agent_pos": np.zeros((1, 4, 2)),
        # "joint_positions": np.array([0, 0, 0, 0, 0, 0, 0]),
        # "joint_velocities": np.array([0, 0, 0, 0, 0, 0, 0]),
        # "ee_pos_quat": np.zeros(7),
        # "gripper_position": np.array([0]),
    }

    action = demo_agent.act(obs)
    print('Action:', action)