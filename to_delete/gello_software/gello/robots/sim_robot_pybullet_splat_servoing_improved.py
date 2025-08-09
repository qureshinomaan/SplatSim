import threading
import time
from typing import Any, Dict, Optional
import numpy as np
import zmq
import torch
from gaussian_splatting.gaussian_renderer import GaussianModel
from gaussian_splatting.scene.cameras import Camera
from argparse import ArgumentParser
from gaussian_splatting.arguments import PipelineParams
import pickle
from gaussian_splatting.gaussian_renderer import render
from gaussian_splatting.utils.sh_utils import RGB2SH

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

    def __init__(self, robot, host: str = "127.0.0.1", port: int = 5556):
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
                    raise NotImplementedError(f"Invalid method: {method}")

                self._socket.send(pickle.dumps(result))
            except zmq.error.Again:
                print("Timeout in ZMQLeaderServer serve")

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()
        self._context.term()

class GaussianRenderServer:
    def __init__(
        self,
        gaussian_path: str,
        host: str = "127.0.0.1",
        port: int = 5556,
    ):
        # Setup ZMQ server
        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)

        # Load Gaussian model
        self.gaussian_model = GaussianModel(3)
        self.gaussian_model.load_ply(gaussian_path)

        # Setup camera with required parameters
        self.camera = Camera(
            colmap_id=0,
            R=torch.eye(3, device='cpu'),
            # R = torch.tensor([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], device='cpu').float(),
            T=torch.tensor([0.0, 0.0, 0.0], device='cpu'),
            FoVx=1.375955594372348,
            FoVy=1.1025297299614814,
            image=torch.zeros((3, 480, 640), device='cpu'),  # Placeholder image
            gt_alpha_mask=None,
            image_name="virtual_camera",
            uid=0
        )

        # Setup rendering parameters
        parser = ArgumentParser(description="Testing script parameters")
        self.pipeline = PipelineParams(parser)
        self.background = torch.tensor([1, 1, 1], dtype=torch.float32, device="cuda")

        self.camera_new = None  # Will store the reinitialized camera
        self.clicked_points_3d = []

    def setup_camera(self):
        """Setup camera with specific parameters"""
        camera = Camera(
            colmap_id=0,
            R=torch.eye(3, device='cpu'),
            T=torch.tensor([0.0, 0.0, 0.0], device='cpu'),
            FoVx=1.375955594372348,
            FoVy=1.1025297299614814,
            image=torch.zeros((3, 480, 640), device='cpu'),  # Placeholder image
            gt_alpha_mask=None,
            image_name="virtual_camera",
            uid=0
        )
        camera.image_width = 640
        camera.image_height = 480
        return camera

    def setup_camera_servo(self, translation, rotation):
        """Reinitialize camera with current pose"""
        camera = Camera(
            colmap_id=0,
            R=rotation,
            T=translation,
            FoVx=1.375955594372348,
            FoVy=1.1025297299614814,
            image=torch.zeros((3, 480, 640), device='cpu'),  # Placeholder image
            gt_alpha_mask=None,
            image_name="virtual_camera",
            uid=0,
            # trans=translation.detach().cpu().numpy()
        )
        camera.image_width = 640
        camera.image_height = 480
        camera.znear = self.camera.znear
        camera.zfar = self.camera.zfar
        return camera

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Return rendered image from current camera pose"""
        # Reinitialize camera with current pose
        camera_new = self.setup_camera_servo(translation=self.camera.T, rotation=self.camera.R)
        self.camera_new = camera_new  # Store for potential future use
        
        # Render the image using reinitialized camera
        rendering = render(camera_new, self.gaussian_model, self.pipeline, self.background)
        
        # Get RGB and depth images
        rgb_image = rendering["render"]
        depth_image = rendering["depth"]

        # Convert RGB tensor to numpy array (CHW -> HWC)
        rgb_image = rgb_image.detach().cpu().numpy()
        rgb_image = np.transpose(rgb_image, (1, 2, 0))
        rgb_image = (rgb_image * 255).astype(np.uint8)

        # Convert depth tensor to numpy array
        depth_image = depth_image.detach().cpu().numpy()

        # Include camera info in observations
        camera_info = {
            "T": self.camera_new.T,
            "R": self.camera_new.R,
            "FoVx": self.camera_new.FoVx,
            "FoVy": self.camera_new.FoVy,
            "image_width": self.camera_new.image_width,
            "image_height": self.camera_new.image_height,
            "znear": self.camera_new.znear,
            "zfar": self.camera_new.zfar,
            "world_view_transform": self.camera_new.world_view_transform,
            "full_proj_transform": self.camera_new.full_proj_transform,
            "camera_center": self.camera_new.camera_center,
            "projection_matrix": self.camera_new.projection_matrix,
        }

        return {
            "wrist_rgb": rgb_image,
            "depth": depth_image,
            "camera_info": camera_info,
            "joint_positions": np.random.rand(7),
        }

    def num_dofs(self) -> int:
        return 7  # 3 for translation, 3 for rotation

    def get_joint_state(self) -> np.ndarray:
        # Return current camera pose as 6-DOF vector
        rotation_angles = self.get_euler_angles(self.camera.R)
        return np.concatenate([self.camera.T.numpy(), rotation_angles])

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        # Handle translation
        velocity = torch.tensor(joint_state[:3], device='cpu', dtype=torch.float32)
        # velocity = np.clip(velocity, -0.02, 0.02)
        self.camera.T = self.camera.T - velocity  # Negative because T is world origin in camera frame

        # Store the clicked points
        if len(joint_state[-1]) > len(self.clicked_points_3d):
            self.clicked_points_3d = joint_state[-1]  # This is the list of 3D points
            point = self.clicked_points_3d[-1]
            for k in range(100):
                # Create sphere with random noise
                xyz_sphere = torch.from_numpy(point + np.random.randn(3)*0.01).to(device='cuda').float().view(1, 3)
                rot_sphere = torch.from_numpy(np.array([0, 0, 0, 1])).to(device='cuda').float().view(1, 4)
                opacity_sphere = torch.from_numpy(np.array([100])).to(device='cuda').float().view(1, 1)
                scales_sphere = torch.from_numpy(np.ones(3)*-4).to(device='cuda').float().view(1, 3)
                features_dc_sphere = torch.from_numpy(RGB2SH(np.array([1, 0, 0]))).to(device='cuda').float().view(1, 1, 3)
                features_rest_sphere = torch.from_numpy(np.array([[0, 0, 0] for _ in range(15)])).to(device='cuda').float().view(1, 15, 3)

                # Append new point to the Gaussian model
                self.gaussian_model._xyz = torch.cat([self.gaussian_model._xyz, xyz_sphere], dim=0)
                self.gaussian_model._rotation = torch.cat([self.gaussian_model._rotation, rot_sphere], dim=0)
                self.gaussian_model._opacity = torch.cat([self.gaussian_model._opacity, opacity_sphere], dim=0)
                self.gaussian_model._scaling = torch.cat([self.gaussian_model._scaling, scales_sphere], dim=0)
                self.gaussian_model._features_dc = torch.cat([self.gaussian_model._features_dc, features_dc_sphere], dim=0)
                self.gaussian_model._features_rest = torch.cat([self.gaussian_model._features_rest, features_rest_sphere], dim=0)

        # Handle rotation
        angular_vel = torch.tensor(joint_state[3:6], device='cpu', dtype=torch.float32)
        # angular_vel = np.clip(angular_vel, -0.02, 0.02)
        
        theta = torch.norm(angular_vel)
        if theta > 1e-6:  # Avoid division by zero
            axis = angular_vel / theta
            K = torch.tensor([[0, -axis[2], axis[1]],
                            [axis[2], 0, -axis[0]],
                            [-axis[1], axis[0], 0]], device='cpu', dtype=torch.float32)
            R = torch.eye(3, device='cpu', dtype=torch.float32) + \
                torch.sin(theta) * K + \
                (1 - torch.cos(theta)) * torch.matmul(K, K)
            
            # Right multiply because we're rotating the world frame relative to the camera frame
            self.camera.R = torch.matmul(self.camera.R, R.transpose(0, 1))

    @staticmethod
    def get_euler_angles(rotation_matrix: torch.Tensor) -> np.ndarray:
        """Convert rotation matrix to euler angles (roll, pitch, yaw)"""
        sy = torch.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = torch.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            pitch = torch.atan2(-rotation_matrix[2, 0], sy)
            yaw = torch.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            roll = torch.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            pitch = torch.atan2(-rotation_matrix[2, 0], sy)
            yaw = 0

        return torch.stack([roll, pitch, yaw]).numpy()

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        while True:
            pass

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None:
        self.stop()
