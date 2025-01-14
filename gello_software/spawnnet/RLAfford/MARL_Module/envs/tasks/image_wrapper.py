import os
import numpy as np
import torch
from einops import rearrange
from isaacgym import gymapi, gymtorch

from tasks.hand_base.vec_task import VecTask

# Notes on the camera coordiante
# up: z
# right: y
# forward: x

cam_params = {
    'middle': {
        'pos': gymapi.Vec3(0.45, 0., 1.6),
        'quat': gymapi.Quat.from_euler_zyx(0.0, 30 / 360 * 6.282, 180 / 360 * 6.282)
    },
    'left': {
        'pos': gymapi.Vec3(0.1, -1., 1.2),
        'quat': gymapi.Quat.from_euler_zyx(0.0, 10 / 360 * 6.282, 100 / 360 * 6.282)
    },
    'right': {
        'pos': gymapi.Vec3(0.1, 1., 1.2),
        'quat': gymapi.Quat.from_euler_zyx(0.0, 10 / 360 * 6.282, 260 / 360 * 6.282)
    }
}

def to_numpy(x):
    if isinstance(x, torch.Tensor):
        return x.cpu().numpy()
    else:
        return x

class ImageWrapper(object):
    def __init__(self, vec_task):
        assert isinstance(vec_task, VecTask)
        self.vec_task = vec_task
        assert self.cfg['env']['enableCameraSensors'] is True, "enableCameraSensors must be True"
        width, height = 224, 224

        # Create camera
        camera_props = gymapi.CameraProperties()
        camera_props.horizontal_fov = 75.0
        camera_props.width = width
        camera_props.height = height
        camera_props.enable_tensors = True

        self.cam_names = ['middle', 'left', 'right']
        self.num_cams = len(self.cam_names)
        self.rgb_buffer = []
        self.depth_buffer = []
        cnt = 0
        self.cams = {}
        
        self.cam_handles = []
        for _ in range(self.num_envs):
            self.cam_handles.append(dict())
                
        for cn in self.cam_names:
            self.cams[cn] = []
            for i in range(self.num_envs):
                cam = self.gym.create_camera_sensor(self.env_ptr_list[i], camera_props)
                cam_transform = gymapi.Transform(cam_params[cn]['pos'], cam_params[cn]['quat'])
                self.gym.set_camera_transform(cam, self.env_ptr_list[i], cam_transform)
                self.cams[cn].append(cnt)
                
                self.cam_handles[i][cn] = cam
                # TODO This is a bug with isaac gym. The cam is only the environment-specific handle
                # TODO However, the correct way to get multiple view from different envionrment is to use a global handle with env 0 ( See the refresh_img_obs function)
                cnt += 1
        self.depth_clip = 2.
        self.recorded_frames = []
        self.recording = False

    def __getattr__(self, attr):
        try:
            return getattr(self.vec_task, attr)
        except:
            return getattr(self.vec_task.task, attr)

    def _refresh_image_obs(self):
        self.gym.render_all_camera_sensors(self.sim)
        self.gym.start_access_image_tensors(self.sim)  # Blocking. Prevent further writing.
        self.rgb_buffer, self.depth_buffer = [], []
        for v, cn in enumerate(self.cam_names):
            for i in range(self.num_envs):
                # TODO Below use env[0]. This is a bug with isaacgym
                rgb = self.gym.get_camera_image_gpu_tensor(
                    self.sim, self.env_ptr_list[0], self.cams[cn][i], gymapi.IMAGE_COLOR)
                depth = self.gym.get_camera_image_gpu_tensor(
                    self.sim, self.env_ptr_list[0], self.cams[cn][i], gymapi.IMAGE_DEPTH)

                self.rgb_buffer.append(gymtorch.wrap_tensor(rgb)[:, :, :3])
                self.depth_buffer.append(-gymtorch.wrap_tensor(depth))  # Revert depth
        self.rgb_buffer = torch.stack(self.rgb_buffer, dim=0)
        self.depth_buffer = torch.stack(self.depth_buffer, dim=0)[:, :, :, None]
        self.depth_buffer[self.depth_buffer == float('inf')] = self.depth_clip
        self.depth_buffer[self.depth_buffer > self.depth_clip] = self.depth_clip
        self.gym.end_access_image_tensors(self.sim)

    def step(self, actions):
        obs, reward, done, info = self.vec_task.step(actions)
        self._refresh_image_obs()
        info['rgb'] = rearrange(self.rgb_buffer, '(v b) h w c -> b v h w c', b=self.num_envs, v=self.num_cams)
        info['depth'] = rearrange(self.depth_buffer, '(v b) h w c -> b v h w c', b=self.num_envs, v=self.num_cams)
        info['proprio'] = self._get_base_observation()
        if self.recording:
            rgb_image = to_numpy(info['rgb'])
            self.recorded_frames.append(rgb_image)
        # self.cv_render(mode='rgb')
        return obs, reward, done, info
    
    def reset(self, *args, **kwargs):        
        if self.cfg["task"]["target"] == 'close' and os.environ.get('CAM_MODE', 'unvaried') == 'varied':
            for i in range(self.num_envs):
                for cn in self.cam_names:
                    mu_transform_pos = cam_params[cn]['pos']
                    l, u = np.array([-0.02, -0.2, -0.04]), np.array([0.12, 0.2, 0.01])
                    sigma_transform_pos = np.random.random(3) * (u - l) + l
                    transform_pos = np.array([mu_transform_pos.x, mu_transform_pos.y, mu_transform_pos.z]) + sigma_transform_pos
                                        
                    transform_pos = gymapi.Vec3(*transform_pos)
                    modded_transform = gymapi.Transform(transform_pos, cam_params[cn]['quat'])
                    
                    self.gym.set_camera_transform(self.cam_handles[i][cn], self.env_ptr_list[i], modded_transform)
                    
        return self.vec_task.reset(*args, **kwargs)

    def print_current_viewer_pose(self):
        t = self.gym.get_viewer_camera_transform(self.viewer, self.env_ptr_list[0])
        print("Camera pos: {} rotation: {}".format(t.p, t.r))

    def cv_render(self, mode='rgb'):
        import cv2
        import numpy as np
        from utils.visualization_utils import make_grid
        if mode == 'rgb':
            img = to_numpy(self.rgb_buffer)
        else:
            img = to_numpy(self.depth_buffer)
            img /= self.depth_clip
            img = np.repeat(img, 3, axis=-1)
        img = rearrange(img, '(v b) h w c -> (b v) h w c', b=self.num_envs, v=self.num_cams)
        img = make_grid(img, ncol=self.num_cams, padding=5)  # output 0-255
        img = img.astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow('camera', img)
        cv2.waitKey(1)

    def start_record(self):
        self.recorded_frames = []
        self.recording = True

    def end_record(self, output_name=None):
        from simple_bc.utils.visualization_utils import make_grid_video_from_numpy
        videos = np.array(self.recorded_frames)
        videos = rearrange(videos, 't b v h w c -> (b v) t h w c')[..., :3]
        if output_name is not None:
            make_grid_video_from_numpy(videos, ncol=self.num_cams * 4, output_name=output_name)
        self.reccorded_frames = []
        return videos
