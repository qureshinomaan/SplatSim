import os
import sys
import shutil
from argparse import ArgumentParser
from PIL import Image
import numpy as np
import torch
import torch.nn.functional as F
import Collision_Predictor_Module.where2act.code.w2a_utils as w2a_utils
from Collision_Predictor_Module.where2act.code.w2a_utils import get_global_position_from_camera
import cv2
from sapien.core import Pose
from env import Env
from camera import Camera
from robots.panda_robot import Robot

from pointnet2_ops.pointnet2_utils import furthest_point_sample

import matplotlib.pyplot as plt
cmap = plt.cm.get_cmap("jet")

# test parameters
parser = ArgumentParser()
parser.add_argument('--exp_name', type=str, help='name of the training run')
parser.add_argument('--shape_id', type=str, help='shape id')
parser.add_argument('--model_epoch', type=int, help='epoch')
parser.add_argument('--model_version', type=str, help='model version')
parser.add_argument('--result_suffix', type=str, default='nothing')
parser.add_argument('--device', type=str, default='cuda:0', help='cpu or cuda:x for using cuda on GPU number x')
parser.add_argument('--overwrite', action='store_true', default=False, help='overwrite if result_dir exists [default: False]')
eval_conf = parser.parse_args()


# load train config
train_conf = torch.load(os.path.join('logs', eval_conf.exp_name, 'conf.pth'))

# load model
model_def = w2a_utils.get_model_module(eval_conf.model_version)

# set up device
device = torch.device(eval_conf.device)
print(f'Using device: {device}')

# check if eval results already exist. If so, delete it.
result_dir = os.path.join('logs', eval_conf.exp_name, f'visu_critic_heatmap-{eval_conf.shape_id}-model_epoch_{eval_conf.model_epoch}-{eval_conf.result_suffix}')
if os.path.exists(result_dir):
    if not eval_conf.overwrite:
        response = input('Eval results directory "%s" already exists, overwrite? (y/n) ' % result_dir)
        if response != 'y':
            sys.exit()
    shutil.rmtree(result_dir)
os.mkdir(result_dir)
print(f'\nTesting under directory: {result_dir}\n')

# create models
network = model_def.Network(train_conf.feat_dim)

# load pretrained model
print('Loading ckpt from ', os.path.join('logs', eval_conf.exp_name, 'ckpts'), eval_conf.model_epoch)
data_to_restore = torch.load(os.path.join('logs', eval_conf.exp_name, 'ckpts', '%d-network.pth' % eval_conf.model_epoch))
network.load_state_dict(data_to_restore, strict=False)
print('DONE\n')

# send to device
network.to(device)

# set models to evaluation mode
network.eval()

# setup env
env = Env()

# setup camera
cam = Camera(env, random_position=True)
#cam = Camera(env, fixed_position=True)
#cam = Camera(env)
env.set_controller_camera_pose(cam.pos[0], cam.pos[1], cam.pos[2], np.pi+cam.theta, -cam.phi)
mat33 = cam.mat44[:3, :3]

# load shape
object_urdf_fn = '../data/where2act_original_sapien_dataset/%s/mobility_vhacd.urdf' % eval_conf.shape_id
object_material = env.get_material(4, 4, 0.01)
state = 'random-closed-middle'
if np.random.random() < 0.5:
    state = 'closed'
print('Object State: %s' % state)
env.load_object(object_urdf_fn, object_material, state=state)
cur_qpos = env.get_object_qpos()

# simulate some steps for the object to stay rest
still_timesteps = 0
wait_timesteps = 0
while still_timesteps < 5000 and wait_timesteps < 20000:
    env.step()
    env.render()
    cur_new_qpos = env.get_object_qpos()
    invalid_contact = False
    for c in env.scene.get_contacts():
        for p in c.points:
            if abs(p.impulse @ p.impulse) > 1e-4:
                invalid_contact = True
                break
        if invalid_contact:
            break
    if np.max(np.abs(cur_new_qpos - cur_qpos)) < 1e-6 and (not invalid_contact):
        still_timesteps += 1
    else:
        still_timesteps = 0
    cur_qpos = cur_new_qpos
    wait_timesteps += 1

if still_timesteps < 5000:
    printout(flog, 'Object Not Still!')
    flog.close()
    env.close()
    exit(1)

### use the GT vision
rgb, depth = cam.get_observation()
object_link_ids = env.movable_link_ids
gt_movable_link_mask = cam.get_movable_link_mask(object_link_ids)

# sample a pixel to interact
xs, ys = np.where(gt_movable_link_mask>0)
if len(xs) == 0:
    print('No Movable Pixel! Quit!')
    env.close()
    exit(1)
idx = np.random.randint(len(xs))
x, y = xs[idx], ys[idx]
Image.fromarray((rgb*255).astype(np.uint8)).save(os.path.join(result_dir, 'rgb.png'))
marked_rgb = (rgb*255).astype(np.uint8)
marked_rgb = cv2.circle(marked_rgb, (y, x), radius=3, color=(0, 0, 255), thickness=5)
Image.fromarray(marked_rgb).save(os.path.join(result_dir, 'point_to_interact.png'))

# get pixel 3D position (world)
position_world = get_global_position_from_camera(cam, depth, y, x)[:3]
print('Position World: %f %f %f' % (position_world[0], position_world[1], position_world[2]))

# prepare input pc
cam_XYZA_id1, cam_XYZA_id2, cam_XYZA_pts = cam.compute_camera_XYZA(depth)
cam_XYZA = cam.compute_XYZA_matrix(cam_XYZA_id1, cam_XYZA_id2, cam_XYZA_pts, depth.shape[0], depth.shape[1])
pt = cam_XYZA[x, y, :3]
ptid = np.array([x, y], dtype=np.int32)
mask = (cam_XYZA[:, :, 3] > 0.5)
mask[x, y] = False
pc = cam_XYZA[mask, :3]
grid_x, grid_y = np.meshgrid(np.arange(448), np.arange(448))
grid_xy = np.stack([grid_y, grid_x]).astype(np.int32)    # 2 x 448 x 448
pcids = grid_xy[:, mask].T
pc_movable = (gt_movable_link_mask > 0)[mask]
idx = np.arange(pc.shape[0])
np.random.shuffle(idx)
while len(idx) < 30000:
    idx = np.concatenate([idx, idx])
idx = idx[:30000-1]
pc = pc[idx, :]
pc_movable = pc_movable[idx]
pcids = pcids[idx, :]
pc = np.vstack([pt, pc])
pcids = np.vstack([ptid, pcids])
pc_movable = np.append(True, pc_movable)
pc[:, 0] -= 5
pc = torch.from_numpy(pc).unsqueeze(0).to(device)

input_pcid = furthest_point_sample(pc, train_conf.num_point_per_shape).long().reshape(-1)
pc = pc[:, input_pcid, :3]  # 1 x N x 3
pc_movable = pc_movable[input_pcid.cpu().numpy()]     # N
pcids = pcids[input_pcid.cpu().numpy()]
pccolors = rgb[pcids[:, 0], pcids[:, 1]]

# push through unet
feats = network.pointnet2(pc.repeat(1, 1, 2))[0].permute(1, 0)    # N x F

# setup robot
robot_urdf_fn = './robots/panda_gripper.urdf'
robot_material = env.get_material(4, 4, 0.01)
robot = Robot(env, robot_urdf_fn, robot_material)

def plot_figure(up, forward):
    # cam to world
    up = mat33 @ up
    forward = mat33 @ forward

    up = np.array(up, dtype=np.float32)
    forward = np.array(forward, dtype=np.float32)
    left = np.cross(up, forward)
    left /= np.linalg.norm(left)
    forward = np.cross(left, up)
    forward /= np.linalg.norm(forward)
    rotmat = np.eye(4).astype(np.float32)
    rotmat[:3, 0] = forward
    rotmat[:3, 1] = left
    rotmat[:3, 2] = up
    rotmat[:3, 3] = position_world - up * 0.1
    pose = Pose().from_transformation_matrix(rotmat)
    robot.robot.set_root_pose(pose)
    env.render()
    rgb_final_pose, _ = cam.get_observation()
    fimg = Image.fromarray((rgb_final_pose*255).astype(np.uint8))
    fimg.save(os.path.join(result_dir, 'action.png'))

# sample a random direction to query
gripper_direction_camera = torch.randn(1, 3).to(device)
gripper_direction_camera = F.normalize(gripper_direction_camera, dim=1)
gripper_forward_direction_camera = torch.randn(1, 3).to(device)
gripper_forward_direction_camera = F.normalize(gripper_forward_direction_camera, dim=1)

up = gripper_direction_camera
forward = gripper_forward_direction_camera
left = torch.cross(up, forward)
forward = torch.cross(left, up)
forward = F.normalize(forward, dim=1)

plot_figure(up[0].cpu().numpy(), forward[0].cpu().numpy())

dirs1 = up.repeat(train_conf.num_point_per_shape, 1)
dirs2 = forward.repeat(train_conf.num_point_per_shape, 1)

# infer for all pixels
with torch.no_grad():
    input_queries = torch.cat([dirs1, dirs2], dim=1)
    net = network.critic(feats, input_queries)
    result = torch.sigmoid(net).cpu().numpy()
    result *= pc_movable

    fn = os.path.join(result_dir, 'pred')
    resultcolors = cmap(result)[:, :3]
    pccolors = pccolors * (1 - np.expand_dims(result, axis=-1)) + resultcolors * np.expand_dims(result, axis=-1)
    w2a_utils.export_pts_color_pts(fn,  pc[0].cpu().numpy(), pccolors)
    w2a_utils.export_pts_color_obj(fn,  pc[0].cpu().numpy(), pccolors)
    w2a_utils.render_pts_label_png(fn,  pc[0].cpu().numpy(), result)

# close env
env.close()

