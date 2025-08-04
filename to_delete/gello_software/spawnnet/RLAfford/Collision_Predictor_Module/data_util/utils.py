from pytorch_lightning import data_loader
import torch
import open3d
import numpy as np
import matplotlib.pyplot as plt

def from_transform_to_tensor(transform):
    pose = transform.p
    ori = transform.r
    tensor_pose = torch.zeros((1, 3), device=0)
    tensor_pose[0][0] = pose.x
    tensor_pose[0][1] = pose.y
    tensor_pose[0][2] = pose.z
    tensor_ori = torch.zeros((1, 4), device=0)
    tensor_ori[0][0] = ori.x
    tensor_ori[0][1] = ori.y
    tensor_ori[0][2] = ori.z
    tensor_ori[0][3] = ori.w
    return tensor_pose, tensor_ori

def visualization(pc_batch, heat_batch=None):

    pt1 = open3d.geometry.PointCloud()

    if heat_batch is None :
        heat_batch = [None] * pc_batch.shape[0]

    for pc, heat in zip(pc_batch, heat_batch) :
        pt1.points = open3d.utility.Vector3dVector(pc)
        print(pc.shape, "points")

        if heat is not None :
            blue = np.zeros(pc.shape)
            red = np.zeros(pc.shape)
            blue[:, 0] = 0.3
            blue[:, 1] = 0.4
            blue[:, 2] = 1
            red[:, 0] = 1
            color = heat.reshape(-1, 1)*red + (1-heat).reshape(-1, 1)*blue
            print("heatmap value between:", heat.min(), heat.max())
            pt1.colors = open3d.utility.Vector3dVector(color)

        open3d.visualization.draw_geometries([pt1])

def generate_ply(pc, heat=None, file_name="./temp.ply") :

    pt1 = open3d.geometry.PointCloud()

    if heat is None :
        heat = [None]

    pt1.points = open3d.utility.Vector3dVector(pc)

    if heat is not None :
        blue = np.zeros(pc.shape)
        red = np.zeros(pc.shape)
        blue[:, 0] = 0.3
        blue[:, 1] = 0.4
        blue[:, 2] = 1
        red[:, 0] = 1
        color = heat.reshape(-1, 1)*red + (1-heat).reshape(-1, 1)*blue
        print("heatmap value between:", heat.min(), heat.max())
        pt1.colors = open3d.utility.Vector3dVector(color)
        pt1.estimate_normals(
            search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                                max_nn=30))
        # open3d.visualization.draw_geometries([pt1], point_show_normal=True)
        open3d.io.write_point_cloud(file_name, pt1)
        
if __name__ == '__main__':

    data_path = "/home/boshi/Documents/Science/E2EAff/logs/franka_cabinet_PC_partial/ppo_pc_pure/ppo_pc_pure_seed-1/rawmap_2000.pt"
    selected_points = torch.load(data_path, map_location="cpu").numpy()
    pc = selected_points[..., :3].reshape(selected_points.shape[0], selected_points.shape[1], 3)
    map = selected_points[..., 3].reshape(selected_points.shape[0], selected_points.shape[1])
    print(np.max(map, axis=-1))
    visualization(pc, map)
    # for i in range(0, 17801, 200) :

    #     rawmap_name = "./maps/rawmap_" + str(i) + ".pt"
    #     target_name = "./plys/rawmap_" + str(i) + ".ply"

    #     selected_points = torch.load(rawmap_name, map_location="cpu").numpy()
    #     pc = selected_points[0, :, :3].reshape(selected_points.shape[1], 3)
    #     map = selected_points[0, :, 3].reshape(selected_points.shape[1])

    #     generate_ply(pc, map, target_name)
