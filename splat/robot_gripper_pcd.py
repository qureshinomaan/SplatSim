
import trimesh
import numpy as np

# Load your mesh
#read all files in the directory
import os
folder = 'pybullet-playground/meshes/ur5/visual/'
filenames = os.listdir(folder)


pverallpcds = []

for filename in filenames:
    if filename.endswith('.obj'):
        mesh_path = os.path.join(folder, filename)
        mesh = trimesh.load(mesh_path)

        # Sample points on the mesh surface
        number_of_points = 1000  # Number of points to sample
        points = trimesh.sample.sample_surface(mesh, number_of_points)
        #convert points to numpy array
        

        pverallpcds.append(points[0])



import open3d as o3d
#convert oveerall pcds into a single numpy array
overall_pcds = np.concatenate(pverallpcds, axis=0)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(overall_pcds)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])