import trimesh
import numpy as np

# Load your mesh
mesh_path = '/Users/nomaan/Desktop/git/splat/pybullet-URDF-models/urdf_models/models/power_drill/textured.obj'  # Adjust the path and file format as needed
mesh = trimesh.load(mesh_path)

# Sample points on the mesh surface
number_of_points = 10000  # Number of points to sample
points = trimesh.sample.sample_surface(mesh, number_of_points)


# points is a tuple containing the sampled points and their corresponding face indices
sampled_points = points[0]  # This is a Nx3 numpy array of the sampled points

# Optionally, visualize the points (if you are using an interactive environment like Jupyter)
print('sampled_points:', sampled_points.shape)

import open3d as o3d
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(sampled_points)
#remove points below z =0
points = np.asarray(pcd.points)
points = points[points[:,2] < 0]
pcd.points = o3d.utility.Vector3dVector(points)


## load drill_point_cloud.ply
pcd2 = o3d.io.read_point_cloud("drill_point_cloud.ply")
#remove points above z =0 from pcd2
points = np.asarray(pcd2.points)
points = points[points[:,2] > 0]

pcd2.points = o3d.utility.Vector3dVector(points)

#coordinate frame
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

o3d.visualization.draw_geometries([pcd, pcd2, mesh_frame])

