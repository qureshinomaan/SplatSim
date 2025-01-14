import open3d as o3d
import numpy as np

#load first point cloud
# pcd = o3d.io.read_point_cloud("point_cloud_transformed.ply")
#load second point cloud
pcd2 = o3d.io.read_point_cloud("robot_pcd.ply")

#check pcd color
print(np.asarray(pcd2.colors)) 

#vis

#coordinate frame
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])

#visualize point cloud with color
o3d.visualization.draw_geometries([pcd2, mesh_frame])