import open3d as o3d
import numpy as np

def get_point_indices_by_color(colors, target_color, tolerance=0.4):
    # Create a mask for points within the tolerance range of the target color
    color_diff = np.linalg.norm(colors - target_color, axis=1).reshape(-1, 1)
    mask = np.all(color_diff <= tolerance, axis=1)
    return np.where(mask)[0]

#load point cloud
def get_segmented_pcd(scale=1.0, transformation_matrix=np.eye(4)):
    pcd = o3d.io.read_point_cloud("robot_pcd.ply")

    # pcd = pcd.transform(transformation_matrix)

    #cooridnate frame
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    #visualize point cloud
    # o3d.visualization.draw_geometries([pcd, mesh_frame])

    #visualize point cloud
    # o3d.visualization.draw_geometries([pcd, mesh_frame])
    # color = [[0, 0, 0], [0.9, 0.9, 0.9], [0.0, 0.4, 0.9], [0.4, 0.9, 0.0], [0.9, 0.0, 0.4], [0.5, 0.5, 0.5], [0.0, 0.9, 0.5], [0.5, 0.0, 0.9], [0.9, 0.5, 0.0], [0.0, 0.5, 0.1], [0.1, 0.0, 0.5], [0.4, 0.8, 0.9], [0.5, 0.1, 0.0], [0.8, 0.9, 0.4], [0.9, 0.4, 0.8], [0.0, 0.0, 0.9]]
    color = [[0, 0, 0], [0.9, 0.9, 0.9], [0.0, 0.4, 0.9], [0.4, 0.9, 0.0], [0.9, 0.0, 0.4], [0.0, 0.9, 0.5], [0.5, 0.0, 0.9], [0.9, 0.5, 0.0], [0.0, 0.5, 0.1],  [0.1, 0.0, 0.5], [0.4, 0.8, 0.9], [0.5, 0.5, 0.5]]

    tolerances = [0.2, 0.65, 0.4, 0.4, 0.4, 0.45, 0.55, 0.5, 0.4, 0.4, 0.65, 0.5]

    ### [[0. 0. 0.]
#  [0. 0. 1.]
#  [0. 1. 0.]
#  [0. 1. 1.]
#  [1. 0. 0.]
#  [1. 0. 1.]
#  [1. 1. 0.]
#  [1. 1. 1.]]

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    colors_copy = colors.copy()
    #round colors to 2 decimal places
    colors = np.round(colors, 2)

    #find unique colors in the point cloud
    unique_colors = np.unique(colors, axis=0)
    #order the colors according to color
    # exit()
    idxs = []
    segmented_points = []
    color_out = []
    tmp = 0
    for c in color:
        # idx = np.where(np.all(colors == color, axis=1))[0]
        # idxs.append(idx)
        # print('total points:', len(idx))
        idx = get_point_indices_by_color(colors, c, tolerance=tolerances[tmp])
        idxs.append(idx)
        #remove the points from the point cloud
        segmented_points.append(points[idx])
        color_out.append(colors[idx])

        colors = np.delete(colors, idx, axis=0)
        points = np.delete(points, idx, axis=0)
        tmp += 1

    sum = 0
    for idx in idxs:
        sum += len(idx)

    for i in range(len(color_out)):
        print('color_out:', i, color_out[i].shape)
        color_out[i][:] = color[i]


    return segmented_points, color_out, idx

#find distance of between segmented points [0] and [1]
if __name__ == '__main__':
    segmented_points, color, idxs = get_segmented_pcd()
    for i in range(len(segmented_points)):
        print('segmented_points:', i, segmented_points[i].shape)

    #concatenate the segmented points
    segmented_points = np.concatenate(segmented_points, axis=0)
    color = np.concatenate(color, axis=0)

    print('segmented_points:', segmented_points.shape)
    print('color:', color.shape)

    #visualize the segmented points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(segmented_points)
    pcd.colors = o3d.utility.Vector3dVector(color)
    o3d.visualization.draw_geometries([pcd])