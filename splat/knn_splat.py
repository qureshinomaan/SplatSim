import numpy as np
import open3d as o3d
import joblib
from sklearn.neighbors import KNeighborsClassifier

#load the knn model
knn = joblib.load('knn_model.pkl')


#load point cloud
pcd = o3d.io.read_point_cloud("../gaussian-splatting/output/ur_rss/point_cloud/iteration_30000/point_cloud.ply")

#load gt pcd 
pcd_1 = o3d.io.read_point_cloud("robot_pcd.ply")

points = np.asarray(pcd.points)

#transform the point cloud to sim frame
# -0.171670 0.028518 -0.018479 0.539438
# 0.021854 0.019779 -0.172500 -0.319750
# -0.026022 -0.171525 -0.022963 0.359500
# 0.000000 0.000000 0.000000 1.000000
transformation_matrix  = np.array([[0.177652, 0.009017, -0.075101, 0.602299],
                            [0.075529, -0.031549, 0.174877, -0.088422],
                            [-0.004104, -0.190276, -0.032555, 0.503297],
                            [0.0, 0.0, 0.0, 1.0]])

rotation_matrix = transformation_matrix[0:3, 0:3] 
translation_matrix = transformation_matrix[0:3, 3]

points = np.dot(rotation_matrix, points.T).T + translation_matrix

#z>0 and z<0.5

# points = points[np.where(points[:, 2] > 0.2)]
# points = points[np.where(points[:, 2] < 0.5)]
# #y < 0.5 and y > -0.5
# points = points[np.where(points[:, 1] < 0.5)]
# points = points[np.where(points[:, 1] > -0.5)]

# #x < 0.5 and x > -0.5
# points = points[np.where(points[:, 0] < 0.5)]
# points = points[np.where(points[:, 0] > -0.5)]
condition = (points[:, 2] > 0.2) & (points[:, 2] < 0.4) & (points[:, 1] < 0.2) & (points[:, 1] > 0.) & (points[:, 0] < 0.6) & (points[:, 0] > -0.)
points = points[np.where(condition)]

#predict the labels
y_pred = knn.predict(points)

#save the labels
print('y_pred:', y_pred.shape)
np.save('labels.npy', y_pred)

#unique labels
print('unique labels:', np.unique(y_pred))

#number of points for each label
for i in np.unique(y_pred):
    print('label:', i, 'points:', len(np.where(y_pred == i)[0]))

#assign random colors to each label
num_classes = len(np.unique(y_pred))
# colors = [[np.random.rand() for _ in range(3)] for _ in range(20)]
colors = [[0, 0, 0], [0.9, 0.9, 0.9], [0.0, 0.4, 0.9], [0.4, 0.9, 0.0], [0.9, 0.0, 0.4], [0.5, 0.5, 0.5], [0.0, 0.9, 0.5], [0.5, 0.0, 0.9], [0.9, 0.5, 0.0], [0.0, 0.5, 0.1], [0.1, 0.0, 0.5], [0.4, 0.8, 0.9], [0.5, 0.1, 0.0], [0.8, 0.9, 0.4], [0.9, 0.4, 0.8], [0.0, 0.0, 0.9]]

label_colors = colors


#get points only for the label 0
condition1 = np.where((y_pred > 0) & (y_pred < 20))
points = points[condition1]

pcd_colors = np.array([label_colors[int(label)] for label in y_pred])
pcd_colors = pcd_colors[condition1]

#color the points according to their labels
pcd.points = o3d.utility.Vector3dVector(points)

pcd.colors = o3d.utility.Vector3dVector(pcd_colors)


#visualize the point cloud
#coordinate frame
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])

o3d.visualization.draw_geometries([pcd, mesh_frame])