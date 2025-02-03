import pybullet as p
import numpy as np
import open3d as o3d
from sklearn.neighbors import KNeighborsClassifier
import argparse
import yaml

from cameras import get_overall_pcd


def main(object_name):
    #connect to pybullet
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)

    #load the robot
    robot_path = args.robot
    robot_id = p.loadURDF(robot_path, useFixedBase=True, basePosition=[0.0, 0.0, -0.1])

    #get the joint states from args
    joint_states = args.joint_states

    #set the joint states
    for joint_index, joint_state in enumerate(joint_states):
        p.resetJointState(robot_id, joint_index, joint_state)

    #get number of links in the object
    num_links = p.getNumJoints(robot_id)

    link_pcds = []
    link_labels = []

    for link_index in range(0, num_links):

        for link_index_1 in range(0, num_links):
            if link_index_1 == link_index:
                #make the link visible
                p.changeVisualShape(robot_id, link_index_1, rgbaColor=[1, 1, 1, 1])
            else:
                p.changeVisualShape(robot_id, link_index_1, rgbaColor=[1, 1, 1, 0])

        pcd = get_overall_pcd()

        link_pcds.append(pcd)
        link_labels.append(link_index)



    #now prepare the data for knn
    segmented_points = []
    segmented_labels = []

    for pcd, label in zip(link_pcds, link_labels):
        segmented_points.append(np.asarray(pcd.points))
        segmented_labels.append(np.ones(len(pcd.points)) * label)

    #combine all the points
    X = np.vstack(segmented_points)
    y = np.hstack(segmented_labels)

    #train the knn
    print('fitting the knn model')
    knn = KNeighborsClassifier(n_neighbors=10)
    knn.fit(X, y)

    print('Done fitting the knn model')

    #visualise the knn predictions
    predictions = knn.predict(X)

    #assign colors to the predictions
    colors = [(np.random.rand(), np.random.rand(), np.random.rand()) for _ in range(num_links)]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(X)
    pcd_colors = np.array([colors[int(label)] for label in predictions])

    pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

    #visualize the knn predictions
    # o3d.visualization.draw_geometries([pcd])

    #load the splat pcd
    pcd_splat = o3d.io.read_point_cloud("/home/nomaan/Desktop/corl24/ocean_backup/gaussian-splatting/output/glasses/point_cloud/iteration_7000/point_cloud.ply")

    #load the transormation matrix (yaml file)
    with open("/home/nomaan/Desktop/corl24/ocean_backup/gaussian-splatting/object_configs/objects.yaml", "r") as file:
        object_configs = yaml.safe_load(file)

    #get the transformation matrix for the glasses
    transformation_matrix = np.array(object_configs["glasses"]["transformation"]['matrix'])

    splat_points = np.asarray(pcd_splat.points)

    #transform the points
    rotation_matrix = transformation_matrix[0:3, 0:3] 
    translation_matrix = transformation_matrix[0:3, 3]

    splat_points = np.dot(rotation_matrix, splat_points.T).T + translation_matrix

    #infer the labels
    splat_labels = knn.predict(splat_points)

    print('splat_labels:', splat_labels)

    #assign colors and visualize

    pcd_colors = np.array([colors[int(label)] for label in splat_labels])
    #create new pcd with the splat points and colors
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(splat_points)
    new_pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw_geometries([pcd, coordinate_frame])
    o3d.visualization.draw_geometries([new_pcd, coordinate_frame])


    #save labels
    np.save('glasses_labels.npy', splat_labels)





    #run the simulation
    while True:
        p.stepSimulation()  

    

if __name__ == '__main__':
    #argument parser
    parser = argparse.ArgumentParser()

    #add arguments with default values
    parser.add_argument('--robot', type=str, default='../pybullet-playground_2/urdf/sisbot.urdf')
    parser.add_argument('--joint_states', type=list, default=[0, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, 0.0, 0.7999999999999996, 0.0, -0.8000070728762431, 0.0, 0.7999947291384548, 0.799996381456464, 0.0, -0.799988452159267, 0.0, 0.7999926186486127])



    #parse the arguments
    args = parser.parse_args()

    main(object_name=args.obj)
