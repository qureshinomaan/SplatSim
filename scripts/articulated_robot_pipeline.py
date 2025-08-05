import os
from argparse import ArgumentParser, Namespace

import pybullet as p
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from sklearn.neighbors import KNeighborsClassifier
import argparse
import torch
import yaml
from gaussian_splatting.scene import Scene

from gaussian_splatting.arguments import ModelParams, Namespace
from gaussian_splatting.gaussian_renderer import GaussianModel

from splatsim.utils.cameras import get_overall_pcd

class FlowStyleMatrixDumper(yaml.SafeDumper):
    def represent_sequence(self, tag, sequence, flow_style=None):
        """
        If every element of 'sequence' is itself a list,
        we'll treat this as a 'matrix' and dump the outer list
        in block style but each inner list in flow (inline) style.
        """
        if all(isinstance(item, list) for item in sequence) and len(sequence) > 0:
            # Outer list in block style
            node = super(FlowStyleMatrixDumper, self).represent_sequence(
                tag, sequence, flow_style=False
            )
            # Force each child (inner list) to be flow-style
            for child in node.value:
                child.flow_style = True
            return node
        else:
            return super(FlowStyleMatrixDumper, self).represent_sequence(
                tag, sequence, flow_style=flow_style
            )
        
def create_window(app, title, geometry_list, x, y):
    w = app.create_window(title, 800, 600, x, y)
    scene = gui.SceneWidget()
    scene.scene = rendering.Open3DScene(w.renderer)
    for i, geometry in enumerate(geometry_list):
        scene.scene.add_geometry(f"geometry_{i}", geometry, rendering.MaterialRecord())
    bounds = geometry_list[0].get_axis_aligned_bounding_box()
    scene.setup_camera(60, bounds, bounds.get_center())
    w.add_child(scene)

def main(args):

    #load the transformation matrix (yaml file)
    with open("./configs/object_configs/objects.yaml", "r") as file:
        object_configs = yaml.safe_load(file)

    if not os.path.exists("data/labels_path"):
        os.makedirs("data/labels_path", exist_ok=True)
    if not os.path.exists("data/pcds_path"):
        os.makedirs("data/pcds_path", exist_ok=True)

    parser = ArgumentParser(description="Testing script parameters")
    model = ModelParams(parser, sentinel=True)
    source_path = object_configs[args.robot_name]["source_path"]
    if not os.path.exists(source_path):
        raise FileNotFoundError(f"Source path not found: {source_path}")
    model_path = object_configs[args.robot_name]["model_path"]
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model path not found: {model_path}")
            
    dataset = model.extract(
        Namespace(
            sh_degree=3,
            # TODO get these from the object config
            source_path=source_path,
            model_path=model_path,
            images="images",
            depths="",
            resolution=-1,
            white_background=False,
            train_test_exp=False,
            data_device="cuda",
            eval=False,
        )
    )
    gaussians = GaussianModel(dataset.sh_degree)
    # This loads the .ply file into self.gaussians_backup
    # Use the minimum number of cameras. This is just for startup
    scene = Scene(
        dataset, gaussians, load_iteration=-1, shuffle=False, train_cam_indices=[0], test_cam_indices=[0],
    )
    # Extract the point cloud from the loaded gaussian splat
    robot_xyz = gaussians.get_xyz.cpu().detach().numpy()
    pcd_splat = o3d.geometry.PointCloud()
    pcd_splat.points = o3d.utility.Vector3dVector(robot_xyz)

    robot_path = object_configs[args.robot_name]["urdf_path"][0]
    initial_joint_positions = object_configs[args.robot_name]['joint_states'][0]

    #connect to pybullet
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)

    #load the robot
    base_position = object_configs[args.robot_name]["base_position"][0]
    robot_id = p.loadURDF(robot_path, useFixedBase=True, basePosition=base_position)

    #get the joint states from args
    joint_states = initial_joint_positions

    #set the joint states
    num_joints = p.getNumJoints(robot_id)
    for joint_index, joint_state in enumerate(joint_states):
        if joint_index >= num_joints:
            print(f"Warning: More joint indexes provided ({joint_index + 1}) than available joints ({num_joints}). Skipping excess joint states.")
            break
        # Set the joint state for the robot
        p.resetJointState(robot_id, joint_index, joint_state)

    #get number of links in the object
    num_links = p.getNumJoints(robot_id)

    link_pcds = []
    link_labels = []

    for link_index in range(-1, num_links):
        p.changeVisualShape(robot_id, link_index, rgbaColor=[1, 1, 1, 0])

    for link_index in range(-1, num_links):

        for link_index_1 in range(-1, num_links):
            if link_index_1 == link_index:
                #make the link visible
                p.changeVisualShape(robot_id, link_index_1, rgbaColor=[1, 1, 1, 1])
            else:
                p.changeVisualShape(robot_id, link_index_1, rgbaColor=[1, 1, 1, 0])

        pcd = get_overall_pcd(imgx = 1000, imgy = 1000)

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
    #save the pcd
    o3d.io.write_point_cloud("data/pcds_path/" + args.robot_name + '_pcd.ply', pcd)

    #visualize the knn predictions
    o3d.visualization.draw_geometries([pcd])

    #get the transformation matrix for the glasses
    #check if the robot_name is in the object_configs
    if args.robot_name not in object_configs:
        raise ValueError(f"Robot name {args.robot_name} not found in object_configs, please find the transformation matrix and update the configs/object_configs/objects.yaml file")
    
    transformation_matrix = np.array(object_configs[args.robot_name]["transformation"]['matrix'])
    

    splat_points = np.asarray(pcd_splat.points)

    #transform the points
    rotation_matrix = transformation_matrix[0:3, 0:3] 
    translation_matrix = transformation_matrix[0:3, 3]

    splat_points = np.dot(rotation_matrix, splat_points.T).T + translation_matrix

    #filter the points based on the aabb
    #get aabb of the robot from array X which is basically the pointcloud of the robot
    urdf_bbox_adjustment = object_configs[args.robot_name]["aabb"]["urdf_bbox_adjustment"]
    aabb = (
        (
            np.min(X[:, 0]) + urdf_bbox_adjustment[0][0], # x min
            np.min(X[:, 1]) + urdf_bbox_adjustment[1][0], # y min
            np.min(X[:, 2]) + urdf_bbox_adjustment[2][0], # z min
        ),
        (
            np.max(X[:, 0]) + urdf_bbox_adjustment[0][1], # x max
            np.max(X[:, 1]) + urdf_bbox_adjustment[1][1], # y max
            np.max(X[:, 2]) + urdf_bbox_adjustment[2][1], # z max
        )
    )


    aabb_list = [[float(val) for val in point] for point in aabb]
    #make the aabb_list upto 4 decimal places
    aabb_list = [[round(val, 4) for val in point] for point in aabb_list]

    # Update the object_configs
    object_configs[args.robot_name]["aabb"]['bounding_box'] = aabb_list
    #properly save the object_configs
    with open("./configs/object_configs/objects.yaml", "w") as file:
        yaml.dump(
            object_configs,
            file,
            Dumper=FlowStyleMatrixDumper,
            sort_keys=False,
            default_flow_style=False
        )

    print('aabb of the robot', aabb_list)

    print('splat points', splat_points.shape)

    splat_points = torch.from_numpy(splat_points).to(device='cuda')
    condition = (splat_points[:, 0] > aabb_list[0][0]) & (splat_points[:, 0] < aabb_list[1][0]) & (splat_points[:, 1] > aabb_list[0][1]) & (splat_points[:, 1] < aabb_list[1][1]) & (splat_points[:, 2] > aabb_list[0][2]) & (splat_points[:, 2] < aabb_list[1][2])
    condition = torch.where(condition)[0]
    splat_points = splat_points[condition]
    print('splat points', splat_points.shape)
    splat_points = splat_points.cpu().numpy()

    #infer the labels
    splat_labels = knn.predict(splat_points)

    #assign colors and visualize

    pcd_colors = np.array([colors[int(label)] for label in splat_labels])
    #create new pcd with the splat points and colors
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(splat_points)
    new_pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    # Open side by side view for original and new point clouds
    gui.Application.instance.initialize()
    # Combine geometries
    pcd_with_frame = [pcd, coordinate_frame]
    new_pcd_with_frame = [new_pcd, coordinate_frame]
    # Register windows
    create_window(gui.Application.instance, "Original Point Cloud", pcd_with_frame, 50, 50)
    create_window(gui.Application.instance, "New Point Cloud", new_pcd_with_frame, 900, 50)
    # Run app (blocks until all windows closed)
    gui.Application.instance.run()

    #save labels
    labels_fn = f"data/labels_path/{args.robot_name}_labels.npy"
    print(f"Saving labels to numpy file {labels_fn}...")
    np.save(labels_fn, splat_labels)

    # #run the simulation
    # print("Running simulation...")
    # while True:
    #     p.stepSimulation()  

    

if __name__ == '__main__':
    #argument parser
    parser = argparse.ArgumentParser()

    #add arguments with default values
    # parser.add_argument('--robot', type=str, default='../../submodules/pybullet-playground-wrapper/pybullet_playground/urdf/sisbot.urdf')
    # parser.add_argument('--joint_states', nargs='+', type=float, default=[0, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.0, 0.0, 0.0, 0.7999999999999996, 0.0, -0.8000070728762431, 0.0, 0.7999947291384548, 0.799996381456464, 0.0, -0.799988452159267, 0.0, 0.7999926186486127])
    # parser.add_argument('--splat_path', type=str, default='/home/jennyw2/data/output/robot_iphone/point_cloud/iteration_30000/point_cloud.ply')
    # parser.add_argument('--robot_name', type=str, default='robot_iphone')

    parser.add_argument('--robot_name', type=str, default='robot_jenny')

    #parse the arguments
    args = parser.parse_args()

    main(args=args)
