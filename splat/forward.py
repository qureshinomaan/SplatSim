import pybullet as p
import numpy as np
import pybullet_data
from get_segmented import get_segmented_pcd
from utils import *
import open3d as o3d
import pytorch3d.transforms 
import torch


scale = 1

transformation_matrix = np.eye(4)

scale = 0.33456
if scale != 1:
    transformation_matrix[:3, :3] = transformation_matrix[:3, :3] * (scale)
transformation_matrix = np.array([-0.272776812315, -0.074060574174, 0.179010376334, -0.366493761539,
                -0.193020626903, 0.130256026983, -0.240235880017, 0.399390488863,
                -0.016514312476, -0.299140989780, -0.148925781250, 0.708794176579,
                0.0, 0.0, 0.0, 1.0]).reshape(4, 4)


#inverse transformation matrix
inv_transformation_matrix_1 = np.linalg.inv(transformation_matrix)
# inv_transformation_matrix_2 = np.concatenate((transformation_matrix[:3, :3].T, -np.dot(transformation_matrix[:3, :3].T, transformation_matrix[:3, 3].reshape(-1, 1))), axis=1)

# inv_transformation_matrix_2 = np.concatenate((inv_transformation_matrix_2, np.array([0, 0, 0, 1]).reshape(1, 4)))

# segmented_pcd, colors = get_segmented_pcd(scale=scale, transformation_matrix=inv_transformation_matrix_1)
segmented_pcd, colors = get_segmented_pcd(1./scale)




def quaternion_to_rot_matrix(quat):
    """
    Convert a quaternion into a rotation matrix.
    """
    w, x, y, z = quat
    return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                     [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])

p.connect(p.DIRECT)
p.resetSimulation()
urdfRootPath = pybullet_data.getDataPath()
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
pandaUid = p.loadURDF('pybullet-playground/urdf/sisbot.urdf', useFixedBase=True, basePosition=[0.0, 0.0, -0.1], flags=flags, globalScaling=1)
# p.resetBasePositionAndOrientation(pandaUid, [0, 0, -0.1], [0, 0, 0, 1])
#reset base by applying transformation matrix

print('det:', np.linalg.det(inv_transformation_matrix_1[:3, :3]*scale))
# exit()

quaternion = pytorch3d.transforms.matrix_to_quaternion(torch.from_numpy(inv_transformation_matrix_1[:3, :3]*scale)).cpu().numpy()
# quaternion = get_quaternion_from_matrix(inv_transformation_matrix_1[:3, :3]*scale)
#convert from xyzw to wxyz
quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])


# p.resetBasePositionAndOrientation(pandaUid, inv_transformation_matrix_1[:3, 3]  , quaternion)
# exit()




# Set the robot to a known configuration
joint_poses = [0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]  # Example configuration
for i in range(len(joint_poses)):
    p.resetJointState(pandaUid, i, joint_poses[i])

overall_transformed = []
inputs_1 = []
inputs_2 = []


# Getting global axis and position of each joint
for l in range(2):
    for joint_index in range(8):
        joint_info = p.getJointInfo(pandaUid, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        joint_axis_local = np.array(joint_info[13])  # Local axis of the joint
        link_state = p.getLinkState(pandaUid, joint_index, computeForwardKinematics=True)
        link_world_pos = link_state[0]
        link_world_ori = link_state[1]

        if l == 0:
            input_1 = (link_world_pos[0], link_world_pos[1], link_world_pos[2], np.array(link_world_ori))
            inputs_1.append(input_1)
        if l == 1:
            input_2 = (link_world_pos[0], link_world_pos[1], link_world_pos[2], np.array(link_world_ori))
            inputs_2.append(input_2)
            r_rel, t = compute_transformation(inputs_1[joint_index], inputs_2[joint_index])
            rotmat = r_rel


            # Apply transformation to the segmented point cloud
            segment = segmented_pcd[joint_index]
            transformed_segment = np.dot(rotmat, segment.T).T + t
            # transformed_segment = []
            # for point in segment:
            #     point_transformed = np.dot(rotmat, point) + t
            #     transformed_segment.append(point_transformed)

            overall_transformed.append(transformed_segment)


        #place a sphere at the link_world_pos
        
        p.stepSimulation()
        print()
    

    joint_poses[1] = joint_poses[1] + 0.8
    joint_poses[2] = joint_poses[2] + 0.8
    joint_poses[3] = joint_poses[3] + 0.8
    joint_poses[4] = joint_poses[4] + 0.8
    joint_poses[6] = joint_poses[6] + 0.3

    for i in range(len(joint_poses)):
        p.resetJointState(pandaUid, i, joint_poses[i])


overall_transformed = np.concatenate(overall_transformed, axis=0)
#visualize the transformed point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(overall_transformed)
# pcd.transform(inv_transformation_matrix_1)

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
coordinate_frame2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
coordinate_frame2.transform(transformation_matrix)

o3d.visualization.draw_geometries([pcd, coordinate_frame, coordinate_frame2])

#save the transformed point cloud
o3d.io.write_point_cloud("point_cloud_transformed.ply", pcd)

exit()

while True:
    p.stepSimulation()

    

p.disconnect()
