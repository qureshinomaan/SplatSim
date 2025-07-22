import sys
import threading
import pybullet as p
from gello_software.gello.robots.sim_robot_pybullet_splat_6DOF import PybulletRobotServer
from pointcloud_utils import get_point_cloud_from_camera, save_point_cloud_as_ply
import time
import numpy as np
import yaml

def main(args):
    # Initialize the PyBullet simulator
    # p.setGravity(0, 0, -9.81)  # Set gravity

    # Get config
    with open('object_configs/objects.yaml', 'r') as file:
        robot_config = yaml.safe_load(file)

    if args.robot_name is not None:
        initial_joint_positions = robot_config[args.robot_name]['joint_states'][0]
        urdf_path = robot_config[args.robot_name]["urdf_path"][0]
    else:
        print("WARNING: no robot name, so using default values")
        initial_joint_positions = [0, 0, -np.pi/2, np.pi/2, 0, np.pi/2, 0]  # Replace with your desired values
        urdf_path = 'pybullet-playground_2/urdf/pybullet_ur5_gripper/robots/urdf/ur5e_with_shear_gripper.urdf'


    # Create an instance of the PybulletRobotServer
    robot_server = PybulletRobotServer(
        urdf_path=urdf_path,
        host='127.0.0.1',
        port=5556,
        print_joints=False,
        use_gripper=False, #True
    )

    # Start the server in a separate thread
    server_thread = threading.Thread(target=robot_server.serve)
    server_thread.start()

    try:
        p.setGravity(0, 0, -9.81)  # Set gravity
        num_joints = p.getNumJoints(robot_server.dummy_robot)
        
        # Let the simulation run for a bit so everything loads
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1/240)


        # Set initial joint positions (example for 7-DOF robot)
        
        if len(initial_joint_positions) > num_joints:
            print(f"Warning: Provided initial joint positions ({len(initial_joint_positions)}) exceed the number of joints ({num_joints}). Truncating to {num_joints} positions.")
            initial_joint_positions = initial_joint_positions[:num_joints]

        # Set the joints
        for joint_index, joint_pos in enumerate(initial_joint_positions):
            if joint_index >= num_joints:
                break
            p.resetJointState(robot_server.dummy_robot, joint_index, joint_pos)

        # Print joint information
        for i in range(num_joints):
            info = p.getJointInfo(robot_server.dummy_robot, i)
            print(f"Joint {i}: name={info[1].decode()}, type={info[2]}, axis={info[13]}")

        # Get joint limits, names, and current positions
        joint_indices = []
        joint_names = []
        joint_lows = []
        joint_highs = []
        joint_inits = []
        for i in range(p.getNumJoints(robot_server.dummy_robot)):
            info = p.getJointInfo(robot_server.dummy_robot, i)
            if info[2] == p.JOINT_REVOLUTE or info[2] == p.JOINT_PRISMATIC:
                joint_indices.append(i)
                joint_names.append(info[1].decode())
                joint_lows.append(info[8])
                joint_highs.append(info[9])
                if i < len(initial_joint_positions):
                    joint_inits.append(initial_joint_positions[i])
                else:
                    joint_inits.append(p.getJointState(robot_server.dummy_robot, i)[0])  # current position

        # Create sliders for each joint, initialized to current joint positions
        sliders = []
        for idx, name, low, high, init in zip(joint_indices, joint_names, joint_lows, joint_highs, joint_inits):
            slider = p.addUserDebugParameter(name, low, high, init)
            sliders.append(slider)

        print("Use the sliders in the PyBullet GUI to control the robot's joints.")

        i = 0
        while True:
            # Read slider values and set joint positions using position control
            max_joint_i = -1
            joint_positions = [0] * 30
            for idx, slider in zip(joint_indices, sliders):
                target_pos = p.readUserDebugParameter(slider)
                joint_positions[idx] = target_pos
                max_joint_i = max(max_joint_i, idx)
                p.setJointMotorControl2(
                    bodyIndex=robot_server.dummy_robot,
                    jointIndex=idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_pos,
                    force=500  # You may need to tune this value
                )
            joint_positions = joint_positions[:max_joint_i + 1]
            print(f"Joint positions:\n{joint_positions}\n")
            p.stepSimulation()
            time.sleep(1/240)
            i += 1

            if i == 100:
                # Capture and save the point cloud every 100 iterations
                points = get_point_cloud_from_camera(p)
                save_point_cloud_as_ply(points, f"scene_pointcloud_{i}.ply")
                print(f"Saved point cloud to scene_pointcloud_{i}.ply")

    except KeyboardInterrupt:
        print("Shutting down the server...")
    finally:
        robot_server.stop()
        server_thread.join()
        p.disconnect()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_name", type=str, default=None)

    args = parser.parse_args()
    main(args)