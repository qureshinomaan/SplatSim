import sys
import threading
import pybullet as p
from gello_software.gello.robots.sim_robot_pybullet_splat_6DOF import PybulletRobotServer  # Adjust the import based on your file structure

def main():
    # Initialize the PyBullet simulator
    # p.setGravity(0, 0, -9.81)  # Set gravity

    # Create an instance of the PybulletRobotServer
    robot_server = PybulletRobotServer(
        urdf_path='./pybullet-playground_2/urdf/sisbot.urdf',  # Adjust the path as necessary
        host='127.0.0.1',
        port=5556,
        print_joints=False,
        use_gripper=True
    )

    # Start the server in a separate thread
    server_thread = threading.Thread(target=robot_server.serve)
    server_thread.start()

    try:
        # Keep the main thread alive to keep the server running
        while True:
            p.stepSimulation()  # Step the simulation
    except KeyboardInterrupt:
        print("Shutting down the server...")
    finally:
        robot_server.stop()  # Stop the server
        server_thread.join()  # Wait for the server thread to finish
        p.disconnect()  # Disconnect from the PyBullet simulator

if __name__ == "__main__":
    main()