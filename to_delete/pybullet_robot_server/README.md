# Adjust the import based on your file structure

def main():
    # Initialize the PyBullet simulator
    p.connect(p.GUI)  # Use p.DIRECT for no GUI
    p.setGravity(0, 0, -9.81)  # Set gravity

    # Create an instance of the PybulletRobotServer
    robot_server = PybulletRobotServer(
        urdf_path='../gaussian-splatting/pybullet-playground/urdf/sisbot.urdf',  # Adjust the path as needed
        host='127.0.0.1',
        port=5556,
        print_joints=False,
        use_gripper=True
    )

    # Start the server in a separate thread
    server_thread = threading.Thread(target=robot_server.serve)
    server_thread.start()

    try:
        # Keep the main thread alive, allowing the server to run
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
```

### Explanation:
1. **Imports**: The script imports necessary modules, including `pybullet` and the `PybulletRobotServer` class from your existing code.
2. **Main Function**: The `main()` function initializes the PyBullet simulator and creates an instance of `PybulletRobotServer`.
3. **Threading**: The server runs in a separate thread to allow the main thread to continue executing the simulation.
4. **Simulation Loop**: The main thread enters a loop where it continuously steps the simulation until interrupted (e.g., by pressing Ctrl+C).
5. **Graceful Shutdown**: When the script is interrupted, it stops the server and disconnects from the PyBullet simulator.

### Usage:
- Save this script in the same directory as your `sim_robot_pybullet_splat_6DOF.py` file or adjust the import path accordingly.
- Run the script using Python. Make sure you have the necessary dependencies installed (like PyBullet and any other libraries your code requires).