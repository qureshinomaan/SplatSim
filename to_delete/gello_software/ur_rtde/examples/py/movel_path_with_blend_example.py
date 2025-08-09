from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("127.0.0.1")

velocity = 0.5
acceleration = 0.5
blend_1 = 0.0
blend_2 = 0.02
blend_3 = 0.0
path_pose1 = [-0.143, -0.435, 0.20, -0.001, 3.12, 0.04, velocity, acceleration, blend_1]
path_pose2 = [-0.143, -0.51, 0.21, -0.001, 3.12, 0.04, velocity, acceleration, blend_2]
path_pose3 = [-0.32, -0.61, 0.31, -0.001, 3.12, 0.04, velocity, acceleration, blend_3]
path = [path_pose1, path_pose2, path_pose3]

# Send a linear path with blending in between - (currently uses separate script)
rtde_c.moveL(path)
rtde_c.stopScript()
