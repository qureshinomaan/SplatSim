from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("127.0.0.1")

# Parameters
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300
joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]

# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
for i in range(1000):
    t_start = rtde_c.initPeriod()
    rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
    joint_q[0] += 0.001
    joint_q[1] += 0.001
    rtde_c.waitPeriod(t_start)

rtde_c.servoStop()
rtde_c.stopScript()
