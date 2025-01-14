from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

rtde_c = RTDEControl("127.0.0.1")
rtde_r = RTDEReceive("127.0.0.1")
init_q = rtde_r.getActualQ()

# Target in the robot base
new_q = init_q[:]
new_q[0] += 0.20

# Move asynchronously in joint space to new_q, we specify asynchronous behavior by setting the async parameter to
# 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
# by the stopJ function due to the blocking behaviour.
rtde_c.moveJ(new_q, 1.05, 1.4, True)
time.sleep(0.2)
# Stop the movement before it reaches new_q
rtde_c.stopJ(0.5)

# Target in the Z-Axis of the TCP
target = rtde_r.getActualTCPPose()
target[2] += 0.10

# Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter to
# 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
# by the stopL function due to the blocking behaviour.
rtde_c.moveL(target, 0.25, 0.5, True)
time.sleep(0.2)
# Stop the movement before it reaches target
rtde_c.stopL(0.5)

# Move back to initial joint configuration
rtde_c.moveJ(init_q)

# Stop the RTDE control script
rtde_c.stopScript()
