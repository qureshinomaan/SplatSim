from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
import time

rtde_c = RTDEControl("localhost")

path = Path()
vel = 0.5
acc = 4.0
blend = 0.099
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.140, -0.400, 0.100, 0, 3.14, 0, vel, acc, 0.0]))
path.addEntry(PathEntry(PathEntry.MoveL, PathEntry.PositionTcpPose, [-0.140, -0.400, 0.300, 0, 3.14, 0, vel, acc, blend]))
path.addEntry(PathEntry(PathEntry.MoveL, PathEntry.PositionTcpPose, [-0.140, -0.600, 0.300, 0, 3.14, 0, vel, acc, blend]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.140, -0.600, 0.100, 0, 3.14, 0, vel, acc, blend]))
path.addEntry(PathEntry(PathEntry.MoveJ, PathEntry.PositionTcpPose, [-0.140, -0.400, 0.100, 0, 3.14, 0, vel, acc, 0.0]))

# First move given path synchronously
print("Move path synchronously...")
rtde_c.movePath(path, False)
print("Path finished!")

# Now move given path asynchronously
print("Move path asynchronously with progress feedback...")
rtde_c.movePath(path, True)

# Wait for start of asynchronous operation
while not rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.010)
print("Async path started.. ")

# Wait for end of asynchronous operation
waypoint = -1
while rtde_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
    time.sleep(0.2)
    new_waypoint = rtde_c.getAsyncOperationProgress()
    if new_waypoint != waypoint:
        waypoint = new_waypoint
        print("Moving to path waypoint ")

print("Async path finished...\n\n")
rtde_c.stopScript()

