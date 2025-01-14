import pybullet as p
import numpy as np
import pybullet_data
import time


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
# pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
pandaUid = p.loadURDF('pybullet-playground/urdf/sisbot_1.urdf', useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)
# pandaUid = p.loadURDF('ros_industrial_training/training/ref/2.8/lesson_urdf/urdf/ur5.urdf',useFixedBase=True, basePosition=[0.0,0.0,0.0], flags=flags)

#shift the robot to 0,0,0
p.resetBasePositionAndOrientation(pandaUid, [0, 0, -0.1], [0, 0, 0, 1])

joint_ids = []
param_ids = []
num_joints = p.getNumJoints(pandaUid)
print('num_joints:', num_joints)

joint_states = p.getJointStates(pandaUid, range(0, num_joints))
joint_poses = [x[0] for x in joint_states]
print('joint_poses:', joint_poses)

#colors for each joint
colors = [np.random.rand(3).tolist() +[1] for _ in range(num_joints)]


#joint positions in degrees
joint_poses = [0, 90, -90, 90, -90, -90, 0]
#convert to radians
joint_poses = [x * np.pi / 180.0 for x in joint_poses]

#set joint positions
for i in range(6):
    p.resetJointState(pandaUid, i, joint_poses[i])


#define a cube
visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.055, 0.055, 0.055])
cubeUid = p.createMultiBody(baseMass=1, baseInertialFramePosition=[0, 0, 0], baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId, basePosition=[0, 0, 0.5], useMaximalCoordinates=True)
#change friction
p.changeDynamics(cubeUid, -1, lateralFriction=0.6)

#change mass
p.changeDynamics(cubeUid, -1, mass=10)

#get the cube to ground
p.resetBasePositionAndOrientation(cubeUid, [0, 0.5, 0.05], [0, 0, 0, 1])

#set gravity
p.setGravity(0, 0, -10)

#set plane
planeUid = p.loadURDF('plane.urdf')


object_trajectory = []
joint_positions = []

for i in range(240):
    p.stepSimulation()
    #get the joint positions
    joint_states = p.getJointStates(pandaUid, range(0, num_joints))
    joint_poses = [x[0] for x in joint_states]
    joint_positions.append(joint_poses)

    #get object position and orientation
    object_pos, object_ori = p.getBasePositionAndOrientation(cubeUid)
    traj_t = [object_pos[0], object_pos[1], object_pos[2], object_ori[0], object_ori[1], object_ori[2], object_ori[3]]
    object_trajectory.append(traj_t)


    time.sleep(1./240.)

object_trajectory = np.array(object_trajectory)
joint_positions = np.array(joint_positions)
#save joint positions and object trajectory
np.save('object_trajectory.npy', object_trajectory)
np.save('joint_trajectory.npy', joint_positions)


