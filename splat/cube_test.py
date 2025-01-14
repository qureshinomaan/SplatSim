import pybullet as p
import numpy as np
import pybullet_data
import time
import urdf_models.models_data as md  


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES



#define a cube
visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
cubeUid = p.createMultiBody(baseMass=1, baseInertialFramePosition=[0, 0, 0], baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId, basePosition=[0, 0, 0.5], useMaximalCoordinates=True)

#get the cube to ground
p.resetBasePositionAndOrientation(cubeUid, [0, 0.0, 0.1], [0, 0, 0, 1])

#set gravity
# p.setGravity(0, 0, -10)

#set plane
# planeUid = p.loadURDF('plane.urdf')


object_trajectory = []
joint_positions = []

while True:
    p.stepSimulation()
    time.sleep(1./240.)

object_trajectory = np.array(object_trajectory)
joint_positions = np.array(joint_positions)
#save joint positions and object trajectory
# np.save('object_trajectory.npy', object_trajectory)
# np.save('joint_positions.npy', joint_positions)


