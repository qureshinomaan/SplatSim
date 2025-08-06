import os
import time
import pdb
import pybullet as p
import pybullet_data
import utils
from collections import deque
import numpy as np

serverMode = p.GUI # GUI/DIRECT
sisbotUrdfPath = "./urdf/sisbot.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
#p.setGravity(0,0,-10) # NOTE
planeID = p.loadURDF("plane.urdf")
shelfID = p.loadSDF("kiva_shelf/model.sdf", globalScaling=0.7)

# setup sisbot
robotStartPos = [0,-0.5,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(sisbotUrdfPath))
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn, 
                     flags=p.URDF_USE_INERTIA_FROM_FILE)
joints, controlRobotiqC2, controlJoints, mimicParentName = utils.setup_sisbot(p, robotID)
eefID = 7 # ee_link

# start simulation
ABSE = lambda a,b: abs(a-b)
yPrev = 0
shoulderPanJoint = joints["shoulder_pan_joint"]
shoulderPoseDq = deque(np.arange(shoulderPanJoint.lowerLimit, shoulderPanJoint.upperLimit, 0.1))
try:
    flag = True
    xin = p.addUserDebugParameter("x", -0.42, 0.42, 0)
    yin = p.addUserDebugParameter("y", -0.42, 0.42, yPrev)
    zin = p.addUserDebugParameter("z", 0.4, 1, 1)
    orn = p.getQuaternionFromEuler([0,0,1.5708])
    while(flag):
        x = p.readUserDebugParameter(xin)
        y = p.readUserDebugParameter(yin)
        z = p.readUserDebugParameter(zin)
        # help IK
        if abs(y-yPrev)>0.01:
            pose = shoulderPoseDq[0]
            print(pose)
            p.setJointMotorControl2(robotID, shoulderPanJoint.id, p.POSITION_CONTROL,
                                    targetPosition=pose, force=shoulderPanJoint.maxForce,
                                    maxVelocity=shoulderPanJoint.maxVelocity)
            shoulderPoseDq.rotate(1)
        yPrev = y
        # apply IK
        jointPose = p.calculateInverseKinematics(robotID, eefID, [x,y,z], orn)
        for i, name in enumerate(controlJoints):
            joint = joints[name]
            pose = jointPose[i]
            if name==mimicParentName:
                controlRobotiqC2(controlMode=p.POSITION_CONTROL, targetPosition=pose)
            else:
                p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)
        rXYZ = p.getLinkState(robotID, eefID)[0] # real XYZ
        print("x_err= {:.2f}, y_err= {:.2f}, z_err= {:.2f}".format(*list(map(ABSE,[x,y,z],rXYZ))))
        p.stepSimulation()
    p.disconnect()
except KeyError:
    p.disconnect()
