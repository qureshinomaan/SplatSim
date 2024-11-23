import os
import time
import pdb
import pybullet as p
import pybullet_data
import utils

serverMode = p.GUI # GUI/DIRECT
sisbotUrdfPath = "./urdf/sisbot.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
#p.setGravity(0,0,-10) # NOTE
planeID = p.loadURDF("plane.urdf")

# setup sisbot
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(sisbotUrdfPath))
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn, 
                     flags=p.URDF_USE_INERTIA_FROM_FILE)
joints, controlRobotiqC2, controlJoints, mimicParentName = utils.setup_sisbot(p, robotID)
eefID = 7 # ee_link

# start simulation
ABSE = lambda a,b: abs(a-b)
try:
    flag = True
    xin = p.addUserDebugParameter("x", -2, 2, 0)
    yin = p.addUserDebugParameter("y", -2, 2, 0)
    zin = p.addUserDebugParameter("z", 0.5, 2, 1)
    while(flag):
        x = p.readUserDebugParameter(xin)
        y = p.readUserDebugParameter(yin)
        z = p.readUserDebugParameter(zin)
        jointPose = p.calculateInverseKinematics(robotID, eefID, [x,y,z])
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
except:
    p.disconnect()
