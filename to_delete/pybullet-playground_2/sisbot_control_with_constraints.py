import os
import time
import pdb
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict

serverMode = p.GUI # GUI/DIRECT
robotUrdfPath = "./urdf/sisbot.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0,0,-10) # NOTE
planeID = p.loadURDF("plane.urdf")

#######################################
###    define and setup robot       ###
#######################################
controlJoints = ["shoulder_pan_joint","shoulder_lift_joint",
                 "elbow_joint", "wrist_1_joint",
                 "wrist_2_joint", "wrist_3_joint",
                 "robotiq_85_left_knuckle_joint"]
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(robotUrdfPath))
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, 
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotID)
jointInfo = namedtuple("jointInfo", 
                       ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity","controllable"])
joints = AttrDict()
for i in range(numJoints):
    info = p.getJointInfo(robotID, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = jointTypeList[info[2]]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    controllable = True if jointName in controlJoints else False
    info = jointInfo(jointID,jointName,jointType,jointLowerLimit,
                     jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
    if info.type=="REVOLUTE": # set revolute joint to static
        p.setJointMotorControl2(robotID, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    joints[info.name] = info

###############################################
## set up mimic joints in robotiq_c2 gripper ##
###############################################
mimicParentName = "robotiq_85_left_knuckle_joint"
mimicChildName = ["robotiq_85_right_knuckle_joint",
                  "robotiq_85_right_finger_joint",
                  "robotiq_85_left_inner_knuckle_joint",
                  "robotiq_85_left_finger_tip_joint",
                  "robotiq_85_right_inner_knuckle_joint",
                  "robotiq_85_right_finger_tip_joint"]
mimicMul = [-1,-1,-1,-1,-1,-1]
mimicChildList = []
parent = joints[mimicParentName]
constraints = dict()
for i, name in enumerate(mimicChildName):
    child = joints[name]
    c = p.createConstraint(robotID, parent.id,
                           robotID, child.id,
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0,0,1],
                           parentFramePosition=[0,0,0],
                           childFramePosition=[0,0,0])
    p.changeConstraint(c, gearRatio=mimicMul[i], maxForce=child.maxForce)
    constraints[name] = c

# start simulation
try:
    flag = True
    userParams = dict()
    for name in controlJoints:
        joint = joints[name]
        userParam = p.addUserDebugParameter(name, joint.lowerLimit, joint.upperLimit, 0)
        userParams[name] = userParam
    while(flag):
        for name in controlJoints:
            joint = joints[name]
            pose = p.readUserDebugParameter(userParams[name])
            p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                    targetPosition=pose, force=joint.maxForce, 
                                    maxVelocity=joint.maxVelocity)
        p.stepSimulation()
    p.disconnect()
except:
    p.disconnect()

