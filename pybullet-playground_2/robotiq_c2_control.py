import os
import time
import pdb
import pybullet as p
import pybullet_data

serverMode = p.GUI # GUI/DIRECT
robotUrdfPath = "./urdf/robotiq_c2.urdf"

# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
#p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")

# define robot
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(robotUrdfPath))
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, 
                     flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

# set all movable joint to static and obtain mimic child joints
numJoints = p.getNumJoints(robotID) 
jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
mimicParentName = "robotiq_85_left_knuckle_joint"
mimicChildName = ["robotiq_85_right_knuckle_joint",
                  "robotiq_85_right_finger_joint",
                  "robotiq_85_left_inner_knuckle_joint",
                  "robotiq_85_left_finger_tip_joint",
                  "robotiq_85_right_inner_knuckle_joint",
                  "robotiq_85_right_finger_tip_joint"]
mimicMul = [-1,-1,-1,-1,-1,-1]
mimicChildList = []
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotID, i)
    jointID = jointInfo[0]
    jointName = jointInfo[1].decode("utf-8")
    jointType = jointTypeList[jointInfo[2]]
    if jointType=="REVOLUTE":
        p.setJointMotorControl2(robotID, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    if jointName in mimicChildName:
        mimicChildList.append(jointID)
    if jointName==mimicParentName:
        mimicParentID = jointID
print("There are {} mimic child".format(len(mimicChildList)))

# contraints to simulate mimic tag in robotiq_c2.urdf
for i, mimicChildID in enumerate(mimicChildList):
    c = p.createConstraint(robotID, mimicParentID, 
                           robotID, mimicChildID,
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0,1,0],
                           parentFramePosition=[0,0,0],
                           childFramePosition=[0,0,0])
    p.changeConstraint(c, gearRatio=mimicMul[i], maxForce=10000)
    constraintInfo = p.getConstraintInfo(c)

# start simulation
try:
    flag = True
    gripper_control = p.addUserDebugParameter("gripper", 0, 0.343, 0)
    while(flag):
        jointPose = p.readUserDebugParameter(gripper_control)
        p.setJointMotorControl2(robotID, mimicParentID, p.POSITION_CONTROL,
                                targetPosition=jointPose)
        p.stepSimulation()
    p.disconnect()
except:
    p.disconnect()

