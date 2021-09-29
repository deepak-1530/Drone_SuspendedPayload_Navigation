#############################################
# Controller node for Quad + Payload system #
# Geometric Controller on SE(3)             #
#############################################

##########################################################
# Subscribers -> Drone Pose (position, yaw)    (30Hz)    #
#             -> Drone Lin-vel and Angular-vel (30Hz)    #
#             -> Drone acceleration            (30Hz)    #
##########################################################

import os
import numpy as np

# helper mathematical functions
import helper

# ros dependencies
import rospy
import tf.Transformations as TF
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

currPose     = [] # current drone position
currVel      = [] # current drone velocity
currAtt      = [] # current drone attitude
currAttVel   = [] # current drone angular velocity

kPos         = []
kVel         = []

attCtrl_tau_ = None

maxAcc       = 3.0
maxVel       = 5.0

# the targets are updated as soon as a path is received
targetPose        = []
targetOrientation = []
targetVel         = []
targetAcc         = []
targetYaw         = 0

bodyRateCmdPublisher = None

########################
# Drone pose callback  #
########################
def mavPoseCallback(msg):
    global currPose, currAtt
    currPose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    currAtt  = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

##########################
# Drone angular velocity #
# callback               #
##########################
def mavVelCallback(msg):
    global currVel, currAttVel
    currVel    = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    currAttVel = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]

##########################
# Goal Update            #
##########################
def targetCallback(msg):
    global targetPose, targetOrientation, targetVel, targetAcc, targetYaw

    targetPt          = MultiDOFJointTrajectoryPoint()
    targetPt          = msg.points[0]

    targetPose        = [pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z]
    targetOrientation = [pt.transforms[0].rotation.x, pt.transforms[0].rotation.y, pt.transforms[0].rotation.z, pt.transforms[0].rotation.w]
    targetVel         = [pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z]
    targetAcc         = [pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z]

    targetAngles      = TF.euler_from_quaternion(targetOrientation)

    targetYaw         = targetAngles[2]



###########################
# Command loop            #
###########################
def cmdLoopCallback(msg):
    global targetPose, targetOrientation, targetVel, targetAcc, targetYaw, bodyRateCmdPublisher
    print(f'Control loop initiated')

    accDesired         = PositionController(targetPosition, targetVek, targetAcc, targetYaw)
    bodyRateCmd        = AttitudeController(accDesired)

    # publish the body rate command
    bodyRateCmdPublisher.publish(bodyRateCmd)


#########################
# Position Controller   #
#########################
def positionController(targetPos, targetVel, targetAcc, targetYaw):
    global kPos, kVel, g, max_fb_acc
    
    # target acceleration
    aRef = targetAcc
    
    # target rotation in quaternion
    qRef = helper.acc2quaternion(aRef-g, targetYaw)
    
    # target rotation matrix
    Rref = helper.quatToRotationMatrix(qRef)

    # now calculate the errors in position and velocity
    posErr = targetPos - currPos
    velErr = targetVel - currVel

    afb    = kPos*posErr + kVel*velErr

    if np.linalg.norm(afb) > max_fb_acc:
        afb = (max_fb_acc / np.linalg.norm(afb))*afb
    
    aDes   = afb + aRef - g

    return aDes

########################
# Attitude Controller  #
########################
def AttitudeController(accDesired):
    global targetYaw

    qDes = helper.acc2quaternion(accDesired, targetYaw)

    rateCmd  = []
    rotmat   = np.zeros((3,3))
    rotmat_d = np.zeros((3,3))
    errorAtt = []

    rotmat   = helper.quatToRotationMatrix(currAttitude)
    rotmat_d = helper.quatToRotationMatrix(qDes)

    errorAtt     = 0.5*hat(rotmat_d.T * rotmat - rotmat.T * rotmat)
    rateCmd[0:3] = (2.0 / attCtrl_tau_)*errorAtt
    rateCmd[3]   = np.max(0.0, np.min(1.0, norm_thrust_const*np.dot(ref_acc, zb) + norm_thrust_offset_))


########################
# Start the controller #
########################
def initController():

    global targetPose, targetVel, targetAcc, currPose, currVel, currAtt
    global kPos, kVel
    global maxAcc, maxVel
    global attCtrl_tau_
    global bodyRateCmdPublisher

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavPoseCallback,tcp_nodelay=True)            # position + orientation callback
    rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, mavVelCallback,tcp_nodelay=True)  # linear + angular velocity callback
    rospy.Subscriber('/target_state',MultiDOFJointTrajectory, targetCallback,tcp_nodelay=True)                # goal position (with orientation) + linear velocity + acceleration

    bodyRateCmdPublisher = rospy.Publisher('/command/bodyRate', AttitudeTarget, queue_size=1)

    ################################
    # set the control timer        #
    ################################
    rospy.Timer(rospy.Duration(0.01), cmdLoopCallback) # this checks the status of the drone and if it is not armed or not in offboard mode -> then it arms it and changes it to offboard mode

    rospy.spin()