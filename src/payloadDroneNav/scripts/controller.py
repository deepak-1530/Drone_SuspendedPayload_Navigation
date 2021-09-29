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
import tf.transformations as TF
from mavros_msgs.msg import State 
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

currPose     = [] # current drone position
currVel      = [] # current drone velocity
currAtt      = [] # current drone attitude
currAttVel   = [] # current drone angular velocity

kPos         = [8.0, 8.0, 10.0]
kVel         = [1.5, 1.5, 3.3]

attCtrl_tau_ = 0.1

maxAcc       = 3.0
maxVel       = 5.0

# the targets are updated as soon as a path is received
targetPose        = []
targetOrientation = []
targetVel         = []
targetAcc         = []
targetYaw         = 0

norm_thrust_offset_ = 0.1
norm_thrust_const   = 0.05
max_fb_acc          = 9.0
g                   = 10


bodyRateCmdPublisher = None

current_state = State() 
offb_set_mode = SetMode

#########################
# State callback        #
#########################
def state_cb(state):
    global current_state
    current_state = state

########################
# Drone pose callback  #
########################
def mavPoseCallback(msg):
    global currPose, currAtt
    currPose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    currAtt  = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]

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
    targetOrientation = [pt.transforms[0].rotation.w, pt.transforms[0].rotation.y, pt.transforms[0].rotation.z, pt.transforms[0].rotation.z]
    targetVel         = [pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z]
    targetAcc         = [pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z]

    targetAngles      = TF.euler_from_quaternion(targetOrientation)

    targetYaw         = targetAngles[2]


###########################
# Command loop            #
###########################
def cmdLoopCallback(msg):
    global targetPose, targetOrientation, targetVel, targetAcc, targetYaw, bodyRateCmdPublisher
    print('Control loop initiated')

    print(currPose, currAtt, currVel, currAttVel)

    if np.linalg.norm(np.array(targetPose)) is not 0:

        accDesired              = positionController(targetPose, targetVel, targetAcc, targetYaw)
        bodyRateCmd             = AttitudeController(accDesired)

        command                 = AttitudeTarget()
        command.header.stamp    = rospy.Time.now()
        command.header.frame_id = "map"
        command.body_rate.x     = bodyRateCmd[0]
        command.body_rate.y     = bodyRateCmd[1]
        command.body_rate.z     = bodyRateCmd[2]
        command.type_mask       = 128
        command.orientation.w   = targetOrientation[0]
        command.orientation.x   = targetOrientation[1]
        command.orientation.y   = targetOrientation[2]
        command.orientation.z   = targetOrientation[3]
        command.thrust          = bodyRateCmd[3]

        # publish the body rate command
        bodyRateCmdPublisher.publish(command)
        print("Command published")


#########################
# Position Controller   #
#########################
def positionController(targetPos, targetVel, targetAcc, targetYaw):
    global kPos, kVel, g, max_fb_acc
    
    # target acceleration
    aRef = targetAcc
    
    # target rotation in quaternion
    qRef = helper.acc2quaternion(np.array(aRef)-g, targetYaw)
    
    # target rotation matrix
    Rref = helper.quatToRotationMatrix(qRef)

    # now calculate the errors in position and velocity
    posErr = targetPos - currPos
    velErr = targetVel - currVel

    afb    = np.diag(kPos)*posErr + np.diag(kVel)*velErr

    if np.linalg.norm(afb) > max_fb_acc:
        afb = (max_fb_acc / np.linalg.norm(afb))*afb
    
    aDes   = afb + aRef - g

    return aDes

########################
# Attitude Controller  #
########################
def AttitudeController(accDesired):
    global targetYaw, norm_thrust_const, norm_thrust_offset_, attCtrl_tau_

    qDes         = helper.acc2quaternion(accDesired, targetYaw)

    rateCmd      = [0,0,0,0]

    rotmat       = helper.quatToRotationMatrix(currAttitude)
    rotmat_d     = helper.quatToRotationMatrix(qDes)

    zb           = rotmat[:,2]

    errorAtt     = 0.5*helper.hat(rotmat_d.T * rotmat - rotmat.T * rotmat)
    rateCmd[0:3] = (2.0 / attCtrl_tau_)*errorAtt
    rateCmd[3]   = np.max(0.0, np.min(1.0, norm_thrust_const*np.dot(ref_acc, zb) + norm_thrust_offset_))
    return rateCmd

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

    bodyRateCmdPublisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    ################################
    # set the control timer        #
    ################################
    rospy.Timer(rospy.Duration(0.01), cmdLoopCallback) # this checks the status of the drone and if it is not armed or not in offboard mode -> then it arms it and changes it to offboard mode

    ####################################################
    # Change drone state to offboard mode              #
    # Drone has already taken off so no need to arm it #
    ####################################################
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode) 

    p = PoseStamped()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0

    localPosePub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)

    # publish the above point to convert to offboard mode
    for i in range(100):
        localPosePub.publish(p)
    
    offb_set_mode = SetMode()
#    offb_set_mode.request.custom_mode='OFFBOARD'

    set_mode_client(base_mode=0, custom_mode="OFFBOARD")

    rospy.spin()

if __name__=="__main__":
    print("Controller started")
    rospy.init_node("controller")
    initController()