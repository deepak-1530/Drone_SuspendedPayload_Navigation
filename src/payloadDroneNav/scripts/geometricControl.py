################################################
# Geometric control for Quadrotors in python   #
################################################


##################################################################################################
# Control of quadrotor is done using body rate commands -> wx, wy, wz, thrust (mass normalized)  #
##################################################################################################


# Pipeline -> fetch target position, orientation, velocity, acceleration

# Inputs to the controller are -> x,y,z, qx, qy, qz, qw [position, orientation (quaternion)]
# Drone moving with a fixed velocity -> 2m/s
# Drone acceleration desired is  0
# Drone yaw is constant      = 0

# errors -> ePos, eVel, 

# b1  = Re1, b2 = Re2, b3 = Re3


import os
import numpy as np

# helper mathematical functions
import helper
from helper import controlParams as cP

# ros dependencies
import tf
import rospy
from mavros_msgs.msg import State 
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import mavros_msgs

#################### CALLBACKS ###################

cP = cP()

current_state = State()

#####################
# State callback    #
#####################
def state_cb(state):
    global current_state
    current_state = state
    print('Current mode is: ')
    print(current_state.mode)

    if current_state.mode != 'OFFBOARD':
        p = PoseStamped()
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0
        p.pose.orientation.w = 1.0

        localPosePub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)

        # publish the above point to convert to offboard mode
        for i in range(100):
            localPosePub.publish(p)

        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='OFFBOARD')


#####################
# Position callback #
#####################
def mavPoseCallback(msg):
    cP.currPose   = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    cP.currOrient = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

#####################
# Velocity callback #
#####################
def mavVelCallback(msg):
    cP.currVel    = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

#####################
# target callback   #
#####################
def targetCallback(msg):
    pt               = msg.points[0]
    cP.targetPose    = [pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z]
    cP.targetOrient  = [pt.transforms[0].rotation.x, pt.transforms[0].rotation.y, pt.transforms[0].rotation.z, pt.transforms[0].rotation.w]
    cP.targetVel     = [pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z]
    cP.targetVel     = [0,0,0]
    cP.targetAcc     = [0,0,0]
##################################################

#####################
# Position Control  #
#####################
def positionController():
    errPos              = np.array(cP.currPose) - np.array(cP.targetPose)
    errVel              = np.array(cP.currVel)  - np.array(cP.targetVel)

    #print(f"position and velocity errors are: {errPos, errVel}")

    bodyFrameThrust     = np.dot(np.diag(cP.kPose), errPos) + np.dot(np.diag(cP.kVel), errVel) + cP.g_ - np.array(cP.targetAcc)
    
    #print(f'body frame thrust shape is: {bodyFrameThrust.shape}')

    b3d                 = -bodyFrameThrust/np.linalg.norm(bodyFrameThrust)

    Rcurr               = helper.quatToRotationMatrix_new(cP.currOrient) # rotation from inertial to body frame

    inertialFrameThrust = bodyFrameThrust*np.dot(Rcurr.T, np.array(cP.e3))
    
    print("***********************")
    print(inertialFrameThrust.shape)
    print("************************")
    
    massNormThrust      = inertialFrameThrust
    return massNormThrust, b3d

######################
# Attitude Control   #
######################
def attitudeController(thrust, b3d):
    rateCmd = [0,0,0,0]
    #Rtar   =  helper.quatToRotationMatrix_new(cP.targetOrient)
    
    Rtar    =  tf.transformations.quaternion_matrix(cP.targetOrient)
    Rcurr   =  tf.transformations.quaternion_matrix(cP.currOrient)    

    #Rcurr  =  helper.quatToRotationMatrix_new(cP.currOrient)

    b1d         = Rtar[0:3,0]
    b2d         = np.cross(b3d, b1d)/np.linalg.norm(np.cross(b3d, b1d))
    proj_b1d    = np.cross(b2d, b3d)/np.linalg.norm(np.cross(b2d, b3d))

    # projected rotation matrix
    Rd          = np.hstack([np.expand_dims(proj_b1d,axis=1), np.expand_dims(b2d,axis=1), np.expand_dims(b3d,axis=1)])

    errorAtt    = 0.5*helper.hatInv(Rd[0:3,0:3].T * Rcurr[0:3,0:3] - Rcurr[0:3,0:3].T * Rd[0:3,0:3])

    print('Attitude Error is: ' + str(np.linalg.norm(errorAtt)))

    rateCmd[0] = (2.0 / cP.attCtrl_tau_)*errorAtt[0]
    rateCmd[1] = (2.0 / cP.attCtrl_tau_)*errorAtt[1]
    rateCmd[2] = (2.0 / cP.attCtrl_tau_)*errorAtt[2]

    print(np.dot(thrust, b3d))
    #print(f'b3d is: {b3d}')
    #print(f'thrust is: {thrust}')

    rateCmd[3] = max(0.0, min(1.0, cP.norm_thrust_const*np.dot(thrust, -b3d) + cP.norm_thrust_offset_))

    return rateCmd

#####################
# main control loop #
#####################
def cmdLoopCallback(msg):
    # fetch all the control parameters like current state and target state of the drone
    
    acc, b3d                = positionController() # gives the thrust value and the desired orientation of the third body frame axis
    bodyRateCmd             = attitudeController(acc, b3d)
    command                 = AttitudeTarget()
    command.header.stamp    = rospy.Time.now()
    command.header.frame_id = "map"
    command.body_rate.x     = bodyRateCmd[0]
    command.body_rate.y     = bodyRateCmd[1]
    command.body_rate.z     = bodyRateCmd[2]
    command.type_mask       = 128
    command.orientation.w   = 1.0#targetOrientation[0]
    command.orientation.x   = 0.0#targetOrientation[1]
    command.orientation.y   = 0.0#targetOrientation[2]
    command.orientation.z   = 0.0#targetOrientation[3]
    command.thrust          = bodyRateCmd[3]

        # publish the body rate command
    cP.bodyRateCmdPublisher.publish(command)
    print(command)


def startController():
    ### Subscribe to all the topics
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavPoseCallback, tcp_nodelay=True)
    rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, mavVelCallback, tcp_nodelay=True)
    rospy.Subscriber('/target_state',MultiDOFJointTrajectory, targetCallback,tcp_nodelay=True)                  # goal position + orientation + velocity +  acceleration
    rospy.Subscriber('/mavros/state', State, state_cb)

    cP.bodyRateCmdPublisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    
    rospy.Timer(rospy.Duration(0.03), cmdLoopCallback)  # main control loop called at 30Hz
    
    print("Controller started")
    rospy.spin()


if __name__=="__main__":
    rospy.init_node("geometric_controller")
    print("Controller node initialized")
    startController()