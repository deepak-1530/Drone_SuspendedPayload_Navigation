# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # Estimtate state of the payload using camera and drone state # # # # #
# # # #            Use EKF -> Extended Kalman Filter                # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

##############################################################################
# Drone state estimator                                                      #
# state update equation based on previous state and control input            #
# IMU on the drone tells the state of the drone                              #
# Drone local pose known -> T265 camera (200Hz)                              #
# Predict the state of the load using the acceleration measured on the drone #
##############################################################################

## EKF equations are as follows -:
## X(t+1)_pred = F*X(t) + B*u(t) // here u(t) is the acceleration in the Inertial frame of reference
## P(t+1)_pred= Jf*P*Jf_transpose + Q // here Q is the process noise covariance matrix
## Z(t+1)_pred = H(x_pred)

class payloadEKF():
    def __init__(self):
        self.massDrone = 1.5                       # in kiloGrams
        self.massLoad  = 0.1                       # in kiloGrams

        # control signal
        self.currentAccelI       = [0,0,0]         # current acceleration in inertial frame
        self.currentAccelB       = [0,0,0]         # current acceleration in body frame
        self.currentOrientationI = [0,0,0]         # current orientation of the drone in inertial frame of reference
        self.numControl          = len(currentAccelI)

        # state vector
        self.p                   = [0,0,0]         # unit position vector from drone to payload in inertial frame
        self.pDot                = [0,0,0]         # unit velocity vector from drone to payload in inertial frame
        self.stateVector         = p + pDot        # state vector comprising of the unit position and velocity vectors[p, pdot]
        self.numStates           = len(stateVector)

        # measurement
        self.payloadPositionB    = [0,0,0]         # payload position in body frame
        self.payloadVelocityB    = [0,0,0]         # payload velocity in body frame
        self.measurements        = payloadPositionB + payloadVelocityB
        self.numMeasures         = len(measurements)
        
        # EKF matrices
        self.stateUpdatedMat     = np.zeros((numStates,   numStates))   # 6x6
        self.controlMat          = np.zeros((numStates,   numControl))  # 6x3
        self.measureMat          = np.zeros((numMeasures, numStates))   # 6x6
        self.innovationMat       = np.zeros((numStates,   numStates))   # 6x6
        self.kalmanGainMat       = np.zeros((numStates,   numStates))   # 6x6
        self.stateUpdateJMat     = np.zeros((numStates,   numStates))   # 6x6
        self.measureJMat         = np.zeros((numMeasures, numMeasures)) # 6x6
        self.statePredict        = self.p + self.pDot

        # noise matrices
        self.processNoise        = np.zeros((numStates, numStates))     # 6x6
        self.measureNoise        = np.zeros((numMeasures, numMeasures)) # 6x6

        self.payLoadPose         = [0,0,0,0,0,0]                        # payload position and velocity calculation using the above predicted/updated state and the drone pose measured using mavros
        self.dronePose           = [0,0,0,0,0,0]                        # drone position and velocity (50 Hz)

        self.initialized         = False

import rospy
import loadPose                       # tells the camera pose estimation 
import numpy as np
from geometry_msgs.msg import PoseStamped


# Workflow -> set callbacks             -> visual pose estimation and IMU accelerometer and gyroscope values
#          -> on receiving IMU data     -> keep updating the state prediction matrix
#          -> on receiving camera image -> update the state matrix with the measured values

loadPoseEKF    = payloadEKF()
frameCount     = 0                # initialize the EKF with the pose of the payload estimated from the first camera frame received -> if frameCount = 1 -> initializeEKF()


# initialize the EKF matrices and perform state estimation