#########################################
# Basic functions for geometric control #
#########################################

import math
import numpy as np
from mavros_msgs.msg import State 


class controlParams():
    def __init__(self):
        self.mass       = 1.5 # kg
        self.currPose   = [0,0,0]    # x,y,z
        self.currVel    = [0,0,0]    # xd, yd, zd
        self.currOrient = [0,0,0,1]  # qx, qy, qz, qw

        self.targetPose   = [0,0,2]    
        self.targetVel    = [0,0,0]
        self.targetOrient = [0,0,0,1]
        self.targetAcc    = [0,0,0]

        self.kPose        = [11,11,21]
        self.kVel         = [1.5,1.5,3.3]

        self.g_           = np.array([0,0,-9.8]) # body frame z axis is aligned with gravity
        self.maxFbAcc     = 3.0
        self.maxVel       = 5.0

        self.norm_thrust_offset_ = 0.10
        self.norm_thrust_const   = 0.05

        self.bodyRateCmdPublisher = None

        self.e1                   = [1,0,0]
        self.e2                   = [0,1,0]
        self.e3                   = [0,0,1]

        self.attCtrl_tau_        = 0.10



#############################
# Convert acc to quaternion #
#############################
def acc2quaternion(acc, yaw):
    proj_xb_des = np.array([math.cos(yaw), math.sin(yaw), 0.0])

    zb_des      = acc/np.linalg.norm(acc)

    yb_des      = np.cross(zb_des, proj_xb_des)/np.linalg.norm(np.cross(zb_des, proj_xb_des))
    xb_des      = np.cross(yb_des, zb_des)/np.linalg.norm(np.cross(yb_des, zb_des))

    #rotmat      = np.array([ xb_des[0], yb_des[0], zb_des[0] ],[ xb_des[1], yb_des[1], zb_des[1] ],[ xb_des[2], yb_des[2], zb_des[2] ])
    rotmat      = np.array([ [xb_des[0], yb_des[0], zb_des[0]], [xb_des[1], yb_des[1], zb_des[1]], [xb_des[2], yb_des[2], zb_des[2]] ])

    quat        = rotationMatrixToQuat(rotmat)
    return quat

#############################
# Convert rotation matrix to 
# quaternion                
#############################
def rotationMatrixToQuat(rotmat):
    quat = [0,0,0,0]
    S    = 0

    tr   = np.trace(np.array(rotmat))

    if tr > 0.0:
        
        S       = math.sqrt(tr + 1.0) * 2.0 # S = 4qw
        quat[0] = 0.25*S
        quat[1] = (rotmat[2,1] - rotmat[1,2]) / S
        quat[2] = (rotmat[0,2] - rotmat[2,0]) / S
        quat[3] = (rotmat[1,0] - rotmat[0,1]) / S
    
    elif rotmat[0,0] > rotmat[1,1] and rotmat[0,0] > rotmat[2,2]:
        S       =  math.sqrt(1.0 + rotmat[0,0] - rotmat[1,1] - rotmat[2,2]) * 2.0
        quat[0] =  (rotmat[2,1] - rotmat[1,2])/S
        quat[1] =  0.25*S
        quat[2] =  (rotmat[0,1] + rotmat[1,0])/S
        quat[3] =  (rotmat[0,2] + rotmat[2,0])/S
    
    elif rotmat[1,1] > rotmat[2,2]:
        S       = math.sqrt(1.0 + rotmat[1,1] - rotmat[0,0] - rotmat[2,2]) * 2.0
        quat[0] = (rotmat[0,2] - rotmat[2,0])/S
        quat[1] = (rotmat[0,1] + rotmat[1,0])/S
        quat[2] = 0.25*S
        quat[3] = (rotmat[1,2] + rotmat[2,1])/S
    
    else:
        S       = math.sqrt(1.0 + rotmat[2,2] - rotmat[0,0] - rotmat[1,1])*2.0
        quat[0] = (rotmat[1,0] - rotmat[0,1])/S
        quat[1] = (rotmat[0,2] - rotmat[2,0])/S
        quat[2] = (rotmat[1,2] + rotmat[2,1])/S
        quat[3] = 0.25*S
    
    return quat

########################
# Convert quaternion to 
# rotation matrix      
########################
def quatToRotationMatrix(quat):
    r00 = quat[0]**2 + quat[1]**2 - quat[2]**2 - quat[3]**2
    r01 = 2*quat[1]*quat[2] - 2*quat[0]*quat[3]
    r02 = 2*quat[0]*quat[2] + 2*quat[1]*quat[3]

    r10 = 2*quat[0]*quat[3] + 2*quat[1]*quat[2]
    r11 = quat[0]**2 - quat[1]**2 + quat[2]**2 - quat[3]**2
    r12 = 2*quat[2]*quat[3] - 2*quat[0]*quat[1]

    r20 = 2*quat[1]*quat[3] - 2*quat[0]*quat[2]
    r21 = 2*quat[0]*quat[1] + 2*quat[2]*quat[3]
    r22 = quat[0]**2 - quat[1]**2 - quat[2]**2 + quat[3]**2
    
    R =  np.array([[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]])
    return R

def quatToRotationMatrix_new(q):
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]

    R = np.zeros((3,3))
    
    R[0,0] = 1 - 2*qy**2 - 2*qz**2
    R[0,1] = 2*qx*qy - 2*qz*qw
    R[0,2] = 2*qx*qz + 2*qy*qw

    R[1,0] = 2*qx*qy + 2*qz*qw
    R[1,1] = 1-2*qx**2 - 2*qz**2
    R[1,2] = 2*qy*qz - 2*qx*qw

    R[2,0] = 2*qx*qz - 2*qy*qw
    R[2,1] = 2*qy*qz + 2*qx*qw
    R[2,2] = 1-2*qx**2 - 2*qy**2

    return R

#####################
# Hat operation     #
#####################
def hat(vec):
    return np.array([[0,-vec[2], vec[1]],[vec[2],0.0,-vec[0]],[-vec[1], vec[0], 0]])

################################
# Inverse Hat or Vee operation #
################################
def hatInv(mat):
    return np.array([mat[2,1], mat[0,2], mat[1,0]])