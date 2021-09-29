#########################################
# Basic functions for geometric control #
#########################################

import math
import numpy as np

#############################
# Convert acc to quaternion #
#############################
def acc2quaternion(acc, yaw):
    proj_xb_des = [math.cos(yaw), math.sin(yaw), 0.0]

    zb_des      = acc/np.linalg.norm(acc)
    yb_des      = np.cross(zb_des, proj_xb_des)/np.linalg.norm(np.cross(zb_des, proj_xb_des))
    xb_des      = np.cross(yb_des, zb_des)/np.linalg.norm(np.cross(yb_des, zb_des))

    rotmat      = np.array([[xb_des[0], yb_des[0], zb_des[0]],[xb_des[1], yb_des[1], zb_des[1]],[xb_des[2], yb_des[2], zb_des[2]]])
    quat        = rotationMatrixToQuat(rotmat)
    return quat

#############################
# Convert rotation matrix to 
# quaternion                
#############################
def rotationMatrixToQuat(rotmat):
    quat = [0,0,0,0]
    S    = 0
    tr   = np.trace(rotmat)
    
    if tr > 0.0:
        S       = math.sqrt(tr + 1.0) * 2.0 # S = 4qw

        quat[0] = 0.25*S
        quat[1] = (rotmat[2,1] - rotmat[1,2]) / S
        quat[2] = (rotmat[0,2] - rotmat[2,0]) / S
        quat[3] = (rotmat[1,0] - rotmat[0,1]) / S
    
    if rotmat[0,0] > rotmat[1,1] and rotmat[0,0] > rotmat[2,2]:
        S       =  math.sqrt(1.0 + rotmat[0,0] - rotmat[1,1] - rotmat[2,2]) * 2.0
        quat[0] =  (rotmat[2,1] - rotmat[1,2])/S
        quat[1] =  0.25*S
        quat[2] =  (rotmat[0,1] + rotmat[1,0])/S
        quat[3] =  (rotmat[0,2] + rotmat[2,0])/S
    
    if rotmat[1,1] > rotmat[2,2]:
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

#####################
# Hat operation     #
#####################
def hat(vec):
    return np.array([[0,-vec[2], vec[1]],[vec[2],0.0,-vec[0]],[-vec[1], vec[0], 0]])