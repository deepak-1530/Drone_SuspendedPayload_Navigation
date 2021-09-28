##############################################
## Geometric control of quadrotors in python #
##############################################

# Input  -> trajectory -> position, velocity, acceleration, orientation
# Output -> wx, wy, wz, thrust

# Controller -> position controller and attitude controller

# take feedback from mavros
# trigger the controller at 1khz
# tune the weights accordingly

g = 10.0 # acceleration due to gravity

# error weights
kPos, kVel, kAcc

targetPos, targetVel, targetAcc, targetYaw

# drone state parameters
# currPos, currVel, currAcc, currYaw
# controller output = wx, wy, wz, thrust  /mavros/setpoint_raw/bodyrate


def positionController(targetPos, targetVel, targetAcc, targetYaw):
    
    # target acceleration
    aRef = targetAcc
    
    # target rotation in quaternion
    qRef = acc2quaternion(aRef-g, targetYaw)
    
    # target rotation matrix
    Rref = quatToRotationMatrix(qRef)

    # now calculate the errors in position and velocity
    posErr = targetPos - currPos
    velErr = targetVel - currVel

    afb    = kPos*posErr + kVel*velErr

    if np.linalg.norm(afb) > max_fb_acc:
        afb = (max_fb_acc / np.linalg.norm(afb))*afb
    
    aDes   = afb + aRef - g

    return aDes

def computeBodyRateCmd(cmdBodyRate_, desired_acc):
    qDes = acc2quaternion(aDes, targetYaw)
    bodyRateCmd = geometricAttController(qDes, desired_acc, currAttitude)

    rateCmd  = []
    rotmat   = np.zeros((3,3))
    rotmat_d = np.zeros((3,3))
    errorAtt = []

    rotmat   = quatToRotationMatrix(currAttitude)
    rotmat_d = quatToRotationMatrix(qDes)

    errorAtt = 0.5*hat(rotmat_d.T * rotmat - rotmat.T * rotmat)
    rateCmd[0:3] = (2.0 / attCtrl_tau_)*errorAtt
    rateCmd[3]   = np.max(0.0, np.min(1.0, norm_thrust_const*np.dot(ref_acc, zb) + norm_thrust_offset_))
