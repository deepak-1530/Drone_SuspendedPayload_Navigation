#########################
# Receive the path from #
# fastplanner           #
#########################

##################################################
# 4th order polynomial                           #
# z is constant  (z=7)                           #
# y is constant  (y=0)                           #
# only x increases                               #
# vx is constant (2 m/s)
# ax is 1.0
# x(t)   = 1.0 + 0.2*t + 0.3*t**2 + 0.4*t**3
# x'(t)  = 0.2 + 0.6*t + 1.2*t**2
# x''(t) = 0.6 + 2.4*t
# move on a straight line with a constant velocity
###################################################

import rospy
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

x = []

def generateWayPoints():
    trajPub = rospy.Publisher('/target_state', MultiDOFJointTrajectory, queue_size=1)
    global x

    t = np.linspace(0,10,1000)

    x = 1.0 + 0.2*t + 0.3*t**2 + 0.4*t**3

    rate = rospy.Rate(10) # publish data at 10 Hz

    i = 0

    while not rospy.is_shutdown():
        print('Publishing target point')

        trajPt = MultiDOFJointTrajectoryPoint()
        
        # set pose
        trajPt.transforms.pose.translation.x = x[i]
        trajPt.transforms.pose.translation.y = 0
        trajPt.transforms.pose.translation.z = 7
        trajPt.transforms.rotation.x    = 0
        trajPt.transforms.rotation.y    = 0
        trajPt.transforms.rotation.z    = 0
        trajPt.transforms.rotation.w    = 1.0

        # set velocities
        trajPt.velocities.linear.x      = 2.0
        trajPt.velocities.linear.y      = 0
        trajPt.velocities.linear.z      = 0

        # set accelerations
        trajPt.accelerations.x           = 0.5
        trajPt.accelerations.y           = 0
        trajPt.accelerations.z           = 0.0

        trajMsg = MultiDOFJointTrajectory()
        trajMsg.header.stamp = rospy.Time.now()
        trajMsg.header.frame = 'map'
        trajMsg.points.append(trajPt)

        trajPub.publish(trajMsg)
        rate.sleep()
        i += 1

if __name__=="__main__":
    print('Trajectory generator started')
    rospy.init_node('trajectoryGenerator')
    generateWayPoints()





    

