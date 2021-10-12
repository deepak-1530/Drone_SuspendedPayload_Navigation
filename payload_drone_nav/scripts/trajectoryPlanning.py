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
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

x = []

def generateWayPoints():
    trajPub = rospy.Publisher('/target_state', MultiDOFJointTrajectory, queue_size=1)
    global x

    t = np.linspace(0,2,10000)

    x = 1.0 + 0.2*t + 0.3*t**2 + 0.4*t**3
    v = 0.2 + 0.6*t + 1.2*t**2
    a = 0.6 + 2.4*t

    rate = rospy.Rate(10) # publish data at 10 Hz

    i = 0

    while not rospy.is_shutdown():
        print('Publishing target point')

        trajPt = MultiDOFJointTrajectoryPoint()
        
        # set pose
        pose = Transform()
        pose.translation.x = x[i]
        pose.translation.y = 0
        pose.translation.z = 2
        pose.rotation.x    = 0
        pose.rotation.y    = 0
        pose.rotation.z    = 0
        pose.rotation.w    = 1.0


        # set velocity
        vel = Twist()
        vel.linear.x       = v[i]
        vel.linear.y       = 0
        vel.linear.z       = 0

        # set accelerations
        acc = Twist()
        acc.linear.x       = a[i]
        acc.linear.y       = 0
        acc.linear.z       = 0

        trajPt.transforms.append(pose)
        trajPt.velocities.append(vel)
        trajPt.accelerations.append(acc)

        trajMsg = MultiDOFJointTrajectory()
        trajMsg.header.stamp = rospy.Time.now()
        trajMsg.header.frame_id = 'map'
        trajMsg.points.append(trajPt)

        trajPub.publish(trajMsg)
        rate.sleep()
        i += 1

if __name__=="__main__":
    print('Trajectory generator started')
    rospy.init_node('trajectoryGenerator')
    generateWayPoints()





    

