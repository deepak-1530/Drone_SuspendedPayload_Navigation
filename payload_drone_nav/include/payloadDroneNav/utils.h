/** ROS HEADERS **/
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Int8.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/PositionTarget.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/CommandTOL.h>

/** PATH **/
#include<nav_msgs/Path.h>

/** Octomap **/
#include<octomap_msgs/conversions.h>
#include<octomap_ros/conversions.h>
#include<octomap/octomap.h>
#include<dynamicEDT3D/dynamicEDTOctomap.h>

/** Map visualization **/
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include<Eigen/Dense>

#include<iostream>
#include<vector>
#include<math.h>
#include<iterator>
#include<cmath>
