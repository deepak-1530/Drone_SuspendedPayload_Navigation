cmake_minimum_required(VERSION 3.0.2)
project(payloadDroneNav)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  std_msgs
  visualization_msgs
)

find_package(dynamicEDT3D)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${DYNAMICEDT3D_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES FastPlannerOctomap
#  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs octomap octomap_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

# add all the files to be compiled
add_executable(Planner src/Planner.cpp src/kinodynamic_astar.cpp)
target_link_libraries(Planner ${catkin_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})



add_executable(Controller src/Controller.cpp)
target_link_libraries(Controller ${catkin_LIBRARIES})

add_executable(pubCamPose src/pubCamPose.cpp)
target_link_libraries(pubCamPose ${catkin_LIBRARIES})


