cmake_minimum_required(VERSION 3.0.2)
project(payloadState)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES payloadState
#  CATKIN_DEPENDS cv_bridge pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)


link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(/usr/local/include)
link_directories(/usr/local/include/lib)


add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} aruco)
