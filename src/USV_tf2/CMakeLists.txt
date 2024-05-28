cmake_minimum_required(VERSION 3.0.2)
project(USV_tf2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES USV_tf2
#  CATKIN_DEPENDS rospy sensor_msgs std_msgs tf2_ros
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
install(PROGRAMS
  src/dynamic_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
