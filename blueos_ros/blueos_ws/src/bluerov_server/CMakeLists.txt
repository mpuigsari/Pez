cmake_minimum_required(VERSION 3.0.2)
project(bluerov_server)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

## Declare this catkin package
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install the Python script(s) into the proper location
catkin_install_python(PROGRAMS
  scripts/server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
