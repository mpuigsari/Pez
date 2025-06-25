cmake_minimum_required(VERSION 3.0.2)
project(pez_comms)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

## This macro will invoke setup.py for you
catkin_python_setup()

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs
)

## Let catkin know where to find headers (none for pure-Python)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install launch files and config files to the share directory
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## (The Python modules and console_scripts entrypoints
##  are handled by catkin_python_setup / setup.py)
