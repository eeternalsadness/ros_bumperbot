cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/simple_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)