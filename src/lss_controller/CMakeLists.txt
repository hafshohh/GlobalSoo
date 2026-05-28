cmake_minimum_required(VERSION 3.0.2)
project(lss_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/differential_control.py
  scripts/manual_control.py
  scripts/XYtes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
