cmake_minimum_required(VERSION 3.0.2)
project(lss_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/boat_tf.py
  scripts/monitoring_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
