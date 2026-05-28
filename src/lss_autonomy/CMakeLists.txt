cmake_minimum_required(VERSION 3.0.2)
project(lss_autonomy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_control.py
  scripts/waypoint_loader.py
  scripts/obstacle_avoidance.py
  scripts/path_planner_dstar_apf.py
  scripts/usv_simulator.py
  scripts/usv_simulator_4dof.py
  scripts/path_smoother.py
  scripts/waypoint_visualizer.py
  scripts/dstar_lite.py
  scripts/safe_apf.py
  scripts/pid_controller.py
  scripts/ilos_follower.py
  scripts/sim_odom_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
