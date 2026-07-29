cmake_minimum_required(VERSION 3.0.2)
project(local_path_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  global_path_planner
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs tf global_path_planner
)

# Python Nodes Installation
catkin_install_python(PROGRAMS
  scripts/mgeo_local_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
