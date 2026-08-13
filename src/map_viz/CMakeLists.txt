cmake_minimum_required(VERSION 3.0.2)
project(map_viz)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mgeo_marker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
