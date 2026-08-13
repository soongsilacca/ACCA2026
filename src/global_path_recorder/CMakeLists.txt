cmake_minimum_required(VERSION 3.0.2)
project(global_path_recorder)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    rospy
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/global_path_plotter_node.py
  scripts/global_path_recorder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
