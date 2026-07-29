cmake_minimum_required(VERSION 3.0.2)
project(global_path_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  PlannerWaypoint.msg
  PlannerTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/global_path_planner_node.py
  scripts/waypoint_global_path_planner_node.py
  scripts/static_path_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
