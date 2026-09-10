cmake_minimum_required(VERSION 3.0.2)
project(safety_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/collision_estop_node.py
  scripts/safety_gate_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
