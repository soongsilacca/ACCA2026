cmake_minimum_required(VERSION 3.0.2)
project(utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_publisher_node src/pcd_publisher_node.cpp)
target_link_libraries(pcd_publisher_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/launcher_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

