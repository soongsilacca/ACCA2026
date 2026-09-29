cmake_minimum_required(VERSION 3.0.2)
project(safety_monitor)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)

# src/safety_monitor (판정 로직) 를 Python 패키지로 설치.
# generate_messages 보다 먼저 호출해야 한다.
catkin_python_setup()

add_message_files(FILES
  TrackedObject.msg
  TrackedObjectArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime morai_msgs nav_msgs pcl_conversions pcl_ros
                 roscpp rospy sensor_msgs std_msgs visualization_msgs
  DEPENDS PCL
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 6.1.1 후방 LiDAR 인지 (C++)
# CMake target names are workspace-global.  Keep the installed ROS executable
# name while avoiding a collision with the rear_lidar_perception package.
add_executable(safety_monitor_rear_lidar_perception_node
  src/perception/rear_lidar_perception_node.cpp)
set_target_properties(safety_monitor_rear_lidar_perception_node PROPERTIES
  OUTPUT_NAME rear_lidar_perception_node)
add_dependencies(safety_monitor_rear_lidar_perception_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(safety_monitor_rear_lidar_perception_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS safety_monitor_rear_lidar_perception_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 6.1.2, 6.2, E-STOP (Python)
catkin_install_python(PROGRAMS
  nodes/rear_collision_node.py
  nodes/sensor_health_node.py
  nodes/estop_gate_node.py
  scripts/bag_to_npz.py
  scripts/replay_sweep.py
  scripts/analyze_tracking_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
