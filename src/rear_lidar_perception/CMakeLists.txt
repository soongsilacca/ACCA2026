cmake_minimum_required(VERSION 3.0.2)
project(rear_lidar_perception)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)

add_message_files(FILES
  TrackedObject.msg
  TrackedObjectArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime morai_msgs nav_msgs pcl_conversions pcl_ros
                 roscpp sensor_msgs std_msgs visualization_msgs
  DEPENDS PCL
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(rear_lidar_perception_node src/rear_lidar_perception_node.cpp)
add_dependencies(rear_lidar_perception_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rear_lidar_perception_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS rear_lidar_perception_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
