cmake_minimum_required(VERSION 3.0.2)
project(mapping)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  roscpp
  sensor_msgs
  tf2_msgs
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  cv_bridge
  rosbag
  std_srvs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    rospy
    std_msgs
    roscpp
    sensor_msgs
    tf2_msgs
    tf2_ros
    tf2_eigen
    pcl_ros
    pcl_conversions
    cv_bridge
    rosbag
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(colored_map_creator_node src/colored_map_creator_node.cpp)
target_link_libraries(colored_map_creator_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(colored_map_creator_node ${catkin_EXPORTED_TARGETS})

add_executable(online_colored_map_creator_node src/online_colored_map_creator_node.cpp)
target_link_libraries(online_colored_map_creator_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(online_colored_map_creator_node ${catkin_EXPORTED_TARGETS})
