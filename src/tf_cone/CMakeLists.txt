cmake_minimum_required(VERSION 3.5)
project(tf_cone)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cone_in_odom_frame src/cone_in_odom_frame.cpp)
add_executable(cone_in_odom_frame_count src/cone_in_odom_frame_count.cpp)
add_executable(cone_poses_in_odomframe src/cone_poses_in_odomframe.cpp)





ament_target_dependencies(cone_in_odom_frame
  rclcpp 
  std_msgs
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

ament_target_dependencies(cone_in_odom_frame_count
  rclcpp 
  std_msgs
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(cone_poses_in_odomframe
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)



install(TARGETS
  cone_in_odom_frame
  cone_in_odom_frame_count
  cone_poses_in_odomframe
  DESTINATION lib/${PROJECT_NAME})

ament_package()
