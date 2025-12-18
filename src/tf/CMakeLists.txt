cmake_minimum_required(VERSION 3.5)
project(tf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(map_odom_tf_publisher_static src/map_odom_tf_publisher_static.cpp)
add_executable(map_odom_tf_publisher_static_bs src/map_odom_tf_publisher_static_bs.cpp)
add_executable(map_odom_tf_publisher_static_tunnel src/map_odom_tf_publisher_static_tunnel.cpp)
add_executable(odom_baselink_tf_publisher src/odom_baselink_tf_publisher.cpp)
add_executable(odometry_map_frame src/odometry_map_frame.cpp)
add_executable(cone_pose_transform src/cone_pose_transform.cpp)
add_executable(cone_pose_transform_ys src/cone_pose_transform_ys.cpp)
add_executable(localization_path_tf src/localization_path_tf.cpp)
add_executable(odom2map src/odom2map.cpp)
add_executable(map_anchor_recorder src/map_anchor_recorder.cpp)


# 패키지에 소스 추가 후 CMakeLists.txt

ament_target_dependencies(map_anchor_recorder rclcpp sensor_msgs)

ament_target_dependencies(map_odom_tf_publisher_static
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(map_odom_tf_publisher_static_bs
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(odom_baselink_tf_publisher
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(map_odom_tf_publisher_static_tunnel
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  Eigen3
)


ament_target_dependencies(odometry_map_frame
  rclcpp
  tf2 
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(cone_pose_transform
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(cone_pose_transform_ys
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(localization_path_tf
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(odom2map
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
)

target_include_directories(map_odom_tf_publisher_static PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(map_odom_tf_publisher_static ${PROJ_LIBRARIES})

target_include_directories(map_odom_tf_publisher_static_bs PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(map_odom_tf_publisher_static_bs ${PROJ_LIBRARIES})

target_include_directories(map_odom_tf_publisher_static_tunnel PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(map_odom_tf_publisher_static_tunnel ${PROJ_LIBRARIES})

target_include_directories(odom_baselink_tf_publisher PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(odom_baselink_tf_publisher ${PROJ_LIBRARIES})
install(TARGETS
  map_odom_tf_publisher_static
  map_odom_tf_publisher_static_bs
  odom_baselink_tf_publisher
  map_odom_tf_publisher_static_tunnel
  odometry_map_frame
  cone_pose_transform
  cone_pose_transform_ys
  localization_path_tf
  odom2map
  map_anchor_recorder
  DESTINATION lib/${PROJECT_NAME})

ament_package()
