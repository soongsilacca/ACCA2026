cmake_minimum_required(VERSION 3.8)
project(pointcloud_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(pointcloud_visualizer_node src/pointcloud_visualizer_node.cpp)

ament_target_dependencies(pointcloud_visualizer_node rclcpp sensor_msgs pcl_conversions PCL)

# Manually link PCL libraries
target_link_libraries(pointcloud_visualizer_node ${PCL_LIBRARIES})

install(TARGETS
  pointcloud_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
