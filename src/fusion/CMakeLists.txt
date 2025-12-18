cmake_minimum_required(VERSION 3.8)
project(fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(cropbox src/cropbox_bs.cpp)
ament_target_dependencies(cropbox rclcpp std_msgs sensor_msgs)

add_executable(cropbox_bum src/cropbox_bum.cpp)
ament_target_dependencies(cropbox_bum rclcpp std_msgs sensor_msgs)

add_executable(cropbox_ys src/cropbox_ys.cpp)
ament_target_dependencies(cropbox_ys rclcpp std_msgs sensor_msgs)

add_executable(image_concat src/image_concat_bs.cpp)
ament_target_dependencies(image_concat rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(fusion_3cam src/fusion_bs_pointstamped_3cam.cpp)
ament_target_dependencies(fusion_3cam rclcpp std_msgs sensor_msgs geometry_msgs darknet_ros_msgs visualization_msgs cv_bridge OpenCV)


#darknet_ros_msgs
install(TARGETS
  cropbox
  cropbox_ys
  cropbox_bum
  image_concat
  fusion_3cam
  DESTINATION lib/${PROJECT_NAME})
ament_package()

