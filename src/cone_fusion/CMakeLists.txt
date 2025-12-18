cmake_minimum_required(VERSION 3.10)
project(cone_fusion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolo_msg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # ✅ OpenCV 추가
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



include_directories(
  /usr/include/eigen3
)

add_executable(fusion_node src/fusion_node.cpp)
ament_target_dependencies(fusion_node rclcpp geometry_msgs yolo_msg Eigen3 sensor_msgs)

add_executable(time src/time_test.cpp)
ament_target_dependencies(time rclcpp geometry_msgs yolo_msg Eigen3 sensor_msgs)

add_executable(cone src/fusion_conetracking.cpp)
ament_target_dependencies(cone rclcpp geometry_msgs yolo_msg Eigen3 sensor_msgs)

add_executable(traffic src/traffic_fusion.cpp)
ament_target_dependencies(traffic
  rclcpp
  geometry_msgs
  yolo_msg
  Eigen3
  OpenCV
  
)

add_executable(tf src/tf_traffic.cpp)
ament_target_dependencies(tf
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  yolo_msg
  
)

install(TARGETS 
cone
traffic
tf
fusion_node
time
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
