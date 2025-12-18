cmake_minimum_required(VERSION 3.10)
project(yolo_msg)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 정의
set(msg_files
  "msg/BoundingBox.msg"
  "msg/BoundingBoxArray.msg"
  "msg/TrafficSign.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)  # ✅ 추가
ament_package()
