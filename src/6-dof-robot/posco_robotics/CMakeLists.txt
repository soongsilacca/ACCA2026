cmake_minimum_required(VERSION 3.8)
project(posco_robotics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  meshes
  rviz
  models
  DESTINATION share/${PROJECT_NAME}
)



install(PROGRAMS
  scripts/pick_and_place.py
  scripts/collision_monitor.py
  scripts/mimic_joint_plugin.py
  scripts/gripper_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
