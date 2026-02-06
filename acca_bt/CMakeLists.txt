cmake_minimum_required(VERSION 3.8)
project(acca_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(erp42_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Find SQLite3 for DB reading
find_package(SQLite3 REQUIRED)

include_directories(include)

add_executable(acca_bt_node src/acca_bt_main.cpp)

# Add all ROS2 dependencies using ament_target_dependencies
ament_target_dependencies(acca_bt_node
  rclcpp
  behaviortree_cpp
  std_msgs
  visualization_msgs
  erp42_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link only non-ROS libraries directly
target_link_libraries(acca_bt_node
  SQLite::SQLite3
)

# Install rules
install(TARGETS acca_bt_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/path_follower.py
  scripts/osm_loader.py
  scripts/stanley.py

  scripts/send_goal.py
  scripts/osrm_bridge.py
  scripts/fake_localization_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()