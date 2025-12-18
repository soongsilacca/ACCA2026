cmake_minimum_required(VERSION 3.8)
project(hw1_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub_node src/pub_node.cpp)
add_executable(sub_node src/sub_node.cpp)

ament_target_dependencies(pub_node rclcpp std_msgs )
ament_target_dependencies(sub_node rclcpp std_msgs )

if(BUILD_TESTING)
  #find_package(ament_lint_augto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  pub_node
  sub_node
  DESTINATION lib/${PROJECT_NAME}
  )
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
