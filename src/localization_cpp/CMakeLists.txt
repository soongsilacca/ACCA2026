cmake_minimum_required(VERSION 3.8)
project(localization_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(rotate_yaw src/rotate_yaw.cpp)
ament_target_dependencies(rotate_yaw rclcpp std_msgs sensor_msgs nav_msgs)

add_executable(gps_dummy src/gps_dummy.cpp)
ament_target_dependencies(gps_dummy rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

add_executable(gps src/gps.cpp)
ament_target_dependencies(gps rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

install(TARGETS
  rotate_yaw
  gps_dummy
  gps
  DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
