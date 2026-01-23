cmake_minimum_required(VERSION 3.8)
project(ndt_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(std_srvs REQUIRED)

# PCL Dependencies
find_package(PCL REQUIRED COMPONENTS common io filters registration search kdtree)

add_executable(ndt_localization_node src/ndt_localization_node.cpp)
ament_target_dependencies(ndt_localization_node
  rclcpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  ndt_omp
)

# Link PCL
target_include_directories(ndt_localization_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(ndt_localization_node ${PCL_LIBRARIES})

# OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  target_link_libraries(ndt_localization_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS
  ndt_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

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
