cmake_minimum_required(VERSION 3.5)
project(gazebo_grasp_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_grasp_fix SHARED
  src/GazeboGraspFix.cpp
  src/GazeboGraspGripper.cpp
)

target_link_libraries(gazebo_grasp_fix
  ${GAZEBO_LIBRARIES}
)

install(TARGETS gazebo_grasp_fix
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(gazebo_grasp_fix)
ament_package()
