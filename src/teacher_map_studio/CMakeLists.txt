cmake_minimum_required(VERSION 3.0.2)
project(teacher_map_studio)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/map_zone_editor.py
  scripts/planner_dispatcher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
