cmake_minimum_required(VERSION 3.0.2)
project(mgeo_route_publisher)

find_package(catkin REQUIRED COMPONENTS rospy nav_msgs geometry_msgs std_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/teacher_route_editor.py
  scripts/mgeo_route_publisher_node.py
  scripts/snap_global_path_to_mgeo.py
  scripts/close_global_path_loop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
