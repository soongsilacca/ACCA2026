cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
)

# dynamic_reconfigure cfg 생성
generate_dynamic_reconfigure_options(
  cfg/MPCParams.cfg
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/stanley_controller_node.py
  scripts/stanley_controller_speed_node.py
  scripts/stanley_global_path_node.py
  scripts/stanley_local_path_node.py
  scripts/pure_pursuit_node.py
  scripts/train_stanley_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
