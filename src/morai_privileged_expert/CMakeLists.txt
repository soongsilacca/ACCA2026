cmake_minimum_required(VERSION 3.0.2)
project(morai_privileged_expert)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs geometry_msgs visualization_msgs morai_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/privileged_expert_node.py
  scripts/privileged_traffic_manager.py
  scripts/morai_carla_compat.py
  scripts/upstream_pdm_loader.py
  scripts/morai_upstream_pdm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
