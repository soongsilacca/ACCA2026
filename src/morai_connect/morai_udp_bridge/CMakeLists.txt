cmake_minimum_required(VERSION 3.0.2)
project(morai_udp_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  morai_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_publisher_node.py
  scripts/imu_publisher_node.py
  scripts/camera_front_publisher_node.py
  scripts/camera_left_publisher_node.py
  scripts/camera_right_publisher_node.py
  scripts/collision_data_publisher_node.py
  scripts/ego_vehicle_status_publisher_node.py
  scripts/ego_ground_truth_state_publisher_node.py
  scripts/object_info_publisher_node.py
  scripts/ego_ctrl_sender_node.py
  scripts/scenario_load_sender_node.py
  scripts/scenario_episode_manager_node.py
  scripts/morai_network_autoconnect_node.py
  scripts/morai_simulator_watchdog_node.py
  scripts/cmd_publisher_example.py
  scripts/traffic_light_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
