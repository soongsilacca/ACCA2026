cmake_minimum_required(VERSION 3.0.2)
project(multimodal_learning)
find_package(catkin REQUIRED COMPONENTS rospy rosbag sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge message_filters morai_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/extract_bag_dataset.py scripts/train.py scripts/evaluate.py
  scripts/inference_node.py scripts/tcp_inference_node.py
  scripts/simlingo_inference_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch model_artifacts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
