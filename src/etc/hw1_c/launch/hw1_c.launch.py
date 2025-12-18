import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

	return LaunchDescription(
	[
		ExecuteProcess(
			cmd=["ros2","run","hw1_c","pub_node"], output = "screen"
		),
		ExecuteProcess(
			cmd=["ros2", "run", "hw1_c", "sub_node"], output = "screen"
		),
	]
	)
