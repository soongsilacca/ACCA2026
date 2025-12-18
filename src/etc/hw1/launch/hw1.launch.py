import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
def generate_launch_description():

    pub_node = Node(
        package='hw1',
        executable='publish_node',
        parameters=[],
        arguments=[],
        output='screen',
    )
    
    sub_node = Node(
        package='hw1',
        executable='subscribe_node',
        parameters=[],
        arguments=[],
        output='screen',
    )

    return LaunchDescription(
        [
            pub_node,
            sub_node
        ]
    )

generate_launch_description()