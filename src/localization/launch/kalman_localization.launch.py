from launch import LaunchDescription
from ament_index_python.packages import *
import launch_ros.actions
import os
import yaml
import pathlib
from launch.substitutions import *
from launch.actions import DeclareLaunchArgument
import xacro
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Directory to your parameters file

    pkg_name = "localization"
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)
    params_file = PathJoinSubstitution(
        [FindPackageShare("localization"), "params", "kalman_localization.yaml"]
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description":robot_description.toxml()}],
        output = 'screen'
    )

    kalman_node = launch_ros.actions.Node(
        package="localization",
        executable="kalman_localization",
        name="kalman_localization_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        robot_state_publisher_node,
        kalman_node,
    ])
    

if __name__ == '__main__':
    generate_launch_description()