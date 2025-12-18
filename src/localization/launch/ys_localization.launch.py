from launch import LaunchDescription
from ament_index_python.packages import *
import launch_ros.actions
import os
import yaml
import pathlib
from launch.substitutions import *
from launch.actions import DeclareLaunchArgument
import xacro


def generate_launch_description():
    pkg_name = "localization"
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf_ys.xacro")
    robot_description = xacro.process_file(xacro_file)
    navsat_transform_yaml = os.path.join(pkg_path, "params", "navsat_transform.yaml")
    param_ekf_yaml = os.path.join(pkg_path, "params", "param_ekf_ys.yaml")
    # param_ekf_yaml = os.path.join(pkg_path, "params", "param_ekf_high_ndt.yaml")
    # param_ekf_yaml = os.path.join(pkg_path, "params", "param_ekf_with_ndt.yaml")
    rviz_path = os.path.join(pkg_path, "urdf", "rviz.rviz")

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{"robot_description":robot_description.toxml()}],
    #     output = 'screen'
    # )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
    {
        "robot_description": robot_description.toxml()
    }
],
        output="screen",
        name="robot_state_publisher_ys"
    )

    # ys_odom_tf_node = launch_ros.actions.Node(
    #     package="tf",
    #     executable="odom_baselink_tf_publisher",
    #     output="screen",
    #     name="odom_baselink_tf_publisher"
    # )
    

    return LaunchDescription(
        [
            launch_ros.actions.SetParameter(name="use_sim_time", value=False),
            robot_state_publisher_node,
            # ys_odom_tf_node,
            # wheel_odometry_node,
            # rviz_node
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
