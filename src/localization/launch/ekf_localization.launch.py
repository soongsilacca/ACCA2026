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
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)
    navsat_transform_yaml = os.path.join(pkg_path, "params", "navsat_transform.yaml")
    param_ekf_yaml = os.path.join(pkg_path, "params", "param_ekf.yaml")

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description.toxml()}],
    #     output="screen",
    # )

    ekf_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[param_ekf_yaml],
        remappings=[("odometry/filtered", "/odometry/navsat")],
        # remappings=[('odometry/filtered', '/localization/kinematic_state')],
        # clear_parameters=True
    )

    # navsat_transform_node = launch_ros.actions.Node(
    #     package="robot_localization",
    #     executable="navsat_transform_node",
    #     name="navsat_transform_node",
    #     parameters=[navsat_transform_yaml],
    #     remappings=[
    #         # ('imu', 'imu/data'),
    #         ("imu", "imu/rotated"),
    #         ("odometry/filtered", "/odometry/navsat"),
    #         # ('odometry/filtered', '/localization/kinematic_state'),
    #         ("gps/fix", "ublox_gps_node/fix"),
    #     ],
    #     output="screen",
    #     # clear_parameters=True
    # )

    # erp_twist_node = launch_ros.actions.Node(
    #     package="localization", executable="erp_twist", name="erp_twist"
    # )

    return LaunchDescription(
        [
            launch_ros.actions.SetParameter(name="use_sim_time", value=False),
            # robot_state_publisher_node,
            # erp_twist_node,
            # wheel_odometry_node,
            # navsat_transform_node,
            ekf_localization_node,
            # rviz_node
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
