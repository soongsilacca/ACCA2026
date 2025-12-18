from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Directory to your parameters file
    params_file = PathJoinSubstitution(
        [FindPackageShare("localization"), "params", "kalman_localization.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="localization",
                executable="kalman_localization",
                name="kalman_localization_node",
                output="screen",
                parameters=[params_file],
            ),
            
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher_baselink_to_velodyne",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "velodyne"],
                output="screen",
            ),
            
        ]
    )
