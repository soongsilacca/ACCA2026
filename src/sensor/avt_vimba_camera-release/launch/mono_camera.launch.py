from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    name = LaunchConfiguration("name")
    frame_id = LaunchConfiguration("frame_id")
    ip = LaunchConfiguration("ip")
    guid = LaunchConfiguration("guid")
    use_measurement_time = LaunchConfiguration("use_measurement_time")
    ptp_offset = LaunchConfiguration("ptp_offset")
    image_proc = LaunchConfiguration("image_proc")

    camera_info_url = PathJoinSubstitution(
        [
            FindPackageShare("avt_vimba_camera"),
            "calibrations",
            "calibration_example.yaml",
        ]
    )

    params_file = PathJoinSubstitution(
        [FindPackageShare("avt_vimba_camera"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="camera"),
            DeclareLaunchArgument("frame_id", default_value="camera"),
            DeclareLaunchArgument("ip", default_value=""),
            DeclareLaunchArgument("guid", default_value=""),
            DeclareLaunchArgument("use_measurement_time", default_value="false"),
            DeclareLaunchArgument("ptp_offset", default_value="-37"),
            DeclareLaunchArgument("image_proc", default_value="false"),
            Node(
                package="avt_vimba_camera",
                executable="mono_camera_exec",
                name=name,
                output="screen",
                parameters=[
                    {
                        "name": name,
                        "frame_id": frame_id,
                        "ip": ip,
                        "guid": guid,
                        "use_measurement_time": use_measurement_time,
                        "ptp_offset": ptp_offset,
                        "camera_info_url": camera_info_url,
                    },
                    params_file,
                ],
            ),
            GroupAction(
                [
                    Node(
                        package="image_proc",
                        executable="image_proc",
                        name="image_proc",
                        namespace=name,
                        output="screen",
                    )
                ],
                condition=IfCondition(image_proc),
            ),
        ]
    )
