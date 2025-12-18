from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
import lifecycle_msgs.msg


def generate_launch_description():
    ld = launch.LaunchDescription()

    lidar_tf = launch_ros.actions.Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "velodyne"],
    )

    imu_tf = launch_ros.actions.Node(
        name="imu_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu_link"],
    )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        "localization_param_dir",
        default=os.path.join(
            get_package_share_directory("pcl_localization_ros2"),
            "param",
            "localization.yaml",
        ),
    )

    pcl_localization = launch_ros.actions.LifecycleNode(
        name="pcl_localization",
        namespace="",
        package="pcl_localization_ros2",
        executable="pcl_localization_node",
        parameters=[localization_param_dir],
        remappings=[
            ("/cloud", "/velodyne_points"),
            # ("/odom", "/odometry/kalman"),
            # ("/imu", "/imu/data"),
            # ("/velodyne_points", "/velodyne_points2"),
            # ('pcl_pose', 'localization/kinematic_state'),
        ],
        output="screen",
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state="unconfigured",
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            pcl_localization
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            pcl_localization
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    kalman_localization = launch_ros.actions.Node(
        package="localization",
        executable="kalman_localization",
        name="kalman_localization_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("localization"),
                "params",
                "kalman_localization.yaml",
            )
        ],
    )

    tf_localization = launch_ros.actions.Node(
        package="localization",
        executable="map_odom_tf_publisher",
        name="map_odom_tf_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("localization"),
                "params",
                "map_odom_tf_publisher.yaml",
            )
        ],
    )

    map_server = launch_ros.actions.Node(
        package="localization",
        executable="real_time_map_server",
        name="real_time_map_server_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("localization"),
                "params",
                "real_time_map_server.yaml",
            )
        ],
    )


    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)

    # PCL Localization
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    ld.add_action(pcl_localization)
    ld.add_action(to_inactive)

    ld.add_action(kalman_localization)
    ld.add_action(tf_localization)

    return ld
