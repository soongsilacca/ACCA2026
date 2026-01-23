#!/usr/bin/env python3
"""
Dual EKF Localization Launch File (Nav2 Recommended)

- EKF Local: Fuses wheel odometry + IMU → odom->base_link TF
- NavSat Transform: Converts GPS to odometry
- EKF Global: Fuses wheel + IMU + GPS → map->odom TF (drift correction)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get share directory
    localization_share_dir = get_package_share_directory('localization')

    # Config files
    ekf_local_config = os.path.join(
        localization_share_dir,
        'config',
        'ekf_local.yaml'
    )
    
    ekf_global_config = os.path.join(
        localization_share_dir,
        'config',
        'ekf_global.yaml'
    )
    
    # Read map anchor config for GPS datum
    map_anchor_file = os.path.join(
        localization_share_dir,
        'config',
        'map_anchor.yaml'
    )
    
    with open(map_anchor_file, 'r') as f:
        map_anchor_config = yaml.safe_load(f)
        gps_params = map_anchor_config['gps_map_origin_node']['ros__parameters']
        origin_lat = gps_params['origin_latitude']
        origin_lon = gps_params['origin_longitude']
        origin_alt = gps_params['origin_altitude']
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ============================================
    # Launch Arguments
    # ============================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # ============================================
    # Wheel Odometry Node
    # ============================================
    wheel_odom_node = Node(
        package='localization',
        executable='wheel_odometry_erp',
        name='wheel_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase': 1.04,
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'publish_tf': False,  # EKF will publish
            
            # Filtering
            'lpf_alpha_speed': 0.7,
            'lpf_alpha_steer': 0.7,
            
            # Steering Tuning
            'k_left': 0.76,
            'k_right': 0.9,
        }]
    )
    
    # ============================================
    # EKF Local (odom->base_link)
    # ============================================
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[
            ekf_local_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
            ('set_pose', 'set_pose/local'),
        ]
    )
    
    # ============================================
    # GPS to Odometry Converter (Custom)
    # ============================================
    gps_to_odom_node = Node(
        package='localization',
        executable='gps_to_odometry',
        name='gps_to_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'datum_latitude': origin_lat,
            'datum_longitude': origin_lon,
            'datum_altitude': origin_alt,
        }]
    )
    
    # ============================================
    # EKF Global (map->odom, with GPS)
    # ============================================
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[
            ekf_global_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
            ('set_pose', 'set_pose/global'),
        ]
    )
    
    # ============================================
    # EKF Global Initializer
    # ============================================
    ekf_init_global_node = Node(
        package='localization',
        executable='ekf_global_initializer',
        name='ekf_global_initializer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'datum_latitude': origin_lat,
            'datum_longitude': origin_lon,
            'datum_altitude': origin_alt,
        }]
    )

    ekf_init_local_node = Node(
        package='localization',
        executable='ekf_local_initializer',
        name='ekf_local_initializer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    imu_nwu_node = Node(
        package='localization',
        executable='imu_nwu_adapter',
        name='imu_nwu_adapter_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odom_map_republisher_node = Node(
        package='localization',
        executable='odom_map_republisher',
        name='odom_map_republisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ============================================
    # PCD Map Publisher (Service-triggered)
    # ============================================
    pcd_map_publisher_node = Node(
        package='localization',
        executable='pcd_map_publisher',
        name='pcd_map_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pcd_file': os.path.join(localization_share_dir, 'map', 'school.pcd')
        }]
    )
    
    # ============================================
    # FAST-LIO Odometry Adapter
    # ============================================
    fastlio_adapter_node = Node(
        package='localization',
        executable='fastlio_odometry_adapter',
        name='fastlio_odometry_adapter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_twist': False,  # Don't use twist (velocity) from FAST-LIO
        }]
    )

    # ============================================
    # NDT Localization Node
    # ============================================
    ndt_localization_node = Node(
        package='ndt_localization',
        executable='ndt_localization_node',
        name='ndt_localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ndt_resolution': 1.0,
            'ndt_step_size': 0.1,
            'scan_voxel_size': 0.3,
            'map_voxel_size': 0.5,
        }]
    )
    
    # ============================================
    # Static TF Publishers
    # ============================================
    
    # IMU TF: base_link -> imu_link
    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        output='screen',
        arguments=['--x', '1', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    velodyne_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_tf_publisher',
        output='screen',
        arguments=['--x', '1', '--y', '0', '--z', '1',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # GPS TF: base_link -> gps
    gps_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf_publisher',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0.5',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'gps'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ============================================
    # Launch Description
    # ============================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        
        # Static TFs
        imu_tf_node,
        velodyne_tf_node,
        gps_tf_node,
        
        # IMU Adapter (ENU -> NWU)
        imu_nwu_node,
        
        # Odom Map Aligner (Forces Map-Odom Orientation Alignment)
        odom_map_republisher_node,
        
        # PCD Map Publisher (Service-triggered)
        pcd_map_publisher_node,
        
        # FAST-LIO Odometry Adapter
        fastlio_adapter_node,
        
        # NDT Localization
        ndt_localization_node,
        
        # Wheel Odometry
        wheel_odom_node,
        
        # EKF Local (odom->base_link)
        ekf_local_node,
        
        # GPS Conversion (custom, simple)
        gps_to_odom_node,
        
        # EKF Global Initializer (sets initial pose from GPS)
        ekf_init_global_node,

        # EKF Local Initializer (sets initial pose from Navheading)
        ekf_init_local_node,
        
        # EKF Global (map->odom)
        ekf_global_node,
    ])
