from launch import LaunchDescription
from launch.actions import *
from ament_index_python.packages import *
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml
import pathlib


def generate_launch_description():
    pkg_name_ntrip = "ntrip_ros"
    pkg_path_ntrip = get_package_share_directory(pkg_name_ntrip)

    pkg_name_xsens = "xsens_mti_ros2_driver"
    pkg_path_xsens = get_package_share_directory(pkg_name_xsens)

    pkg_name_ublox = "ublox_gps"
    pkg_path_ublox = get_package_share_directory(pkg_name_ublox)

    pkg_name_erp = "erp42_communication"
    pkg_path_erp = get_package_share_directory(pkg_name_erp)

    pkg_name_lidar = "velodyne"
    pkg_path_lidar = get_package_share_directory(pkg_name_lidar)

    pkg_name_camera = "usb_cam"
    pkg_path_camera = get_package_share_directory(pkg_name_camera)


    erp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path_erp, 'launch', 'erp42.launch.py'))
    )

    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path_ntrip, 'launch', 'ntrip_client_RTS.launch.py'))
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path_ublox, 'launch', 'ublox_gps_node-launch.py'))
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path_xsens, 'launch', 'xsens_mti_node.launch.py'))
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path_lidar, 'launch', 'velodyne-all-nodes-VLP32C-launch.py'))
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path_camera, 'launch', 'camera.launch.py'))
    )

    
    return LaunchDescription([
        erp_launch,
        # ntrip_launch,
        gps_launch,
        imu_launch,
        lidar_launch,
        camera_launch,
    ])

if __name__ == '__main__':
    generate_launch_description()



