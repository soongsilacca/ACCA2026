
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import xacro

def generate_launch_description():
    pkg_name = "localization"
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            remappings=[
            # ('imu', 'imu/data'),
            ('imu', 'imu/rotated'),
            # ('imu', 'imu/pfiltered'),
            ('odometry/filtered', '/odometry/navsat'),
            # ('odometry/filtered', '/localization/kinematic_state'),
            ('gps/fix', 'ublox_gps_node/fix'),
        ],
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'navsat_transform.yaml')],
           ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{"robot_description":robot_description.toxml()}],
            output='screen'),

        launch_ros.actions.Node(
            package="localization_cpp",
            executable="gps_dummy",
            name="gps_jamming_filter",
            output = "screen"
        ),
        launch_ros.actions.Node(
            package='localization',
            executable='erp_twist',
            name='erp_twist',
            output='screen',
        ),
            
])
