import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_posco_robotics = get_package_share_directory('posco_robotics')
    xacro_file = os.path.join(pkg_posco_robotics, 'urdf', 'indy7_with_gripper.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    # Gazebo launch
    # Add the share directory to the GAZEBO_MODEL_PATH
    install_dir = get_package_share_directory('posco_robotics').split('/share')[0]
    indy_description_path = get_package_share_directory('indy_description').split('/share')[0]
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + indy_description_path + '/share'
    else:
        model_path =  install_dir + "/share" + ':' + indy_description_path + '/share'

    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    gazebo_offline = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        ]),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'indy7', '-topic', 'robot_description'],
        output='screen'
    )

    # RViz2
    rviz_config = os.path.join(pkg_posco_robotics, 'rviz', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Position Controller (to hold joints)
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen'
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_env,
        gazebo_offline,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
        gripper_controller_spawner,
        rviz,
    ])
