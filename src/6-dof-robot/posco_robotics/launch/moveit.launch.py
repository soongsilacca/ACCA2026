import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    pkg_posco_robotics = get_package_share_directory('posco_robotics')
    
    # Paths
    xacro_file = os.path.join(pkg_posco_robotics, 'urdf', 'indy7_with_gripper.urdf.xacro')
    srdf_file = os.path.join(pkg_posco_robotics, 'config', 'indy7.srdf')
    kinematics_file = os.path.join(pkg_posco_robotics, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(pkg_posco_robotics, 'config', 'joint_limits.yaml')
    moveit_controllers_file = os.path.join(pkg_posco_robotics, 'config', 'moveit_controllers.yaml')
    ompl_planning_file = os.path.join(pkg_posco_robotics, 'config', 'ompl_planning.yaml')
    ros2_controllers_file = os.path.join(pkg_posco_robotics, 'config', 'ros2_controllers.yaml')
    
    # Process xacro
    robot_desc = Command(['xacro ', xacro_file])
    
    # Gripper Action Server (for MoveIt integration)
    gripper_action_server = Node(
        package='posco_robotics',
        executable='gripper_action_server.py',
        name='gripper_action_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # RVizobot description parameter
    robot_description = {'robot_description': robot_desc}
    
    # SRDF parameter
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Kinematics parameter - load as YAML
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = kinematics_yaml
    
    # Load OMPL planning configuration
    with open(ompl_planning_file, 'r') as f:
        ompl_planning_yaml = yaml.safe_load(f)
    
    # Planning pipeline parameters
    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning_yaml
    }
    
    # Load MoveIt controllers configuration - load as YAML dictionary
    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers_yaml = yaml.safe_load(f)
    
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Gazebo environment setup
    # Gazebo environment setup
    install_dir = get_package_share_directory('posco_robotics').split('/share')[0]
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        model_path = install_dir + "/share"

    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    gazebo_offline = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    # Gazebo launch
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
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Spawn Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'indy7', '-topic', 'robot_description'],
        output='screen'
    )

    # Spawn Ground Plane
    spawn_ground = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ground_plane',
            '-file', os.path.join(pkg_posco_robotics, 'models', 'ground_plane', 'model.sdf'),
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Arm Controller  
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Load joint limits
    with open(joint_limits_file, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)
    
    # MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {'use_sim_time': True},
            {'publish_robot_description_semantic': True},
        ],
    )

    # MoveIt RViz
    rviz_config = os.path.join(pkg_posco_robotics, 'rviz', 'moveit.rviz')
    
    # Planning configuration
    robot_description_kinematics = PathJoinSubstitution([FindPackageShare('posco_robotics'), 'config', 'kinematics.yaml'])
    ompl_planning_config = PathJoinSubstitution([FindPackageShare('posco_robotics'), 'config', 'ompl_planning.yaml'])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--params-file', robot_description_kinematics],
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines_config,
            {'use_sim_time': True},
        ],
    )

    # Delay MoveGroup and RViz to ensure controllers are loaded
    delayed_move_group = TimerAction(
        period=5.0,
        actions=[move_group_node]
    )
    
    delayed_rviz = TimerAction(
        period=6.0,
        actions=[rviz]
    )

    return LaunchDescription([
        gazebo_env,
        gazebo_offline,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_ground,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mimic_joint_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='posco_robotics',
            executable='mimic_joint_plugin.py',
            name='mimic_joint_plugin',
            output='screen',
        ),
        gripper_action_server,
        delayed_move_group,
        delayed_rviz,
    ])

