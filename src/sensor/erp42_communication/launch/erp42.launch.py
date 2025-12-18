from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    erp_port_arg = DeclareLaunchArgument("erp_port", default_value=TextSubstitution(text="/dev/ttyUSB1"))

    erp42_serial_node = Node(
        package='erp42_communication',
        executable='erp42_serial',
        name='erp42_serial',
        output='screen',
        parameters=[{
            "erp_port" : LaunchConfiguration('erp_port'),
        }]
    )

    return LaunchDescription([
        erp_port_arg,
        erp42_serial_node
    ])