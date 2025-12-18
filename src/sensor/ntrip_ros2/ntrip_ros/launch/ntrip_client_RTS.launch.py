from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntrip_ros',
            executable='socket_ntrip',
            parameters=[
                {'rtcm_topic': '/rtcm'},
                {'target_host': 'RTS2.ngii.go.kr'},
                {'target_port': '2101'},
                {'mountpoint': 'VRS-RTCM32'},
                {'useragent': 'NTRIP u-blox'},
                # Sometimes connection can rejected because of useragent
                # Then, change it to below one.
                {'useragent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36'},
                {'user': 'tge1375@naver.com:ngii'},
                {'verbose': True},
            ],
            output='screen',
            emulate_tty=True
        )
    ])
