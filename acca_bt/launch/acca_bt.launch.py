from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# 런치 파일을 실행하면 가장 먼저 호출되는 메인 함수입니다.
def generate_launch_description():
    # 1. 사용할 패키지 이름을 변수로 저장해둡니다. (나중에 경로 찾을 때 씀)
    pkg_name = 'acca_bt'
    
    # 2. 외부에서 터미널로 입력받은 'use_sim_time' 값을 저장할 변수입니다.
    # (예: ros2 launch ... use_sim_time:=true)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # --- [3. 런치 인자(Argument) 선언] ---
    # 터미널에서 사용자가 설정을 바꿀 수 있도록 옵션을 만들어주는 부분입니다.

    # 3-1. 시뮬레이션 시간 사용 여부 (기본값: false / 가상 환경이면 true로 설정해야 함)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 3-2. OSM 지도 파일 경로 설정 (기본값: 작성자님의 학교 지도 경로)
    osm_file_arg = DeclareLaunchArgument(
        'osm_file',
        default_value='/home/won/BT/osm/school4.osm',
        description='Full path to the OSM file to load'
    )

    # 3-3. 사용할 Behavior Tree XML 파일 이름 설정 (기본값: acca_bt_simple.xml)
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value='acca_bt.xml',
        description='Behavior Tree XML file name (in bt_xml folder)'
    )

    # 3-4. 가상 시뮬레이터 모드 (True면 가상 위치 생성)
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable fake localization simulation'
    )
    
    sim_mode = LaunchConfiguration('sim_mode')

    # --- [4. 경로 설정] ---
    # 설치된 패키지 폴더 안의 'bt_xml' 폴더 경로와 파일명을 합쳐서 전체 경로를 만듭니다.
    bt_xml_path = [
        os.path.join(get_package_share_directory(pkg_name), 'bt_xml'),
        '/',
        LaunchConfiguration('bt_xml') # 위에서 설정한 파일명을 가져옴
    ]

    # --- [5. 노드(Node) 실행 설정] ---
    # 실제로 실행할 프로그램(노드)들을 정의합니다.

    # 5-0. 가상 시뮬레이터 (sim_mode:=true일 때만 실행)
    # from launch.conditions import IfCondition
    # fake_sim_node = Node(
    #     package=pkg_name,
    #     executable='fake_localization_sim.py',
    #     name='fake_localization_sim',
    #     output='screen',
    #     condition=IfCondition(sim_mode),
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # 5-1. 지도 로더 노드 (OSM 파일을 읽어서 시각화 및 데이터로 변환)
    osm_loader = Node(
        package=pkg_name,              # 패키지 이름
        executable='osm_loader.py',    # 실행할 파일 이름 (파이썬)
        name='osm_loader',             # 노드 이름
        output='screen',               # 터미널에 로그 출력
        parameters=[{                  # 노드에 전달할 파라미터
            'osm_file': LaunchConfiguration('osm_file'), # 지도 경로 전달
            'use_sim_time': use_sim_time
        }]
    )

    # 5-2. 경로 추종 노드 (Stanley 또는 Pure Pursuit 알고리즘 담당)
    path_follower = Node(
        package=pkg_name,
        executable='path_follower.py',
        name='path_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )



    # 5-4. ★핵심★ Behavior Tree 노드 (C++로 만든 메인 제어기)
    acca_bt_node = Node(
        package=pkg_name,
        executable='acca_bt_node',     # C++ 실행 파일
        name='acca_bt_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'bt_xml_path': bt_xml_path # 완성된 XML 경로를 파라미터로 넘겨줌
        }]
    )

    # 5-5. RViz2 노드 (시각화 도구)
    # 패키지 안에 있는 'acca_bt.rviz' 설정 파일을 찾습니다.
    rviz_config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'acca_bt.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir], # 설정 파일(-d)을 적용해서 실행
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- [6. 최종 실행 목록 반환] ---
    # 위에서 정의한 설정(Argument)과 노드(Node)들을 모두 리스트에 담아 리턴하면
    # ROS 2가 순서대로 실행시킵니다.
    return LaunchDescription([
        use_sim_time_arg, # 설정 1
        osm_file_arg,     # 설정 2
        bt_xml_arg,       # 설정 3
        sim_mode_arg,     # 설정 4
        
        fake_sim_node,    # 시뮬레이터 (조건부)
        osm_loader,       # 노드 1 (지도)
        path_follower,    # 노드 2 (제어)

        acca_bt_node,     # 노드 4 (BT 메인)
        rviz_node         # 노드 5 (시각화)
    ])