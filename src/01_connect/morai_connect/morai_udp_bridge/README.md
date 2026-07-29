# morai_udp_bridge

MORAI 시뮬레이터와 UDP 소켓으로 통신하는 ROS 노드 패키지입니다.  
`lib/` 디렉토리의 ctypes 구조체를 통해 MORAI 고유 바이너리 프로토콜을 파싱/직렬화합니다.

## 노드 목록

### 수신 (MORAI → ROS)

| 노드 | 수신 포트 | 발행 토픽 |
|------|-----------|-----------|
| `gps_publisher_node.py` | 7777 | `/gps` |
| `imu_publisher_node.py` | 7778 | `/imu` |
| `ego_vehicle_status_publisher_node.py` | 7779 | `/morai/ego_vehicle_status` |
| `camera_front_publisher_node.py` | 7785 | `/camera/front/image_raw` |
| `camera_left_publisher_node.py` | 7786 | `/camera/left/image_raw` |
| `camera_right_publisher_node.py` | 7787 | `/camera/right/image_raw` |
| `collision_data_publisher_node.py` | — | `/morai/collision` |

### 송신 (ROS → MORAI)

| 노드 | 송신 포트 | 구독 토픽 |
|------|-----------|-----------|
| `ego_ctrl_sender_node.py` | 7600 | `/cmd` (`CtrlCmd`) |
| `traffic_light_controller_node.py` | 7607 | `/morai/traffic_light_ctrl` |
| `scenario_load_sender_node.py` | 9095 | `/morai/scenario_load` |

### 유틸리티

| 노드 | 설명 |
|------|------|
| `domain_randomizer.py` | 시나리오 JSON 도메인 랜덤화 (장애물/보행자/NPC 위치 무작위화) |

## Launch 파일

```bash
# 전체 센서 브릿지
roslaunch morai_udp_bridge morai_bridge.launch

# 센서만 (Ego Status 제외)
roslaunch morai_udp_bridge sensors.launch

# 특정 시나리오 로드
roslaunch morai_udp_bridge load_scenario.launch

# 시나리오 랜덤화 후 로드 (domain_randomizer → scenario_load_sender 순차 실행)
roslaunch morai_udp_bridge randomize_and_load.launch

# 신호등 컨트롤러 GUI
roslaunch morai_udp_bridge traffic_light_controller.launch
```

## Domain Randomization

`domain_randomizer.py`는 기준 시나리오 JSON을 읽어 다음 요소를 구역 규칙 기반으로 랜덤화합니다:

| 구역 | 대상 | 랜덤화 항목 |
|------|------|-------------|
| 정적 장애물 구역 | 정적 오브젝트 1개 | 종류(DataID), 위치, 방향 |
| 동적 장애물 구역 | 보행자 1명 | 출발/도착 보도 방향, Y 위치, 속도 |
| 회전교차로 구역 | NPC 차량 | 차종(DataID) |
| 고속도로 구역 | NPC 차량 | 차종, 속도(40~90 km/h), 차선 편향 |
| 음영 구역 | 정적 장애물 | 종류, 위치 |

```bash
# 독립 실행
python3 domain_randomizer.py --input base_scene.json --output randomized.json

# ROS 노드로 실행
rosrun morai_udp_bridge domain_randomizer.py
```

## UDP 프로토콜 참고

모든 패킷은 `lib/define/` 의 ctypes 구조체로 정의됩니다.  
패킷 구조: `header(14B) | data_length(4B) | aux_data(12B) | payload | tail(\r\n)`
