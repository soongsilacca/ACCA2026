# MORAI Privileged Expert

MORAI에서 전체 주행 임무를 수행하고 학습용 teacher label을 생성하기 위한
rule-based privileged expert 패키지다. MGeo 경로를 기본 주행 경로로 사용하고,
시뮬레이터 ground truth 객체와 신호 상태를 이용해 추종, 감속, 정지, 회피를
결정한다. 선택한 경로와 속도는 기존 MPC에 전달한다.

## 현재 상태

현재 코드는 MORAI에서 실제 주행 가능한 상태다. 다만 CARLA Garage의 실행기를
그대로 구동하는 것은 아니며, PDM-Lite의 시뮬레이터 독립적인 핵심 로직을 MORAI
ROS/UDP 인터페이스에 맞게 포팅한 구현이다.

원본 소스는 다음 위치에 복사되어 있다.

- 저장소: `third_party/carla_garage`
- upstream: `autonomousvision/carla_garage`
- branch: `leaderboard_2`
- commit: `72f39a63423a5edef6904b1487e0360a64bcf445`
- license: MIT

포팅된 핵심 코드는 `scripts/pdm_lite_core.py`, ROS 입출력과 행동 결정은
`scripts/privileged_expert_node.py`에 있다.

## CARLA API와 MORAI 입력 대응

| CARLA PDM-Lite 입력 | MORAI 대체 입력 |
| --- | --- |
| `Waypoint`, route | MGeo `/global_path`, `/local_route` |
| Ego actor pose/velocity | `/localization/kinematic_state` |
| Vehicle actor | `/Object_topic.npc_list` |
| Walker actor | `/Object_topic.pedestrian_list` |
| Static actor | `/Object_topic.obstacle_list` |
| Actor bounding box | ObjectInfo `size`, `heading`, `position` |
| Traffic-light actor | MORAI TrafficCtrl UDP와 `/SetTrafficLight` |
| Planned trajectory | `/privileged_expert/path` |
| Desired speed | `/privileged_expert/target_velocity` |
| Vehicle control | 기존 MPC → `/ctrl_cmd` |

## 구현 완료

### 센서와 통신

- Ego 위치와 속도는 `/localization/kinematic_state` 사용
- MORAI ObjectInfo UDP를 `morai_msgs/ObjectStatusList`로 변환
- NPC, 보행자, 정적 장애물의 ID, 위치, 방향, 속도, 가속도, 크기 사용
- ObjectInfo의 km/h 속도를 내부 계획 단위인 m/s로 변환
- ObjectInfo가 유실되면 기본 설정에서 safe stop
- 센서, localization, MGeo, controller를 하나의 launch로 실행

현재 UDP 포트는 다음과 같다.

| 데이터 | MORAI/Host | ROS PC/Destination |
| --- | ---: | ---: |
| ObjectInfo | 7605 | 7505 |
| TrafficCtrl | 7607 | 7503 |
| EgoVehicleStatus | 9010 | 9011 |

모든 연결은 UDP이며 ROSBridge는 사용하지 않는다.

### PDM-style planning

- MGeo global path resampling
- 횡방향 3개 proposal: `[-1.5, 0.0, 1.5] m`
- 속도 4개 proposal: `[0.0, 0.35, 0.65, 1.0] × MGeo speed`
- 총 12개 trajectory proposal 생성
- 가감속 제한을 적용한 longitudinal rollout
- 시간축 actor 위치 예측
- Ego와 actor의 oriented bounding box SAT 충돌 검사
- collision, clearance, lateral, comfort, progress, speed 점수 계산
- CARLA Garage IDM 적분 로직을 이용한 선행 차량 속도 결정
- 선택 경로와 orientation을 MPC에 전달

### 객체별 정책

- `npc_list`: 미래 위치 예측, IDM FOLLOW, 임박한 충돌 STOP
- `pedestrian_list`: 접근 시 `YIELD_PEDESTRIAN` 감속, 미래 OBB 충돌 시 완전 정지
- ObjectInfo 보행자 속도가 0이면 연속 위치 변화로 이동 속도 추정
- 보행자 예측 폭에 1.2m 불확실성 padding 적용
- 경로와 겹치지 않는 보도 위 사람에게는 정지하지 않음
- `obstacle_list`: 중심 경로를 막으면 좌우 proposal 중 안전 경로 선택
- 정적 장애물 회피 판단 거리 20m, lateral transition 8m
- 적색 신호가 장애물보다 가까우면 MGeo 중심 경로를 유지하며 신호 정지
- 25m 이내의 5m/s 이하 NPC는 안전한 옆 proposal이 있으면 `AVOID_NPC`
- 이미 정적 장애물과 겹친 복구 상황에서는 무한 정지하지 않고 이탈

### 신호등

- traffic manager가 4~9초 간격으로 신호 상태를 임의 변경
- 직진/우회전은 green, 좌회전은 green-left 상태 확인
- 진행 불가 신호에서 `SLOW_SIGNAL` 후 `STOP_SIGNAL`
- 정지선 기준 20m 전방 정지 buffer 적용
- 제동 지연 보상을 위해 목표 지점 5m 전부터 `STOP_SIGNAL` full brake 적용
- 5m 이내에 함께 설치된 차량 신호등은 같은 phase로 모두 UDP 제어
- 지나간 신호 상태를 폐기하고 현재 제어 중인 최신 신호만 판단
- 현재 `EgoVehicleStatus.link_id`를 여러 upstream approach link 매핑에 역조회
- 앞쪽 300m, 횡방향 8m 이내이며 ego 진행 방향과 일치하는 approach만 선택
- MGeo `related_signal`로 straight/left/left_unprotected/right 판정
- protected left는 green-left bit가 없으면 일반 green에서도 정지
- 신호 명령이 1.5초 동안 갱신되지 않으면 신호 정지 판단 자동 해제

### 출력과 시각화

| Topic | Type | 내용 |
| --- | --- | --- |
| `/privileged_expert/path` | `nav_msgs/Path` | MPC 선택 경로 |
| `/privileged_expert/target_velocity` | `std_msgs/Float32` | 목표 속도, m/s |
| `/privileged_expert/behavior` | `std_msgs/String` | 현재 행동 |
| `/privileged_expert/action` | `std_msgs/String` | teacher action |
| `/privileged_expert/mode_scores` | `Float32MultiArray` | STOP/DRIVE gate |
| `/privileged_expert/proposal_scores` | `Float32MultiArray` | proposal 점수 |
| `/privileged_expert/teacher_labels` | `std_msgs/String` | JSON 학습 label |
| `/privileged_expert/markers` | `MarkerArray` | proposal/actor/HUD |
| `/privileged_expert/ready` | `std_msgs/Bool` | 입력 준비 상태 |

RViz에는 후보 및 선택 경로, 객체 bounding box, 객체 예측 방향, ID와 속도,
현재 행동, 위험 객체와 신호 상태를 표시한다.

## 빌드와 실행

```bash
cd ~/acca_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg morai_privileged_expert morai_udp_bridge
source devel/setup.bash
roslaunch morai_privileged_expert expert.launch
```

기본 launch는 센서, localization, MGeo route publisher, ObjectInfo, expert,
traffic manager, MPC와 RViz를 함께 실행한다.

## TCP teacher rosbag 수집

Expert 주행을 먼저 실행한 뒤 별도 터미널에서 TCP 학습용 bag recorder를 실행한다.

```bash
cd ~/acca_ws
source devel/setup.bash
roslaunch morai_privileged_expert record_tcp_training_bag.launch
```

기본 출력은 `~/acca_ws/tcp_teacher_YYYY-MM-DD-HH-MM-SS.bag`이다. 저장 위치를
바꾸려면 기존 디렉터리를 지정한다. 장시간 수집은 기본적으로 4GB 단위로
자동 분할된다.

```bash
roslaunch morai_privileged_expert record_tcp_training_bag.launch \
  bag_prefix:=/data/tcp_teacher_
```

기록 토픽은 TCP front camera, GT ego state, localization state, MGeo route/token/속도, teacher가 선택한
path/속도/DRIVE·STOP·AVOID, ObjectInfo, 신호 상태, MPC 출력과 collision이다.
용량이 큰 raw camera와 Velodyne packet은 기록하지 않는다. 녹화를 끝낼 때는
recorder 터미널에서 `Ctrl-C`를 눌러 bag index가 정상적으로 닫히게 한다.

차량을 움직이지 않고 확인하려면:

```bash
roslaunch morai_privileged_expert expert.launch launch_controller:=false
```

MORAI 신호를 직접 변경하지 않으려면:

```bash
roslaunch morai_privileged_expert expert.launch control_traffic_lights:=false
```

센서 스택이 이미 실행 중이면:

```bash
roslaunch morai_privileged_expert expert.launch launch_sensors:=false
```

## 실행 확인

```bash
rostopic hz /Object_topic
rostopic echo -n1 /Object_topic
rostopic echo /privileged_expert/ready
rostopic echo /privileged_expert/behavior
rostopic echo /privileged_expert/target_velocity
rostopic echo /privileged_expert/teacher_labels
```

정상 주행 조건:

- `/privileged_expert/ready: true`
- `/privileged_expert/path`가 MPC subscriber와 연결됨
- 주행 행동에서 `/privileged_expert/target_velocity > 0`
- 정지 행동에서 target velocity 0, STOP score 1

## 주요 설정

설정 파일은 `config/expert.yaml`이다.

| 설정 | 현재값 | 의미 |
| --- | ---: | --- |
| `planning_horizon_m` | 120.0 | 계획 거리 |
| `prediction_horizon_sec` | 4.0 | actor 미래 예측 시간 |
| `lane_change_length_m` | 8.0 | lateral shift 길이 |
| `avoid_distance_m` | 20.0 | 정적 장애물 회피 개시 거리 |
| `pedestrian_caution_clearance_m` | 3.0 | 보행자 감속 여유 |
| `pedestrian_caution_time_sec` | 3.5 | 보행자 감속 시간 범위 |
| `signal_stop_buffer_m` | 20.0 | 정지선 전 정지 거리 |
| `signal_full_stop_lead_m` | 5.0 | full-stop 조기 진입 거리 |
| `signal_slowdown_distance_m` | 45.0 | 적색 신호 감속 시작 거리 |
| `maximum_speed_mps` | 27.78 | expert 최고속도(100km/h) |
| `lead_vehicle_distance_m` | 100.0 | NPC 선행차 추종 시작 거리 |
| `traffic_state_timeout_sec` | 1.5 | 신호 상태 유효 시간 |
| `require_objects` | true | ObjectInfo 유실 시 safe stop |

## 지금까지 수정한 문제

- 존재하지 않는 개별 `ObjectStatus.header` 접근으로 node가 죽던 문제
- ObjectInfo 타입을 NPC=1, pedestrian=0, obstacle=2로 분류
- ObjectInfo 속도 단위를 km/h에서 m/s로 변환
- 멀리 떨어진 객체가 경로 index 0으로 잡히던 허위 STOP
- 보도 옆 보행자가 원형 clearance만으로 STOP되던 문제
- 정적 장애물을 늦게 발견해 모든 회피 후보가 충돌하던 문제
- 장애물과 이미 겹친 상태에서 영원히 EMERGENCY_STOP 되던 deadlock
- 지나간 신호의 red 상태가 남아 엉뚱한 위치에서 정지하던 문제
- catkin Python relay가 `pdm_lite_core`를 가리던 import 문제

## 남은 작업

우선순위 순서다.

1. MORAI 반복 주행으로 보행자 감속 거리와 제동 profile 튜닝
2. MGeo link polygon을 이용한 drivable-area 및 차선 경계 score 추가
3. 반대 차선 회피 금지 또는 overtaking 조건 추가
4. NPC constant-velocity 예측을 kinematic bicycle forecast로 교체
5. proposal hysteresis를 추가해 좌우 경로 진동 방지
6. traffic light와 controlled link를 직접 연결해 교차로 매칭 강화
7. stop sign, junction priority, parked vehicle, blocked-route recovery 추가
8. PDM progress/comfort score를 upstream 설정값과 정밀 비교
9. rosbag replay와 collision/latency/success 자동 평가 도구 추가
10. teacher label dataset schema 고정 및 학습 파이프라인 연결
11. 장시간 주행 회귀 테스트 추가

## 알려진 한계

- 동적 actor 예측은 현재 constant velocity다.
- lateral proposal의 실제 차선 경계 통과 여부를 아직 검사하지 않는다.
- 정적 장애물 회피 시 대향차선 여부를 아직 판단하지 않는다.
- 신호 제어는 실험용 random phase manager다. 외부 신호를 사용하려면
  `control_traffic_lights:=false`로 실행해야 한다.
- CARLA runtime 전체가 아니라 PDM-Lite planning primitive의 MORAI 포팅본이다.
  upstream CARLA API와 ScenarioRunner는 직접 실행하지 않는다.
