# 01_connect — MORAI 시뮬레이터 연동

MORAI 시뮬레이터와 ROS 간의 **UDP 통신 브릿지** 레이어입니다.  
센서 데이터 수신, 차량 제어 명령 송신, 시나리오 로드, 신호등 제어를 담당합니다.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `morai_connect/morai_udp_bridge` | MORAI ↔ ROS UDP 브릿지 노드 모음 |
| `morai_msgs` | MORAI 전용 ROS 메시지 정의 |

## 빠른 시작

```bash
# 전체 브릿지 (GPS, IMU, Ego Status, Ctrl Cmd)
roslaunch morai_udp_bridge morai_bridge.launch

# 시나리오 랜덤화 후 자동 로드
roslaunch morai_udp_bridge randomize_and_load.launch
```
