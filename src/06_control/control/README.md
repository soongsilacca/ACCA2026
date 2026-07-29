# control

Stanley Method와 Pure Pursuit 기반 횡방향 제어기 패키지입니다.  
`/cmd` (`CtrlCmd`) 토픽으로 MORAI 시뮬레이터에 조향/속도/제동 명령을 전송합니다.

## 노드 목록

| 노드 | 설명 |
|------|------|
| `stanley_local_path_node.py` | Stanley 컨트롤러 — 지역 경로 추종 + 동적 속도 프로파일 |
| `stanley_global_path_node.py` | Stanley 컨트롤러 — 전역 경로 추종 + 고정 목표 속도 |
| `stanley_controller_node.py` | Stanley 컨트롤러 기본 구현 |
| `stanley_controller_speed_node.py` | 속도 기반 Stanley 컨트롤러 |
| `pure_pursuit_node.py` | Pure Pursuit 컨트롤러 |
| `train_stanley_model.py` | Stanley 파라미터 온라인 학습 스크립트 (PyTorch MLP) |

## Launch 파일

```bash
# 지역 경로 Stanley (권장 — 장애물 회피 연동)
roslaunch control stanley_local_path.launch

# 전역 경로 Stanley
roslaunch control stanley_global_path.launch
```

## 토픽

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/local_path` | 입력 | `Path` | 추종할 지역 경로 |
| `/global_path` | 입력 | `Path` | 추종할 전역 경로 |
| `/localization/kinematic_state` | 입력 | `Odometry` | 자차 위치/헤딩/속도 |
| `/morai/ego_vehicle_status` | 입력 | `EgoVehicleStatus` | 조향각, 실제 속도 |
| `/cmd` | 출력 | `CtrlCmd` | 조향/속도/제동 명령 |

## Stanley 알고리즘

```
δ = ψ_e + arctan(k_e · e_fa / (k_v + v))
```

- `ψ_e`: 경로 헤딩 오차
- `e_fa`: 전방 축 횡방향 오차
- `k_e`: 횡방향 게인
- `k_v`: 속도 댐핑 게인
- `v`: 현재 속도

## 모델 학습

```bash
# Stanley 파라미터 온라인 학습
python3 train_stanley_model.py
```

주행 데이터를 기반으로 MLP 모델이 `k_e`, `k_v` 파라미터를 자동 최적화합니다.
