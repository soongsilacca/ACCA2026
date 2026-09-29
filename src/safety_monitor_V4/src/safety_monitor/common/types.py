"""ROS와 무관한 데이터 형식.

좌표계
- base_link: 자차 뒤차축 중심, x 전방, y 좌측
- map: MORAI/MGeo 공통 좌표
- Frenet: 목표 차선 중심선 기준 (s 진행 방향, d 좌측 양수)

Python 3.8(Ubuntu 20.04) 호환을 위해 typing 모듈 표기를 사용한다.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class EgoState:
    stamp: float
    x: float
    y: float
    yaw: float          # rad, map 기준
    speed: float        # m/s, 진행 방향 속도
    valid: bool = True


@dataclass
class TrackObs:
    """safety_monitor/TrackedObject 한 개 (base_link 기준)."""
    track_id: int
    x: float            # 박스 중심 (보이는 점들의 축정렬 박스)
    y: float
    dim_x: float        # base_link x축 방향 길이
    dim_y: float        # base_link y축 방향 길이
    dim_z: float
    vx: float           # 지면 기준 속도를 base_link 축으로 표현
    vy: float
    is_dynamic: bool
    compensated: bool
    coasting: bool
    time_since_update: float = 0.0


@dataclass
class Target:
    """목표 차선 Frenet으로 옮긴 대상 차량."""
    track_id: int
    s_front: float      # 대상 앞 끝 (자차 쪽 끝)
    d: float            # 대상 횡위치 (박스 중심)
    width: float
    length: float
    v_s: float          # 목표 차선 방향 속도
    front_x_base: float  # base_link 기준 앞 끝 x (ROI 잘림 판단용)
    measured: bool      # 이번 스캔에서 실제 관측되었는지 (coasting 아님)
    in_target_lane: bool = True


@dataclass
class TargetJudgment:
    track_id: int
    collision: bool
    gap: float
    v_rel: float
    t_enter: Optional[float]
    window: Optional[Tuple[float, float]]
    t_c: Optional[float]


@dataclass
class MonitorResult:
    stamp: float
    raw: bool                   # 이번 프레임 판정
    estop: bool                 # 발행/해제 규칙 적용 후
    reason: str
    lane_change_active: bool = False
    watched: List[int] = field(default_factory=list)
    judgments: List[TargetJudgment] = field(default_factory=list)
    holds: List[int] = field(default_factory=list)
