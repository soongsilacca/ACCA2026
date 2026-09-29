"""충돌 판정 파라미터. config/collision.yaml과 이름을 맞춘다."""
from dataclasses import dataclass, fields


@dataclass
class CollisionParams:
    # 자차 치수 (base_link = 뒤차축 중심, rear_lidar.yaml 주석 기준)
    ego_front: float = 3.845          # base_link -> 앞 범퍼
    ego_rear: float = 0.79            # base_link -> 뒤 범퍼
    ego_width: float = 1.89

    # 대상 차량 치수 가정 (후방 LiDAR는 앞면만 보므로 전장은 가정값)
    obs_length_default: float = 4.6
    obs_width_min: float = 1.6
    obs_width_max: float = 2.2

    # 차선
    lane_half_width: float = 1.75     # 차선 변경 판정: local route가 중심선에서 이 값 이상 벗어나면 차선을 넘은 것으로 봄
    lane_match_max_dist: float = 3.0
    lane_match_max_heading_deg: float = 45.0
    chain_back: float = 80.0          # 목표 차선 중심선을 뒤로 이어 붙일 길이
    chain_fwd: float = 200.0
    stop_on_map_failure: bool = False  # 자차/목표 차선 매칭 실패 시 정지할지

    # 차선 변경 종료: 목표 차선 기준 횡오프셋과 heading이 모두 수렴
    done_lateral: float = 0.3
    done_heading_deg: float = 3.0

    # ② 횡방향
    lateral_margin: float = 0.15
    route_step: float = 0.2           # local route 재표본화 간격

    # ③ 종방향
    long_margin: float = 0.5
    v_rel_eps: float = 0.05

    # ④ 결합
    horizon: float = 3.0

    # 예외 첫째: 정지 중 재출발 가정 속도
    v_stop: float = 0.3
    v_restart: float = 4.0

    # ① 감시 대상 진입/제외 (상대속도 기준, 이력 적용)
    v_enter: float = 0.5
    v_exit: float = -0.5

    # 예외 둘째: 통과 차량 정지 유지
    hold_trigger_gap: float = 2.0         # 끊긴 시점의 종방향 간격(자차 뒤 끝 - 대상 앞 끝)이 이 값 이하일 때 적용
    hold_extra: float = 0.68              # t_hold, 해제에 필요한 시간(5프레임)과 동일
    hold_max: float = 5.0                 # 유지 상한 (재관측 불가 대비)
    hold_velocity_window: int = 5         # 속도 스냅샷: 최근 N개 관측 프레임 중앙값
    hold_velocity_max_front_x: float = 1.2  # ROI(x<=1.5)에 박스가 잘리기 전 프레임만 사용

    # 발행/해제 (프레임 단위)
    n_on: int = 2
    n_off: int = 5

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in d.items() if k in names}
        unknown = sorted(k for k in d if k not in names)
        p = cls(**kwargs)
        p.validate()
        return p, unknown

    def validate(self):
        if self.v_exit > self.v_enter:
            raise ValueError("v_exit must be <= v_enter")
        if self.obs_width_min > self.obs_width_max:
            raise ValueError("obs_width_min must be <= obs_width_max")
        if self.horizon <= 0 or self.route_step <= 0:
            raise ValueError("horizon and route_step must be positive")

    @property
    def ego_length(self):
        return self.ego_front + self.ego_rear
