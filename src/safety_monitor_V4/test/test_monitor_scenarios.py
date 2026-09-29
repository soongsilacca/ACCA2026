"""직선 2차선 도로에서 좌측 차선 변경 시나리오.

차선 0: y = 0, 차선 1(목표): y = 3.5. 모두 +x 방향. 각 차선은 link 두 개로 나뉘어
node로 연결되어 있어 chain 이어 붙이기도 함께 확인한다.
"""
import numpy as np
import pytest

from safety_monitor.collision.monitor import CollisionMonitor
from safety_monitor.common.params import CollisionParams
from safety_monitor.common.types import EgoState, TrackObs
from safety_monitor.lanes.mgeo_lanes import LaneMap, Link


def make_map():
    links = []
    for lane, y in ((0, 0.0), (1, 3.5)):
        xs1 = np.linspace(-300, 0, 7)
        xs2 = np.linspace(0, 300, 7)
        links.append(Link("L%da" % lane, np.column_stack([xs1, np.full_like(xs1, y)]),
                          "n%d0" % lane, "n%d1" % lane))
        links.append(Link("L%db" % lane, np.column_stack([xs2, np.full_like(xs2, y)]),
                          "n%d1" % lane, "n%d2" % lane))
    return LaneMap(links)


LANEMAP = make_map()


def lane_change_route(x0=0.0, offset=3.5):
    x = np.linspace(x0, x0 + 60.0, 121)
    u = np.clip((x - x0 - 5.0) / 30.0, 0, 1)
    y = offset * (3 * u ** 2 - 2 * u ** 3)
    return np.column_stack([x, y])


def straight_route(x0=0.0, y=0.0):
    x = np.linspace(x0, x0 + 60.0, 121)
    return np.column_stack([x, np.full_like(x, y)])


def car(tid, x, y, vx, coasting=False, dim_x=0.4, dim_y=1.8):
    """base_link 기준 후방 차량 (앞면만 보이는 박스)."""
    return TrackObs(tid, x, y, dim_x, dim_y, 1.5, vx, 0.0,
                    True, True, coasting)


def car_at_map(tid, ego, mx, my, mvx, coasting=False):
    """map 좌표로 지정한 차량을 ego 기준 base_link TrackObs로 변환."""
    c, s = np.cos(ego.yaw), np.sin(ego.yaw)
    dx, dy = mx - ego.x, my - ego.y
    xb, yb = c * dx + s * dy, -s * dx + c * dy
    vxb, vyb = c * mvx, -s * mvx
    return TrackObs(tid, xb - 0.2, yb, 0.4, 1.8, 1.5, vxb, vyb, True, True, coasting)


def new_monitor(**kw):
    p = CollisionParams(**kw)
    return CollisionMonitor(p, LANEMAP)


EGO = EgoState(0.0, 0.0, 0.0, 0.0, 10.0)


def run(mon, frames, ego=EGO, route=None, dt=0.137):
    route = lane_change_route() if route is None else route
    out = []
    for i, objs in enumerate(frames):
        out.append(mon.step(i * dt, ego, route, objs))
    return out


def test_fast_vehicle_in_target_lane_triggers_after_two_frames():
    mon = new_monitor()
    res = run(mon, [[car(1, -20.0, 3.5, 20.0)]] * 3)
    assert [r.raw for r in res] == [True, True, True]
    assert [r.estop for r in res] == [False, True, True]
    j = res[0].judgments[0]
    assert j.t_enter is not None and j.window is not None
    assert j.t_c <= 3.0


def test_slower_vehicle_not_watched():
    mon = new_monitor()
    res = run(mon, [[car(1, -20.0, 3.5, 8.0)]] * 3)
    assert not any(r.raw for r in res)
    assert res[-1].watched == []


def test_vehicle_in_own_lane_ignored():
    mon = new_monitor()
    res = run(mon, [[car(1, -10.0, 0.0, 20.0)]] * 3)
    assert not any(r.raw for r in res)


def test_no_lane_change_no_judgment():
    mon = new_monitor()
    res = run(mon, [[car(1, -10.0, 3.5, 25.0)]] * 3, route=straight_route())
    assert not any(r.raw for r in res)
    assert not res[-1].lane_change_active


def test_vehicle_far_behind_outside_horizon():
    mon = new_monitor()
    # 상대속도 2 m/s, 30 m 뒤 -> 3 s 안에 겹치지 않음
    res = run(mon, [[car(1, -30.0, 3.5, 12.0)]] * 3)
    assert not any(r.raw for r in res)
    assert res[-1].watched == [1]


def test_stopped_ego_uses_restart_speed():
    """차선 변경 도중 정지. 현재 속도(0)를 쓰면 진입 시각이 정의되지 않아 판정이 풀린다."""
    route = lane_change_route()
    i = 24                                            # x = 12 m 지점, 아직 횡방향으로 겹치기 전
    yaw = float(np.arctan2(route[i + 1, 1] - route[i, 1], route[i + 1, 0] - route[i, 0]))
    ego = EgoState(0.0, route[i, 0], route[i, 1], yaw, 0.0)
    obs = car_at_map(1, ego, 0.0, 3.5, 6.0)           # 목표 차선, 약 12 m 뒤, 6 m/s
    res = run(new_monitor(), [[obs]] * 2, ego=ego, route=route[i:])
    j = res[0].judgments[0]
    assert j.t_enter is not None and j.t_enter < 1.0
    assert res[1].estop

    # 재출발 가정 속도를 쓰지 않으면(= v_restart 0) 판정이 풀리는 것을 확인
    res0 = run(new_monitor(v_restart=0.0), [[obs]] * 2, ego=ego, route=route[i:])
    assert res0[0].judgments[0].t_enter is None and not res0[1].estop


def test_release_after_five_clear_frames():
    mon = new_monitor()
    frames = [[car(1, -20.0, 3.5, 20.0)]] * 2 + [[]] * 5
    res = run(mon, frames)
    assert [r.estop for r in res] == [False, True, True, True, True, True, False]


def test_pass_hold_keeps_stop_after_tracking_lost():
    mon = new_monitor()
    frames = [
        [car(1, -3.0, 3.5, 12.0)],
        [car(1, -1.6, 3.5, 12.0)],
        [car(1, -1.3, 3.5, 12.0, coasting=True)],   # 관측 끊김
        [],                                          # 트랙 삭제
        [], [], [], [], [], [],
    ]
    res = run(mon, frames)
    assert res[2].holds == [1]
    assert all(r.raw for r in res[2:])               # 1.4 s 동안 유지 (t_end + t_hold 이내)
    assert all(r.estop for r in res[1:])


def test_pass_hold_released_by_time():
    mon = new_monitor(hold_max=1.0)
    frames = [[car(1, -1.6, 3.5, 12.0)], [car(1, -1.3, 3.5, 12.0, coasting=True)]] + [[]] * 12
    res = run(mon, frames)
    assert res[1].holds == [1]
    assert res[-1].holds == []
    assert not res[-1].estop


def test_pass_hold_not_triggered_when_far():
    mon = new_monitor()
    frames = [[car(1, -15.0, 3.5, 11.0)], [car(1, -14.0, 3.5, 11.0, coasting=True)], []]
    res = run(mon, frames)
    assert all(r.holds == [] for r in res)


def test_lane_change_completion_resets():
    mon = new_monitor()
    run(mon, [[]])
    assert mon.lane.active
    ego_done = EgoState(1.0, 40.0, 3.5, 0.0, 10.0)
    r = mon.step(1.0, ego_done, straight_route(40.0, 3.5), [])
    assert not r.lane_change_active


def test_target_kept_until_completion_even_if_reclassified():
    """자차가 경계를 넘어 대상이 '자차 현재 차선'이 되어도 종료 전이면 계속 감시."""
    mon = new_monitor()
    run(mon, [[]])
    ego_mid = EgoState(0.5, 20.0, 2.5, 0.12, 10.0)   # 경계 통과 후, 아직 수렴 전
    route = lane_change_route()[40:]
    r = mon.step(0.5, ego_mid, route, [car_at_map(1, ego_mid, 5.0, 3.5, 20.0)])
    assert r.lane_change_active
    assert r.watched == [1]


def test_uncompensated_tracks_stop_during_lane_change():
    mon = new_monitor()
    obs = car(1, -20.0, 3.5, 0.0)
    obs.compensated = False
    res = run(mon, [[obs]] * 2)
    assert res[0].reason == "uncompensated_tracks" and res[1].estop


def test_invalid_ego_stops():
    mon = new_monitor()
    bad = EgoState(0.0, 0.0, 0.0, 0.0, 10.0, valid=False)
    res = run(mon, [[]] * 2, ego=bad)
    assert res[1].estop


def test_stopped_while_already_overlapping_keeps_entry_zero():
    """이미 횡방향으로 겹친 채 정지: 재출발 여부와 무관하게 t_enter = 0."""
    route = lane_change_route()
    i = 36                                            # x = 18 m, 앞 모서리가 이미 목표 차선 안
    yaw = float(np.arctan2(route[i + 1, 1] - route[i, 1], route[i + 1, 0] - route[i, 0]))
    ego = EgoState(0.0, route[i, 0], route[i, 1], yaw, 0.0)
    obs = car_at_map(1, ego, 5.0, 3.5, 5.0)
    res = run(new_monitor(), [[obs]] * 2, ego=ego, route=route[i:])
    assert res[0].judgments[0].t_enter == 0.0
    assert res[1].estop


def test_mgeo_json_loading(tmp_path):
    import json
    data = [
        {"idx": "A", "points": [[0, 0, 0], [50, 0, 0]], "from_node_idx": "n0", "to_node_idx": "n1"},
        {"idx": "B", "points": [[50, 0, 0], [100, 0, 0]], "from_node_idx": "n1", "to_node_idx": "n2"},
        {"idx": "C", "points": [[0, 3.5, 0], [100, 3.5, 0]], "from_node_idx": "m0", "to_node_idx": "m1"},
    ]
    (tmp_path / "link_set.json").write_text(json.dumps(data))
    lm = LaneMap.from_mgeo(str(tmp_path))
    assert lm.match(10.0, 0.3, 0.0) == "A"
    assert lm.match(10.0, 3.0, 0.0) == "C"
    assert lm.match(10.0, 0.3, np.pi) is None          # 역방향은 매칭하지 않음
    ch = lm.chain("A")
    assert ch.ordered_ids == ["A", "B"] and ch.poly.length == pytest.approx(100.0)
