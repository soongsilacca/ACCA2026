"""npz 재생 경로 확인: 합성 bag 데이터 -> replay_sweep CLI."""
import json
import os
import subprocess
import sys

import numpy as np

from safety_monitor.common.params import CollisionParams
from safety_monitor.lanes.mgeo_lanes import LaneMap
from safety_monitor.replay import ReplayData, estop_intervals, run

HERE = os.path.dirname(os.path.abspath(__file__))


def write_fixture(tmp_path):
    links = []
    for lane, y in ((0, 0.0), (1, 3.5)):
        links.append({"idx": "L%d" % lane, "points": [[-300, y, 0], [300, y, 0]],
                      "from_node_idx": "a%d" % lane, "to_node_idx": "b%d" % lane})
    (tmp_path / "link_set.json").write_text(json.dumps(links))

    dt, n = 0.137, 20
    frame_t = np.arange(n) * dt
    rows = []
    for i in range(n):
        # 목표 차선 후방 20 m, 상대속도 10 m/s 로 접근 (앞면만 보이는 박스)
        rows.append([i, 7, -20.0 + 10.0 * i * dt, 3.5, 0.4, 1.8, 1.5, 20.0, 0.0, 1, 1, 0, 0.0])
    ego_t = np.arange(0, n * dt + 0.5, 0.02)
    ego = np.column_stack([ego_t, 10.0 * ego_t, np.zeros_like(ego_t),
                           np.zeros_like(ego_t), np.full_like(ego_t, 10.0)])
    # local route (base_link): 5 m 앞부터 30 m 동안 좌측 3.5 m 이동
    x = np.linspace(0, 60, 121)
    u = np.clip((x - 5) / 30, 0, 1)
    route = np.column_stack([x, 3.5 * (3 * u ** 2 - 2 * u ** 3)])
    route_t = frame_t.copy()
    # 자차가 진행하면 base_link 기준 경로도 같이 당겨짐 (여기선 첫 몇 프레임만 쓰도록 단순화)
    pts = np.vstack([route - [10.0 * t, 0.0] for t in route_t])
    offsets = np.arange(len(route_t) + 1) * len(route)
    np.savez_compressed(tmp_path / "case.npz", trk_frame_t=frame_t, trk_rows=np.array(rows, float),
                        ego=ego, route_t=route_t, route_offsets=offsets, route_pts=pts,
                        route_frame=np.asarray("base_link"))
    return tmp_path


def test_replay_run(tmp_path):
    d = write_fixture(tmp_path)
    data = ReplayData(str(d / "case.npz"))
    res = run(data, CollisionParams(), LaneMap.from_mgeo(str(d)))
    iv = estop_intervals(res)
    assert res[0].raw and res[1].estop
    assert len(iv) >= 1 and iv[0][2].startswith("collision:7")


def test_replay_sweep_cli(tmp_path):
    d = write_fixture(tmp_path)
    script = os.path.join(HERE, "..", "scripts", "replay_sweep.py")
    out = subprocess.run([sys.executable, script, str(d / "case.npz"), "--mgeo", str(d),
                          "--set", "horizon=1.0,3.0", "--frames", str(d / "f.csv")],
                         capture_output=True, text=True, check=True).stdout
    assert "[horizon=1.0]" in out and "[horizon=3.0]" in out
    assert os.path.exists(d / "f.csv")
    print(out)
