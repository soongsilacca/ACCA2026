#!/usr/bin/env python3
"""npz를 재생하며 파라미터 조합별 E-STOP 결과를 비교한다. ROS 없이 실행 가능.

예)
  python3 replay_sweep.py lane_change.npz --mgeo ~/mgeo/K-City \
      --set horizon=2.5,3.0,3.5 --set long_margin=0.3,0.5
  python3 replay_sweep.py lane_change.npz --mgeo ~/mgeo/K-City --frames frames.csv
"""
import argparse
import csv
import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from safety_monitor.common.params import CollisionParams  # noqa: E402
from safety_monitor.lanes.mgeo_lanes import LaneMap  # noqa: E402
from safety_monitor.replay import ReplayData, estop_intervals, run  # noqa: E402


def parse_value(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--mgeo", required=True, help="link_set.json 또는 그 폴더")
    ap.add_argument("--params", help="collision.yaml (기본값 위에 덮어씀)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=V1,V2",
                    help="비교할 파라미터 값 목록")
    ap.add_argument("--frames", help="첫 조합의 프레임별 결과를 CSV로 저장")
    args = ap.parse_args()

    base = {}
    if args.params:
        import yaml
        with open(args.params) as f:
            base = yaml.safe_load(f) or {}
    grid = []
    for item in args.set:
        key, vals = item.split("=", 1)
        grid.append((key, [parse_value(v) for v in vals.split(",")]))

    lanemap = LaneMap.from_mgeo(os.path.expanduser(args.mgeo))
    data = ReplayData(args.npz)
    print("frames=%d, duration=%.1fs" % (len(data.frame_t),
          (data.frame_t[-1] - data.frame_t[0]) if len(data.frame_t) else 0.0))

    keys = [k for k, _ in grid]
    combos = list(itertools.product(*[v for _, v in grid])) or [()]
    t0 = float(data.frame_t[0]) if len(data.frame_t) else 0.0
    for n, combo in enumerate(combos):
        cfg = dict(base)
        cfg.update(dict(zip(keys, combo)))
        params, unknown = CollisionParams.from_dict(cfg)
        if unknown:
            print("  (ignored: %s)" % unknown)
        results = run(data, params, lanemap)
        iv = estop_intervals(results)
        label = " ".join("%s=%s" % kv for kv in zip(keys, combo)) or "base"
        n_est = sum(r.estop for r in results)
        print("[%s] estop_frames=%d intervals=%d" % (label, n_est, len(iv)))
        for a, b, why in iv:
            print("    %7.2f ~ %7.2f s  (%.2f s)  %s" % (a - t0, b - t0, b - a, why))
        if n == 0 and args.frames:
            with open(args.frames, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "raw", "estop", "lane_change", "reason", "watched",
                            "track", "gap", "v_rel", "t_enter", "t_start", "t_end", "t_c"])
                for r in results:
                    if not r.judgments:
                        w.writerow([r.stamp - t0, int(r.raw), int(r.estop),
                                    int(r.lane_change_active), r.reason, r.watched])
                    for j in r.judgments:
                        ws, we = j.window if j.window else ("", "")
                        w.writerow([r.stamp - t0, int(r.raw), int(r.estop),
                                    int(r.lane_change_active), r.reason, r.watched,
                                    j.track_id, j.gap, j.v_rel, j.t_enter, ws, we, j.t_c])
            print("frame log -> %s" % args.frames)


if __name__ == "__main__":
    main()
