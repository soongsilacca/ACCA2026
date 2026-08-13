# Multimodal Planner V8

V8 removes learned speed imitation. The network predicts only a fixed-distance
Frenet lateral residual plus `STOP/DRIVE`.

```text
shared V7 encoders
  ├─ spatial Δd head: s=3,6,...,60 m
  └─ motion-state head: STOP / DRIVE

long Mission-Planner route + interpolated Δd
  → 0.1 m × 80 m MPC path
  → deterministic maximum-feasible velocity profile
```

The existing processed dataset contains a 63 m route (roughly 61 m forward),
so supervised residual anchors stop at 60 m. Stations not reached by the raw
four-second target are validity-masked; their labels are not fabricated or
corrected. At runtime the Mission Planner must provide at least 80 m ahead,
preferably a 100 m local crop from the global route. Straight extrapolation
beyond a short route is rejected.

The velocity planner uses competition maximum speed, path curvature, vehicle
acceleration/deceleration constraints, and optional signal/obstacle stopping
distance. Human future speed and the MGeo speed-zone flag do not command V8.
