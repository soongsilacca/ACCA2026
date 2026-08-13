# Multimodal Planner V7

V7 keeps the supplied Local Route as the geometric prior and learns only the
time-indexed speed profile and path-normal avoidance residual.

```text
MGeo speed-zone flag ─────────────→ fixed non-learned base speed
Camera + LiDAR + Ego + History ──→ speed delta and lateral residual Δd

future_speed = clamp(base_speed + Δspeed)
s[i] = s[i-1] + future_speed[i] × 0.2 sec
xy[i] = LocalRoute(s[i]) + Δd[i] × route_normal(s[i])
```

Direct model outputs:

- `future_speed`: 20 normalized values, 4 seconds
- `lateral_residual_m`: 20 signed values, left positive and right negative
- `motion_state_logits`: STOP / DRIVE

The derived controller trajectory remains `[20,4]` with
`relative_x, relative_y, relative_yaw, future_speed`. The current origin is
prepended separately for a `[21,4]` controller path.

The original dataset trajectory is never numerically corrected. Route progress
and signed lateral residual labels are deterministic projections of the
provided target onto the provided Local Route.

Avoidance samples are defined by a maximum absolute residual of at least 0.75 m
and receive a separate sample weight during training.

The current MGeo schema has a binary `speed_zone_flag`, not a numeric m/s
field. V7 therefore maps that flag to configurable fixed speeds. There is no
trainable base-speed head. When the MGeo producer publishes a numeric speed,
the fixed mapping can be replaced directly without changing the residual heads.
