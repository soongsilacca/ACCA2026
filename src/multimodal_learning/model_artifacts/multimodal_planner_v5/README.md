# Multi-View Temporal Trajectory Planner V5

V5 keeps the V4 K=1 sensor/fusion architecture and fixes the controller
boundary. Training now supervises the transition from the current local ego
origin `(0,0)` to the first future waypoint. There is no mode-score head, mode
classification loss, oracle mode, or mode-utilization metric.

## Output and target contract

The model returns:

- `trajectory`: `[batch, 20, 4]`
- `trajectory_with_origin`: `[batch, 21, 4]`

The four channels are `relative_x`, `relative_y`, `relative_yaw`, and
`future_speed`. The controller trajectory prepends `(0, 0, 0)` as the current
local-route origin; its speed is copied from the first future point.

The loader uses provided target values without repair, interpolation, smoothing,
or GPS-jump correction. Current legacy files can only be used with the explicit
`--allow-legacy-target-fields` option.

## GT-matched derivative loss

V5 minimizes:

```text
L = L_position
  + 1.00 * L_start
  + 0.50 * L_step
  + 0.25 * L_acceleration
  + 0.20 * L_yaw
  + 0.10 * L_heading
  + 0.40 * L_speed
```

- `L_position`: SmoothL1 between predicted and GT XY.
- `L_start`: SmoothL1 between the predicted and GT first future waypoint. It
  does not force the first future waypoint to `(0,0)`.
- `L_step`: SmoothL1 between predicted and GT first XY differences after
  prepending `(0,0)` to both paths. This includes origin to first future point.
- `L_acceleration`: SmoothL1 between predicted and GT second XY differences on
  those origin-prepended paths.
- `L_yaw`: wrapped/cyclic yaw error.
- `L_heading`: cyclic cosine-direction error between predicted yaw and the
  direction computed from origin-prepended predicted XY. The first segment uses
  the local ego heading `0`. Its normalization is clamped near zero-length
  segments to keep gradients finite. GT-stationary segments are masked because
  path heading is undefined there.
- `L_speed`: SmoothL1 between predicted and GT future speed.

The second-difference target is the GT second difference, not zero. Therefore
valid GT curvature and lane changes are retained while non-GT zigzag is
penalized. Position loss keeps the solution anchored to GT and prevents a
constant wrong-side offset.

Weights are configurable with `--start-weight`, `--step-weight`,
`--acc-weight`, `--yaw-weight`, `--heading-weight`, and `--speed-weight`.

## Validation metrics

Metrics are reported for all, GPS-blackout, and non-blackout samples:

- ADE/FDE at 1, 2, and 4 seconds
- longitudinal/lateral/yaw MAE
- waypoint step MAE
- second-difference acceleration MAE
- XY/yaw heading consistency MAE
- first-waypoint error, predicted/GT first forward distance, and origin
  direction-mismatch rate
- GT-relative lateral sign-flip count and rate with a 0.1 m deadband

Sign flips are diagnostic only; they are not directly optimized.

## Train-only camera augmentation

V5 applies one geometry-preserving photometric transform shared by all three
cameras and all five history frames in a sample:

- color jitter probability `0.8`
- brightness factor `[0.8, 1.2]`
- contrast factor `[0.8, 1.2]`
- saturation factor `[0.85, 1.15]`
- hue shift `[-0.03, 0.03]`
- fog probability `0.25`, strength `[0.08, 0.28]`

Validation is never augmented. No crop, flip, rotation, LiDAR noise, route
perturbation, or target transformation is applied. Training augmentation can be
disabled for an ablation with `--no-photometric-augmentation`.

## Test

```bash
cd /home/libok/morai_project
/home/libok/vla_train_env/bin/python -m unittest \
  multimodal_planner_v5.test_training
```

## Training

```bash
cd /home/libok/morai_project
/home/libok/vla_train_env/bin/python -u -m multimodal_planner_v5.train \
  --data-root /home/libok/morai_project/morai_dataset/processed/v001 \
  --output-dir /home/libok/morai_project/training_outputs/multimodal_planner_v5_raw_v001_weighted \
  --epochs 20 \
  --batch-size 1 \
  --grad-accum 8 \
  --num-workers 2 \
  --warmup-epochs 0 \
  --blackout-weight 2.0 \
  --outlier-count 24 \
  --log-every 100 \
  --save-every 1 \
  --no-amp \
  --init-checkpoint /home/libok/morai_project/training_outputs/multimodal_planner_v4_raw_v001_weighted/epoch_005.pt \
  --allow-legacy-target-fields
```
