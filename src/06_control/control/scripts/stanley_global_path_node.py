#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
stanley_global_path_node.py

Stanley lateral controller with PyTorch Multi-Task Real-Time Dynamic Parameter Optimization.

Subscribes:
  /global_trajectory             (global_path_planner/PlannerTrajectory)
  /localization/kinematic_state  (nav_msgs/Odometry)

Publishes:
  /cmd                           (morai_msgs/CtrlCmd)
"""

import os
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from global_path_planner.msg import PlannerTrajectory
from morai_msgs.msg import CtrlCmd
from tf.transformations import euler_from_quaternion

# PyTorch 지원 확인
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class StanleyGlobalPathController:
    # Vehicle geometry (Hyundai Ioniq 5)
    WHEELBASE      = 3.000   # m
    FRONT_OVERHANG = 0.845   # m
    REAR_OVERHANG  = 0.790   # m

    def __init__(self):
        rospy.init_node('stanley_global_path', anonymous=False)

        def get_p(name, default):
            return rospy.get_param(f'~stanley/{name}', rospy.get_param(f'~{name}', default))

        # Base static fallbacks
        self.k_e_base          = float(get_p('k_e',           2.2))
        self.k_v_base          = float(get_p('k_v',           1.2))
        self.target_speed      = float(get_p('target_speed',  8.0))
        self.accel_value       = float(get_p('accel_value',   0.4))
        self.brake_value       = float(get_p('brake_value',   0.0))
        self.base_lookahead_b  = float(get_p('base_lookahead', 3.0))
        self.lookahead_gain    = float(get_p('lookahead_gain', 0.1))
        self.lookahead_idx     = int(get_p('lookahead_idx', 2))
        self.max_steer_deg     = float(get_p('max_steer_deg', 40.0))
        self.control_hz        = float(get_p('control_hz',    20.0))
        self.max_idx_jump      = int(get_p('max_idx_jump',  10))

        self.max_steer_rad = math.radians(self.max_steer_deg)

        # Active Dynamic Gains
        self.k_e            = self.k_e_base
        self.k_v            = self.k_v_base
        self.base_lookahead = self.base_lookahead_b
        self.last_steer_cmd = 0.0

        # State
        self.path:           PlannerTrajectory = None
        self.odom:           Odometry = None
        self.path_xy:        np.ndarray = None   # (N, 2)
        self.target_speeds:  np.ndarray = None   # (N,)
        self.closest_idx:    int = 0
        self.path_updated:   bool = False
        self.curve_gain:     float = 0.0
        self.current_yaw_rate: float = 0.0
        self.cte_integral:   float = 0.0

        # PyTorch 모델 및 실시간 학습 버퍼
        self.param_model = None
        self.online_optimizer = None
        self.online_buffer = []
        self.online_step = 0

        # 패키지 경로 탐색 및 모델 로드
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        model_dir = os.path.join(package_path, 'model')
        os.makedirs(model_dir, exist_ok=True)
        self.save_model_path = os.path.join(model_dir, 'stanley_param.pt')

        self._init_torch_model(package_path)

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd', CtrlCmd, queue_size=1)

        # Subscribers
        rospy.Subscriber('/global_trajectory', PlannerTrajectory, self.trajectory_callback, queue_size=1)
        rospy.Subscriber('/localization/kinematic_state', Odometry, self.odom_callback, queue_size=1)

        # Control loop timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_hz), self.control_loop)

        rospy.loginfo(
            f"[Stanley-GlobalPath] Started | PyTorch Multi-Task Adaptive Engine Active | "
            f"k_e={self.k_e:.2f} k_v={self.k_v:.2f} target_speed={self.target_speed:.1f}m/s"
        )

    def _init_torch_model(self, package_path):
        if not TORCH_AVAILABLE:
            rospy.logwarn("[Stanley-ML] PyTorch is not available. Using static fallback gains.")
            return

        try:
            import sys
            script_dir = os.path.dirname(os.path.abspath(__file__))
            if script_dir not in sys.path:
                sys.path.insert(0, script_dir)

            from train_stanley_model import NormalizedStanleyParamMLP
            model_candidates = [
                self.save_model_path,
                os.path.join(package_path, 'config', 'stanley_param.pt'),
            ]
            loaded = False
            for m_path in model_candidates:
                if os.path.exists(m_path):
                    try:
                        m = torch.jit.load(m_path)
                        m.eval()
                        test_in = torch.randn(1, 6, dtype=torch.float32)
                        if m(test_in).shape[1] == 4:
                            self.param_model = m
                            rospy.loginfo("[Stanley-ML] Valid 4-output TorchScript model loaded from: %s", m_path)
                            loaded = True
                            break
                    except Exception:
                        try:
                            m = torch.load(m_path)
                            if hasattr(m, 'eval'): m.eval()
                            test_in = torch.randn(1, 6, dtype=torch.float32)
                            if m(test_in).shape[1] == 4:
                                self.param_model = m
                                rospy.loginfo("[Stanley-ML] Valid 4-output PyTorch native model loaded from: %s", m_path)
                                loaded = True
                                break
                        except Exception as e:
                            rospy.logwarn("[Stanley-ML] Failed to load candidate %s: %s", m_path, str(e))

            if not loaded:
                dummy_mean = np.array([8.0, 0.0, 0.0, 0.0, 0.0, 0.005], dtype=np.float32)
                dummy_std  = np.array([3.0, 0.1, 0.1, 0.5, 0.1, 0.01], dtype=np.float32)
                self.param_model = NormalizedStanleyParamMLP(dummy_mean, dummy_std, input_dim=6, hidden_dim=64, output_dim=4)
                rospy.loginfo("[Stanley-ML] Initialized new PyTorch NormalizedStanleyParamMLP (4-outputs) model for online learning.")

            if hasattr(self.param_model, 'parameters'):
                self.online_optimizer = torch.optim.AdamW(self.param_model.parameters(), lr=5e-5, weight_decay=1e-4)
        except Exception as e:
            rospy.logerr("[Stanley-ML] Model init exception: %s", str(e))

    def trajectory_callback(self, msg: PlannerTrajectory):
        if len(msg.waypoints) == 0:
            return
        xs = np.array([wp.position.x for wp in msg.waypoints])
        ys = np.array([wp.position.y for wp in msg.waypoints])
        self.target_speeds = np.array([wp.target_speed for wp in msg.waypoints])
        self.path_xy = np.stack([xs, ys], axis=1)
        self.path = msg
        self.closest_idx = 0
        self.path_updated = True
        rospy.loginfo(f"[Stanley-GlobalPath] Global Trajectory updated: {len(msg.waypoints)} waypoints.")

    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.current_yaw_rate = float(msg.twist.twist.angular.z)
        self.control_loop(None)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _get_pose(self):
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        x, y = pose.position.x, pose.position.y
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        speed = math.hypot(twist.linear.x, twist.linear.y)
        return x, y, yaw, speed

    def _find_closest_idx(self, fx: float, fy: float) -> int:
        path = self.path_xy
        if path is None or len(path) == 0:
            return 0
        n = len(path)

        if self.path_updated:
            dists = np.hypot(path[:, 0] - fx, path[:, 1] - fy)
            closest = int(np.argmin(dists))
            self.path_updated = False
            return closest

        start = self.closest_idx
        if start >= n:
            return n - 1

        search_window = max(self.max_idx_jump, 20)
        end = min(start + search_window, n)

        sub = path[start:end]
        dists = np.hypot(sub[:, 0] - fx, sub[:, 1] - fy)
        local_idx = int(np.argmin(dists))
        target_idx = start + local_idx

        if target_idx - self.closest_idx > self.max_idx_jump:
            target_idx = self.closest_idx + self.max_idx_jump

        return target_idx

    def _update_dynamic_parameters(self, speed: float, e_ct: float, e_yaw: float):
        if TORCH_AVAILABLE and self.param_model is not None:
            try:
                in_feat = np.array([[
                    float(speed),
                    float(self.current_yaw_rate),
                    float(self.last_steer_cmd),
                    float(e_ct),
                    float(e_yaw),
                    float(self.curve_gain)
                ]], dtype=np.float32)

                with torch.no_grad():
                    inp_t = torch.from_numpy(in_feat)
                    out_t = self.param_model(inp_t).squeeze().numpy()
                    if out_t.shape[0] >= 3:
                        self.k_e            = float(np.clip(out_t[0], 0.15, 2.5))
                        self.k_v            = float(np.clip(out_t[1], 0.5,  6.0))
                        self.base_lookahead = float(np.clip(out_t[2], 1.5,  7.0))
                        # lateral_offset is strictly FIXED (static parameter)
            except Exception as e:
                rospy.logerr_throttle(5.0, "[Stanley-ML] Parameter inference error: %s", str(e))

        if speed >= 1.5:
            sample = [
                float(speed), float(self.current_yaw_rate), float(self.last_steer_cmd),
                float(e_ct), float(e_yaw), float(self.curve_gain),
                float(abs(e_ct)), float(abs(self.target_speed - speed)), float(abs(e_yaw)),
                float(e_ct)
            ]
            self.online_buffer.append(sample)
            if len(self.online_buffer) > 400:
                self.online_buffer.pop(0)

            self.online_step += 1
            if (self.online_step % 10 == 0) and (self.online_optimizer is not None) and (len(self.online_buffer) >= 32):
                try:
                    indices = np.random.choice(len(self.online_buffer), 32, replace=False)
                    batch = np.array([self.online_buffer[i] for i in indices], dtype=np.float32)
                    bx = torch.from_numpy(batch[:, :6])
                    
                    e_ct_b    = torch.from_numpy(batch[:, 6])
                    v_err_b   = torch.from_numpy(batch[:, 7])
                    e_yaw_b   = torch.from_numpy(batch[:, 8])
                    raw_e_ct  = torch.from_numpy(batch[:, 9])

                    self.param_model.train()
                    self.online_optimizer.zero_grad()
                    pred = self.param_model(bx)  # (32, 4) [k_e, k_v, base_lookahead, lateral_offset]

                    v_b = bx[:, 0]
                    curve_b = bx[:, 5]
                    
                    target_k_e    = torch.clamp((1.2 + 0.5 * e_ct_b + 0.2 * e_yaw_b) / (1.0 + 0.05 * torch.clamp(v_b - 5.0, min=0.0)), 0.8, 2.2)
                    target_k_v    = torch.clamp(1.2 + 0.15 * v_b, 1.0, 3.0)
                    target_lh     = torch.clamp(2.5 + 0.15 * v_b, 1.5, 7.0)
                    target_offset = torch.clamp(0.6 * torch.clamp(raw_e_ct, min=0.0) + 0.3 * curve_b, 0.00, 0.50)

                    if pred.shape[1] == 4:
                        loss = torch.mean((pred[:, 0] - target_k_e)**2 +
                                          (pred[:, 1] - target_k_v)**2 +
                                          (pred[:, 2] - target_lh)**2 +
                                          (pred[:, 3] - target_offset)**2)
                    else:
                        loss = torch.mean((pred[:, 0] - target_k_e)**2 +
                                          (pred[:, 1] - target_k_v)**2 +
                                          (pred[:, 2] - target_lh)**2)

                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(self.param_model.parameters(), max_norm=0.5)
                    self.online_optimizer.step()
                    self.param_model.eval()

                    rospy.loginfo_throttle(10.0, f"[Stanley-ONLINE] Param Model self-tuned in RAM (loss: {loss.item():.4f})")
                except Exception:
                    pass

            if (self.online_step % 300 == 0) and hasattr(self.param_model, 'eval'):
                try:
                    example_in = torch.randn(1, 6, dtype=torch.float32)
                    traced = torch.jit.trace(self.param_model, example_in)
                    traced.save(self.save_model_path)
                    rospy.loginfo(f"[Stanley-SAVE] Checkpoint updated and saved -> {self.save_model_path}")
                except Exception:
                    pass

    def _stanley_steer(self, x: float, y: float, yaw: float, speed: float):
        path = self.path_xy
        if path is None or len(path) == 0:
            return 0.0, 0, 0.0, 0.0

        # 1. Dynamic lookahead to compensate for latency at speed
        dynamic_lookahead = self.base_lookahead + self.lookahead_gain * speed
        fx = x + dynamic_lookahead * math.cos(yaw)
        fy = y + dynamic_lookahead * math.sin(yaw)

        # 2. Find closest waypoint on reference path
        idx = self._find_closest_idx(fx, fy)
        self.closest_idx = idx
        n = len(path)

        # 3. Reference path heading
        heading_idx = min(idx + self.lookahead_idx, n - 1)
        if heading_idx > idx:
            dx = path[heading_idx, 0] - path[idx, 0]
            dy = path[heading_idx, 1] - path[idx, 1]
        else:
            prev_idx = max(idx - 1, 0)
            dx = path[idx, 0] - path[prev_idx, 0]
            dy = path[idx, 1] - path[prev_idx, 1]

        path_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(path_yaw - yaw)

        if idx + 2 < n:
            v1 = path[idx + 1] - path[idx]
            v2 = path[idx + 2] - path[idx + 1]
            self.curve_gain = float(abs(math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])))

        # 4. Cross-track error (cte)
        nearest_x, nearest_y = path[idx, 0], path[idx, 1]
        ex = nearest_x - fx
        ey = nearest_y - fy
        cte = math.cos(path_yaw) * ey - math.sin(path_yaw) * ex

        # 5. Dynamic neural network parameter update
        self._update_dynamic_parameters(speed, cte, heading_error)

        # 6. High-speed error desensitization
        speed_attenuation = 1.0 + 0.05 * max(speed - 5.0, 0.0)
        effective_k_e = self.k_e / speed_attenuation

        # 7. Stanley Steering Angle Calculation (Yaw error attenuated, CTE response strengthened)
        cte_term = math.atan2(effective_k_e * cte, self.k_v + speed)
        
        yaw_weight = 0.50  # Attenuate heading error to prevent sharp/fast turns
        cte_weight = 1.25  # Strengthen CTE response for precise centering
        raw_steer = (yaw_weight * heading_error) + (cte_weight * cte_term)
        raw_steer = max(-self.max_steer_rad, min(self.max_steer_rad, raw_steer))

        # 8. Steer EMA Smoothing Filter
        steer_alpha = 0.35
        steer = steer_alpha * raw_steer + (1.0 - steer_alpha) * self.last_steer_cmd
        self.last_steer_cmd = steer
        return steer, idx, cte, heading_error

    def control_loop(self, event):
        if self.path is None or self.path_xy is None or len(self.path_xy) == 0:
            rospy.logwarn_throttle(5.0, "[Stanley-GlobalPath] Waiting for /global_trajectory...")
            return
        if self.odom is None:
            rospy.logwarn_throttle(5.0, "[Stanley-GlobalPath] Waiting for /localization/kinematic_state...")
            return

        x, y, yaw, speed = self._get_pose()
        steer_rad, idx, e_ct, e_yaw = self._stanley_steer(x, y, yaw, speed)

        base_target_speed = self.target_speeds[idx] if (self.target_speeds is not None and idx < len(self.target_speeds)) else self.target_speed

        # Automatic slowdown on curves to prevent turning too fast & crossing lanes
        if self.curve_gain > 0.02:
            curve_speed_limit = max(4.0, base_target_speed / (1.0 + 10.0 * self.curve_gain))
            target_speed = min(base_target_speed, curve_speed_limit)
        else:
            target_speed = base_target_speed

        speed_error = target_speed - speed
        if speed_error > 0:
            accel_gain = self.accel_value if self.accel_value > 0.0 else 0.4
            accel = min(1.0, max(0.1, speed_error * accel_gain))
            brake = 0.0
        else:
            accel = 0.0
            if target_speed < 0.5:
                brake = 1.0
            else:
                brake = min(0.6, abs(speed_error) * 0.15)

        cmd = CtrlCmd()
        cmd.ctrl_mode = 2        # AutoMode
        cmd.gear      = 4        # Drive
        cmd.cmd_type  = 1        # Throttle/Brake/Steer mode
        cmd.accel     = float(accel)
        cmd.brake     = float(brake)
        cmd.steer     = float(steer_rad)

        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            2.0,
            f"[Stanley-Adaptive] v={speed:.1f}m/s e_ct={e_ct:.3f}m e_yaw={math.degrees(e_yaw):.2f}deg "
            f"k_e={self.k_e:.2f} k_v={self.k_v:.2f} lookahead={self.base_lookahead:.2f}m"
        )


if __name__ == '__main__':
    try:
        StanleyGlobalPathController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
