#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import math
import socket
import struct
import threading
import time
from pathlib import Path

import rospy
from morai_msgs.msg import EgoVehicleStatus
import tkinter as tk
from tkinter import ttk, messagebox

class TrafficLightControllerGUI(tk.Tk):
    TRAFFIC_LIGHT_ID_PREFIX = "C1256W"
    TRAFFIC_LIGHT_ID_MIN = 1
    TRAFFIC_LIGHT_ID_MAX = 124

    def __init__(self):
        super().__init__()
        self.title("Traffic Light Controller (UDP)")
        self.geometry("450x500")
        self.configure(bg="#1e1e2e")

        # ROS Params
        self.udp_ip = rospy.get_param('~ip', '127.0.0.1')
        self.udp_port = rospy.get_param('~port', 7607)
        # A complete 124-light sweep should finish quickly. At 2 ms per
        # packet, one sweep takes about 0.25 s instead of several seconds.
        self.all_send_interval = max(
            0.0, float(rospy.get_param('~all_send_interval', 0.002))
        )
        self.all_cycle_interval = max(
            0.0, float(rospy.get_param('~all_cycle_interval', 0.1))
        )
        self.all_send_repeats = max(
            1, int(rospy.get_param('~all_send_repeats', 1))
        )
        self.control_radius = max(
            0.0,
            float(
                rospy.get_param(
                    '~control_radius',
                    rospy.get_param('~nearest_tl_max_distance', 100.0),
                )
            ),
        )
        self.nearest_tl_max_distance = self.control_radius
        try:
            import rospkg
            default_map_dir = os.path.join(
                rospkg.RosPack().get_path('map_viz'),
                'scripts',
            )
        except Exception:
            default_map_dir = str(
                Path(__file__).resolve().parents[3]
                / 'map_viz'
                / 'scripts'
            )
        self.map_dir = rospy.get_param('~map_dir', default_map_dir)

        self.current_link_id = "Unknown"
        self.target_tl_id = "None"
        self.ego_x = None
        self.ego_y = None
        self.link_to_tl = {}
        self.all_tl_ids = [
            f"{self.TRAFFIC_LIGHT_ID_PREFIX}{index:06d}"
            for index in range(
                self.TRAFFIC_LIGHT_ID_MIN,
                self.TRAFFIC_LIGHT_ID_MAX + 1,
            )
        ]
        self.traffic_light_points = []
        self.traffic_light_types = {}
        self._all_send_stop = None
        self._all_send_thread = None
        self._all_status = None
        self._all_radius_mode = False

        self._load_traffic_light_mapping()

        # UI Setup
        style = ttk.Style(self)
        style.theme_use('default')
        style.configure("TFrame", background="#1e1e2e")
        style.configure("TLabel", background="#1e1e2e", foreground="#cdd6f4", font=("sans-serif", 10, "bold"))
        style.configure("TButton", background="#313244", foreground="#ffffff", font=("sans-serif", 9, "bold"))

        info_frame = ttk.Frame(self, padding=10)
        info_frame.pack(fill="x")
        
        self.lbl_link = ttk.Label(info_frame, text="Current Link ID: Unknown", foreground="#89b4fa")
        self.lbl_link.pack(anchor="w")

        self.lbl_tl = ttk.Label(info_frame, text="Target Traffic Light ID: None", foreground="#f9e2af")
        self.lbl_tl.pack(anchor="w")

        self.apply_to_all = tk.BooleanVar(value=False)
        self.chk_apply_to_all = tk.Checkbutton(
            info_frame,
            text=f"반경 {self.control_radius:.0f}m 내 신호등에 적용",
            variable=self.apply_to_all,
            command=self._on_apply_to_all_changed,
            bg="#1e1e2e",
            fg="#cdd6f4",
            activebackground="#1e1e2e",
            activeforeground="#cdd6f4",
            selectcolor="#313244",
            font=("sans-serif", 10, "bold"),
        )
        self.chk_apply_to_all.pack(anchor="w", pady=(8, 0))

        btn_frame = ttk.Frame(self, padding=10)
        btn_frame.pack(fill="both", expand=True)

        # (Name, Status Value, Color Hex)
        self.buttons_info = [
            ("Red", 1, "#f38ba8"),
            ("Yellow", 4, "#f9e2af"),
            ("Green", 16, "#a6e3a1"),
            ("GreenLeft", 32, "#94e2d5"),
            ("Red + GreenLeft", 33, "#e78284"),
            ("Green + GreenLeft", 48, "#74c7ec"),
            ("Yellow + Green", 20, "#cba6f7"),
            ("Yellow + GreenLeft", 36, "#89dceb"),
            ("Red + Yellow", 5, "#fab387"),
            ("Default (Auto)", -1, "#45475a")
        ]

        for name, val, color in self.buttons_info:
            btn = tk.Button(
                btn_frame, text=f"{name} ({val})", bg=color, fg="#11111b" if val != -1 else "#ffffff",
                font=("sans-serif", 10, "bold"), bd=0, pady=5,
                command=lambda v=val: self.send_traffic_light_cmd(v)
            )
            btn.pack(fill="x", pady=3)

        # Start ROS Subscriber in a separate thread
        threading.Thread(target=self._ros_spin, daemon=True).start()

    def _load_traffic_light_mapping(self):
        try:
            # 1. Load the complete simulator traffic-light positions.
            point_by_id = {}
            control_set_path = (
                f"{self.map_dir}/traffic_light_control_set.json"
            )
            with open(control_set_path, 'r', encoding='utf-8') as f:
                control_set = json.load(f)

            for traffic_light in control_set:
                tl_id = str(traffic_light.get('idx', ''))
                point = traffic_light.get('point')
                if (
                    tl_id in self.all_tl_ids
                    and isinstance(point, list)
                    and len(point) >= 2
                ):
                    point_by_id[tl_id] = (
                        float(point[0]),
                        float(point[1]),
                    )
                    self.traffic_light_types[tl_id] = str(
                        traffic_light.get('type', 'car')
                    ).lower()

            # 2. Load Nodes
            node_path = f"{self.map_dir}/node_set.json"
            with open(node_path, 'r', encoding='utf-8') as f:
                node_data = json.load(f)
            
            # Map node_id -> traffic_light_id
            node_to_tl = {}
            mapped_tl_ids = set()
            for n in node_data:
                tl_id = n.get('traffic_light_id')
                if tl_id:
                    node_to_tl[n.get('idx')] = tl_id
                    # LCS IDs are lane-control signs, not vehicle traffic lights.
                    is_vehicle_traffic_light = not str(tl_id).upper().startswith(
                        'LCS'
                    )
                    if is_vehicle_traffic_light:
                        mapped_tl_ids.add(tl_id)
                    point = n.get('point')
                    if (
                        is_vehicle_traffic_light
                        and isinstance(point, list)
                        and len(point) >= 2
                        and tl_id not in point_by_id
                    ):
                        point_by_id[tl_id] = (
                            float(point[0]),
                            float(point[1]),
                        )

            self.traffic_light_points = [
                (tl_id, point[0], point[1])
                for tl_id, point in point_by_id.items()
            ]

            # 3. Load Links
            link_path = f"{self.map_dir}/link_set.json"
            with open(link_path, 'r', encoding='utf-8') as f:
                link_data = json.load(f)
            
            # 4. Map link_id -> traffic_light_id via to_node
            count = 0
            for link in link_data:
                link_idx = link.get('idx')
                to_node = link.get('to_node_idx')
                if to_node in node_to_tl:
                    self.link_to_tl[link_idx] = node_to_tl[to_node]
                    count += 1
            
            rospy.loginfo(
                f"Successfully loaded {count} Link -> Traffic Light mappings "
                f"and {len(self.all_tl_ids)} controllable traffic lights "
                f"({len(self.traffic_light_points)} positions, "
                f"{len(mapped_tl_ids)} referenced by node_set.json)."
            )
        except Exception as e:
            rospy.logerr(f"Failed to load map data for traffic lights: {e}")

    def _ros_spin(self):
        rospy.Subscriber('/morai/ego_vehicle_status', EgoVehicleStatus, self._ego_status_cb)
        rospy.Subscriber('/morai/ego_topic', EgoVehicleStatus, self._ego_status_cb) # fallback
        rospy.spin()

    def _ego_status_cb(self, msg):
        self.ego_x = float(msg.position.x)
        self.ego_y = float(msg.position.y)
        link_id = msg.link_id
        target_tl_id = self.link_to_tl.get(link_id)
        if not target_tl_id:
            target_tl_id = self._find_nearest_traffic_light(
                msg.position.x, msg.position.y
            )

        if (
            link_id != self.current_link_id
            or target_tl_id != self.target_tl_id
        ):
            self.current_link_id = link_id
            self.target_tl_id = target_tl_id or "None"

            # Update UI safely
            self.after(0, self._update_ui_labels)
            self.after(0, self._send_priority_target)

    def _find_nearest_traffic_light(self, ego_x, ego_y):
        nearest_id = None
        nearest_distance = float('inf')

        for tl_id, tl_x, tl_y in self.traffic_light_points:
            distance = math.hypot(tl_x - ego_x, tl_y - ego_y)
            if distance < nearest_distance:
                nearest_id = tl_id
                nearest_distance = distance

        if nearest_distance <= self.nearest_tl_max_distance:
            return nearest_id
        return None

    def _find_traffic_lights_within_radius(self):
        if self.ego_x is None or self.ego_y is None:
            return []

        radius_ids = []
        for tl_id, tl_x, tl_y in self.traffic_light_points:
            distance = math.hypot(tl_x - self.ego_x, tl_y - self.ego_y)
            if distance <= self.control_radius:
                radius_ids.append(tl_id)
        return radius_ids

    def _send_priority_target(self):
        if (
            not self.apply_to_all.get()
            and self._all_status is not None
            and self.target_tl_id != "None"
        ):
            self._start_all_sender(
                [self.target_tl_id],
                self._all_status,
                radius_mode=False,
            )

    def _update_ui_labels(self):
        self.lbl_link.config(text=f"Current Link ID: {self.current_link_id}")
        self.lbl_tl.config(text=f"Target Traffic Light ID: {self.target_tl_id}")
        radius_count = len(self._find_traffic_lights_within_radius())
        self.chk_apply_to_all.config(
            text=(
                f"반경 {self.control_radius:.0f}m 내 신호등에 적용 "
                f"({radius_count}개)"
            )
        )

    def send_traffic_light_cmd(self, status_val):
        if self.apply_to_all.get():
            target_ids = self._find_traffic_lights_within_radius()
        elif self.target_tl_id != "None" and self.target_tl_id:
            target_ids = [self.target_tl_id]
        else:
            target_ids = []

        if not target_ids:
            warning = (
                f"No traffic lights found within {self.control_radius:.0f}m!"
                if self.apply_to_all.get()
                else "No traffic light associated with the current link!"
            )
            messagebox.showwarning("Warning", warning)
            return

        self._start_all_sender(
            target_ids,
            status_val,
            radius_mode=self.apply_to_all.get(),
        )

    def _on_apply_to_all_changed(self):
        self._stop_all_sender()

    def _start_all_sender(self, target_ids, status_val, radius_mode):
        self._stop_all_sender()

        stop_event = threading.Event()
        self._all_send_stop = stop_event
        self._all_status = status_val
        self._all_radius_mode = radius_mode
        self._all_send_thread = threading.Thread(
            target=self._run_all_sender,
            args=(target_ids, status_val, stop_event, radius_mode),
            daemon=True,
        )
        self._all_send_thread.start()

    def _stop_all_sender(self):
        if self._all_send_stop is not None:
            self._all_send_stop.set()
        self._all_send_stop = None
        self._all_send_thread = None
        self._all_status = None
        self._all_radius_mode = False

    def _run_all_sender(
        self, target_ids, status_val, stop_event, radius_mode
    ):
        rospy.loginfo(
            f"Started continuous TrafficLight CMD: Targets={len(target_ids)}, "
            f"Status={status_val}"
        )

        while not stop_event.is_set() and not rospy.is_shutdown():
            if radius_mode:
                target_ids = self._find_traffic_lights_within_radius()
            if not target_ids:
                if stop_event.wait(self.all_cycle_interval):
                    break
                continue
            if not self._send_traffic_light_cmds(
                target_ids,
                status_val,
                self.all_send_interval,
                stop_event,
                log_result=False,
                repeat_count=self.all_send_repeats,
            ):
                break
            if stop_event.wait(self.all_cycle_interval):
                break

        rospy.loginfo("Stopped continuous TrafficLight CMD.")

    def _send_traffic_light_cmds(
        self,
        target_ids,
        status_val,
        interval,
        stop_event=None,
        log_result=True,
        repeat_count=1,
    ):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                for index, tl_id in enumerate(target_ids):
                    target_status = self._status_for_traffic_light(
                        tl_id, status_val
                    )
                    packet = self._build_traffic_light_packet(
                        tl_id, target_status
                    )
                    for repeat_index in range(repeat_count):
                        if stop_event is not None and stop_event.is_set():
                            return True

                        sock.sendto(packet, (self.udp_ip, self.udp_port))

                        is_last_packet = (
                            index == len(target_ids) - 1
                            and repeat_index == repeat_count - 1
                        )
                        if interval > 0 and not is_last_packet:
                            if stop_event is not None:
                                if stop_event.wait(interval):
                                    return True
                            else:
                                time.sleep(interval)

            if log_result:
                rospy.loginfo(
                    f"Sent UDP TrafficLight CMD: Targets={len(target_ids)}, "
                    f"Status={status_val} to {self.udp_ip}:{self.udp_port}"
                )
            return True

        except Exception as e:
            rospy.logerr(f"Failed to send UDP packet: {e}")
            error_message = f"Failed to send UDP packet:\n{e}"
            self.after(
                0,
                lambda message=error_message: messagebox.showerror(
                    "Error", message
                ),
            )
            return False

    def _status_for_traffic_light(self, tl_id, status_val):
        if status_val == -1:
            return status_val

        # Pedestrian heads support red/green only. Convert vehicle-only
        # yellow/left states so these signals also receive a valid state.
        if self.traffic_light_types.get(tl_id) == 'pedestrian':
            return 16 if int(status_val) & 16 else 1
        return status_val

    def _build_traffic_light_packet(self, tl_id, status_val):
        # Header: #TrafficLight$ (14s)
        # Data Length: 14 (i)
        # Aux Data: 0, 0, 0 (3i)
        # TL Index: 12 bytes max (12s)
        # TL Status: short (h)
        # Tail: \r\n (2s)
        # Format: <14si3i12sh2s
        header = b'#TrafficLight$'
        data_length = 14
        aux = (0, 0, 0)

        tl_idx_bytes = tl_id.encode('utf-8')
        # Pad or truncate to 12 bytes
        tl_idx_padded = tl_idx_bytes.ljust(12, b'\x00')[:12]

        tail = b'\r\n'

        return struct.pack(
            '<14si3i12sh2s',
            header,
            data_length,
            aux[0], aux[1], aux[2],
            tl_idx_padded,
            int(status_val),
            tail
        )

if __name__ == '__main__':
    rospy.init_node('traffic_light_controller_gui', anonymous=True)
    app = TrafficLightControllerGUI()
    app.mainloop()
