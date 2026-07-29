#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import socket
import struct
import threading
import rospy
from morai_msgs.msg import EgoVehicleStatus
import tkinter as tk
from tkinter import ttk, messagebox

class TrafficLightControllerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Traffic Light Controller (UDP)")
        self.geometry("450x450")
        self.configure(bg="#1e1e2e")

        # ROS Params
        self.udp_ip = rospy.get_param('~ip', '127.0.0.1')
        self.udp_port = rospy.get_param('~port', 7607)
        try:
            import rospkg
            default_map_dir = os.path.join(rospkg.RosPack().get_path('hdmap_loader'), 'scripts')
        except Exception:
            default_map_dir = str(Path(__file__).resolve().parents[3] / '02_map' / 'hdmap_loader' / 'scripts')
        self.map_dir = rospy.get_param('~map_dir', default_map_dir)

        self.current_link_id = "Unknown"
        self.target_tl_id = "None"
        self.link_to_tl = {}

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

        btn_frame = ttk.Frame(self, padding=10)
        btn_frame.pack(fill="both", expand=True)

        # (Name, Status Value, Color Hex)
        self.buttons_info = [
            ("Red", 1, "#f38ba8"),
            ("Yellow", 4, "#f9e2af"),
            ("Green", 16, "#a6e3a1"),
            ("GreenLeft", 32, "#94e2d5"),
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
            # 1. Load Nodes
            node_path = f"{self.map_dir}/node_set.json"
            with open(node_path, 'r', encoding='utf-8') as f:
                node_data = json.load(f)
            
            # Map node_id -> traffic_light_id
            node_to_tl = {}
            for n in node_data:
                tl_id = n.get('traffic_light_id')
                if tl_id:
                    node_to_tl[n.get('idx')] = tl_id

            # 2. Load Links
            link_path = f"{self.map_dir}/link_set.json"
            with open(link_path, 'r', encoding='utf-8') as f:
                link_data = json.load(f)
            
            # 3. Map link_id -> traffic_light_id via to_node
            count = 0
            for link in link_data:
                link_idx = link.get('idx')
                to_node = link.get('to_node_idx')
                if to_node in node_to_tl:
                    self.link_to_tl[link_idx] = node_to_tl[to_node]
                    count += 1
            
            rospy.loginfo(f"Successfully loaded {count} Link -> Traffic Light mappings from node_set & link_set.")
        except Exception as e:
            rospy.logerr(f"Failed to load map data for traffic lights: {e}")

    def _ros_spin(self):
        rospy.Subscriber('/morai/ego_vehicle_status', EgoVehicleStatus, self._ego_status_cb)
        rospy.Subscriber('/morai/ego_topic', EgoVehicleStatus, self._ego_status_cb) # fallback
        rospy.spin()

    def _ego_status_cb(self, msg):
        link_id = msg.link_id
        if link_id != self.current_link_id:
            self.current_link_id = link_id
            self.target_tl_id = self.link_to_tl.get(link_id, "None")
            
            # Update UI safely
            self.after(0, self._update_ui_labels)

    def _update_ui_labels(self):
        self.lbl_link.config(text=f"Current Link ID: {self.current_link_id}")
        self.lbl_tl.config(text=f"Target Traffic Light ID: {self.target_tl_id}")

    def send_traffic_light_cmd(self, status_val):
        if self.target_tl_id == "None" or not self.target_tl_id:
            messagebox.showwarning("Warning", "No traffic light associated with the current link!")
            return

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
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
            
            tl_idx_bytes = self.target_tl_id.encode('utf-8')
            # Pad or truncate to 12 bytes
            tl_idx_padded = tl_idx_bytes.ljust(12, b'\x00')[:12]
            
            tail = b'\r\n'

            packet = struct.pack(
                '<14si3i12sh2s',
                header,
                data_length,
                aux[0], aux[1], aux[2],
                tl_idx_padded,
                int(status_val),
                tail
            )
            
            sock.sendto(packet, (self.udp_ip, self.udp_port))
            sock.close()
            rospy.loginfo(f"Sent UDP TrafficLight CMD: ID={self.target_tl_id}, Status={status_val} to {self.udp_ip}:{self.udp_port}")
            
        except Exception as e:
            rospy.logerr(f"Failed to send UDP packet: {e}")
            messagebox.showerror("Error", f"Failed to send UDP packet:\n{e}")

if __name__ == '__main__':
    rospy.init_node('traffic_light_controller_gui', anonymous=True)
    app = TrafficLightControllerGUI()
    app.mainloop()
