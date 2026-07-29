#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MORAI & ROS Categorized Control Launcher GUI
A Tkinter-based control panel categorized into System, Sensor, Perception,
Localization, Planning, and Control sections.
Includes Real-time Sensor Topic Monitoring and Drive Mode Control (Manual / Auto / Parking).
"""

import os
import sys
import time
import signal
import socket
import struct
import subprocess
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

import rospy
from morai_msgs.msg import CtrlCmd

def find_workspace_root():
    curr = os.path.dirname(os.path.abspath(__file__))
    while curr != os.path.dirname(curr):
        if os.path.exists(os.path.join(curr, 'devel', 'setup.bash')):
            return curr
        curr = os.path.dirname(curr)
    # Fallback to standard relative path if not found
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../../'))

WORKSPACE_ROOT = find_workspace_root()

def get_bash_prefix():
    setup_path = os.path.join(WORKSPACE_ROOT, 'devel', 'setup.bash')
    return (
        "source /opt/ros/noetic/setup.bash; "
        f"source {setup_path}; "
    )

CATEGORIES = [
    {"id": "System", "name": "System & Core", "color": "#cba6f7", "desc": "ROS Core Master, MORAI Simulator & RViz"},
    {"id": "Scenario", "name": "Scenario", "color": "#f5e0dc", "desc": "MORAI Scenario Loading & Randomization"},
    {"id": "Sensor", "name": "Sensor & Network", "color": "#89dceb", "desc": "MORAI UDP Bridge & Sensor Communication"},
    {"id": "Perception", "name": "Perception", "color": "#a6e3a1", "desc": "LiDAR Filtering & Perception Pipeline"},
    {"id": "Localization", "name": "Localization", "color": "#f9e2af", "desc": "Global EKF & NDT Map Localization"},
    {"id": "Planning", "name": "Planning", "color": "#89b4fa", "desc": "Global Waypoint Planner & Local Lattice Planner"},
    {"id": "Control", "name": "Control", "color": "#f38ba8", "desc": "Stanley & Pure Pursuit Controllers"},
]

DEFAULT_COMMANDS = [
    # System
    {
        "id": "morai",
        "category": "System",
        "name": "MORAI Simulator",
        "cmd": "cd ~/MoraiLauncher_Lin && ./MORAISim.sh",
        "desc": "MORAI Autonomous Driving Simulator",
    },
    {
        "id": "roscore",
        "category": "System",
        "name": "ROS Core Master",
        "cmd": "roscore",
        "desc": "ROS Master & Parameter Server",
    },
    {
        "id": "rviz",
        "category": "System",
        "name": "RViz 3D Visualizer",
        "cmd": "rviz -d $(rospack find utils)/rviz/default.rviz",
        "desc": "RViz 3D Visualizer with default setup",
    },
    {
        "id": "load_scenario",
        "category": "Scenario",
        "name": "Load Specific Scenario",
        "cmd": "roslaunch morai_udp_bridge load_scenario.launch",
        "desc": "Load predefined specific scenario and map into MORAI",
    },
    {
        "id": "load_random_scenario",
        "category": "Scenario",
        "name": "Load Random Scenario",
        "cmd": "roslaunch morai_udp_bridge randomize_and_load.launch",
        "desc": "Randomize and load scenario and map into MORAI",
    },
    # Sensor & Network
    {
        "id": "morai_bridge",
        "category": "Sensor",
        "name": "MORAI UDP Bridge",
        "cmd": "roslaunch morai_udp_bridge morai_bridge.launch",
        "desc": "UDP Bridge for GPS, IMU, Ego Status, LiDAR & Cmd",
    },
    {
        "id": "traffic_light_ctrl",
        "category": "Sensor",
        "name": "Traffic Light Controller",
        "cmd": "roslaunch morai_udp_bridge traffic_light_controller.launch",
        "desc": "GUI to override traffic lights via UDP based on Ego Link",
    },

    # Perception
    {
        "id": "lidar_filtering",
        "category": "Perception",
        "name": "LiDAR Ground & Crop Filter",
        "cmd": "roslaunch lidar_filtering lidar_pipeline.launch",
        "desc": "LiDAR Ground & CropBox Filter Pipeline",
    },

    # Localization
    {
        "id": "localization",
        "category": "Localization",
        "name": "Global Localization EKF",
        "cmd": "roslaunch localization localization_global.launch",
        "desc": "Global EKF & NDT Map-based Localization",
    },

    # Planning
    {
        "id": "waypoint_planner",
        "category": "Planning",
        "name": "Waypoint Global Planner",
        "cmd": "roslaunch global_path_planner waypoint_global_path_planner.launch",
        "desc": "Waypoint-sequence Global Path Planner",
    },
    {
        "id": "static_path_pub",
        "category": "Planning",
        "name": "Static Path Marker Publisher",
        "cmd": "rosrun global_path_planner static_path_publisher_node.py",
        "desc": "Static Global Path Marker Publisher",
    },
    {
        "id": "local_planner",
        "category": "Planning",
        "name": "Lattice Local Path Planner",
        "cmd": "roslaunch local_path_planner local_path_planner.launch",
        "desc": "Lattice Rollout Obstacle Avoidance Planner",
    },
    {
        "id": "mgeo_marker",
        "category": "Planning",
        "name": "MGeo HD Map Marker",
        "cmd": "roslaunch hdmap_loader mgeo_marker.launch",
        "desc": "MGeo HD Map Link Visualizer",
    },

    # Control
    {
        "id": "stanley_global",
        "category": "Control",
        "name": "Stanley Controller (Global Path)",
        "cmd": "roslaunch control stanley_global_path.launch",
        "desc": "Stanley Controller with Static Target Speed",
    },
    {
        "id": "stanley_local",
        "category": "Control",
        "name": "Stanley Controller (Local Path)",
        "cmd": "roslaunch control stanley_local_path.launch",
        "desc": "Stanley Controller with Dynamic Speed Profile",
    },
    {
        "id": "pure_pursuit",
        "category": "Control",
        "name": "Pure Pursuit Controller",
        "cmd": "roslaunch control pure_pursuit.launch",
        "desc": "Pure Pursuit Lateral Controller",
    },
]

SENSOR_TOPIC_MAP = {
    "GPS": ["/gps", "/ublox/fix"],
    "IMU": ["/imu"],
    "Ego Status": ["/morai/ego_vehicle_status", "/morai/ego_topic"],
    "LiDAR": ["/velodyne_points", "/points_raw", "/autoware_lidar_pipeline/points_ground"],
    "Localization": ["/localization/kinematic_state", "/odom"]
}

class ProcessHandler:
    def __init__(self, cmd_id, name, category, cmd_str, log_callback, status_callback):
        self.cmd_id = cmd_id
        self.name = name
        self.category = category
        self.cmd_str = cmd_str
        self.log_callback = log_callback
        self.status_callback = status_callback
        self.process = None
        self.thread = None
        self.is_running = False

    def start(self):
        if self.is_running and self.process and self.process.poll() is None:
            self.log_callback(self.cmd_id, f"[{self.name}] Already running (PID: {self.process.pid})\n")
            return

        full_cmd = f"bash -c '{get_bash_prefix()} {self.cmd_str}'"
        self.log_callback(self.cmd_id, f"[{self.name}] Launching: {self.cmd_str}\n")
        
        try:
            self.process = subprocess.Popen(
                full_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                cwd=WORKSPACE_ROOT,
                text=True,
                bufsize=1
            )
            self.is_running = True
            self.status_callback(self.cmd_id, "RUNNING", self.process.pid)
            
            self.thread = threading.Thread(target=self._read_output, daemon=True)
            self.thread.start()
        except Exception as e:
            self.log_callback(self.cmd_id, f"[{self.name}] Error starting process: {e}\n")
            self.is_running = False
            self.status_callback(self.cmd_id, "STOPPED", None)

    def _read_output(self):
        try:
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    self.log_callback(self.cmd_id, line)
                else:
                    break
        except Exception:
            pass
        finally:
            if self.process:
                self.process.wait()
            self.is_running = False
            self.status_callback(self.cmd_id, "STOPPED", None)
            self.log_callback(self.cmd_id, f"[{self.name}] Process terminated.\n")

    def stop(self):
        if not self.is_running or not self.process:
            self.status_callback(self.cmd_id, "STOPPED", None)
            return

        self.log_callback(self.cmd_id, f"[{self.name}] Stopping process (PID: {self.process.pid})...\n")
        try:
            pgid = os.getpgid(self.process.pid)
            os.killpg(pgid, signal.SIGINT)
            
            for _ in range(10):
                if self.process.poll() is not None:
                    break
                time.sleep(0.1)
                
            if self.process.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            self.log_callback(self.cmd_id, f"[{self.name}] Stop error: {e}\n")
        finally:
            self.is_running = False
            self.status_callback(self.cmd_id, "STOPPED", None)

class ROSLauncherGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MORAI & ROS Categorized Control Dashboard")
        self.geometry("1500x900")
        self.minsize(1200, 700)
        
        # Modern Dark Color Palette
        self.colors = {
            "bg": "#1e1e2e",
            "card_bg": "#252538",
            "card_header": "#2b2b3f",
            "fg": "#cdd6f4",
            "subtext": "#a6adc8",
            "accent": "#89b4fa",
            "green": "#a6e3a1",
            "red": "#f38ba8",
            "yellow": "#f9e2af",
            "orange": "#fab387",
            "purple": "#cba6f7",
            "cyan": "#89dceb",
            "log_bg": "#11111b",
            "log_fg": "#a6e3a1",
            "entry_bg": "#313244",
            "button_bg": "#45475a"
        }

        self.configure(bg=self.colors["bg"])
        self._setup_styles()

        self.processes = {}
        self.log_widgets = {}
        self.status_labels = {}
        self.cmd_entries = {}
        self.category_section_frames = {}
        self.category_count_labels = {}
        self.sensor_status_labels = {}
        self.sensor_last_seen = {s: 0.0 for s in SENSOR_TOPIC_MAP.keys()}
        self.log_queue = queue.Queue()
        self.cmd_pub = None

        self.selected_category_filter = "All"
        self.current_vehicle_mode = "MANUAL"  # 'MANUAL', 'AUTO', or 'PARKING'
        self.is_parking_active = False
        self.parking_thread = None

        self._build_ui()
        self._init_default_commands()
        self.set_category_filter("All")
        
        self.after(100, self._process_log_queue)
        self.after(500, self._update_sensor_status_ui)

        self._start_ros_thread()

        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('default')
        
        style.configure(".", background=self.colors["bg"], foreground=self.colors["fg"])
        style.configure("TFrame", background=self.colors["bg"])
        style.configure("Card.TFrame", background=self.colors["card_bg"], relief="flat")
        
        style.configure("Header.TLabel", font=("sans-serif", 15, "bold"), foreground=self.colors["accent"], background=self.colors["bg"])
        style.configure("SubHeader.TLabel", font=("sans-serif", 10), foreground=self.colors["subtext"], background=self.colors["bg"])
        style.configure("ItemTitle.TLabel", font=("sans-serif", 10, "bold"), foreground=self.colors["fg"], background=self.colors["card_bg"])
        
        style.configure("TButton", font=("sans-serif", 9, "bold"), background=self.colors["button_bg"], foreground="#ffffff", borderwidth=0, padding=6)
        style.map("TButton", background=[("active", self.colors["accent"])], foreground=[("active", "#11111b")])
        
        style.configure("Start.TButton", background="#2e7d32", foreground="#ffffff")
        style.map("Start.TButton", background=[("active", "#4caf50")])
        
        style.configure("Stop.TButton", background="#c62828", foreground="#ffffff")
        style.map("Stop.TButton", background=[("active", "#ef5350")])

        style.configure("Restart.TButton", background="#d84315", foreground="#ffffff")
        style.map("Restart.TButton", background=[("active", "#ff7043")])

        style.configure("TNotebook", background=self.colors["bg"], borderwidth=0)
        style.configure("TNotebook.Tab", background=self.colors["card_bg"], foreground=self.colors["fg"], padding=[10, 5], font=("sans-serif", 9, "bold"))
        style.map("TNotebook.Tab", background=[("selected", self.colors["accent"])], foreground=[("selected", "#11111b")])

    def _start_ros_thread(self):
        def ros_worker():
            try:
                rospy.init_node('ros_launcher_gui', anonymous=True, disable_signals=True)
                self.cmd_pub = rospy.Publisher('/cmd', CtrlCmd, queue_size=10)
                
                def make_cb(sensor_name):
                    def cb(msg):
                        self.sensor_last_seen[sensor_name] = time.time()
                    return cb

                for sensor_name, topics in SENSOR_TOPIC_MAP.items():
                    for topic in topics:
                        try:
                            rospy.Subscriber(topic, rospy.AnyMsg, make_cb(sensor_name))
                        except Exception:
                            pass

                rospy.spin()
            except Exception:
                pass

        threading.Thread(target=ros_worker, daemon=True).start()

    def _build_ui(self):
        # Header Panel
        header_frame = ttk.Frame(self, padding=(15, 10))
        header_frame.pack(fill="x")
        
        title_label = ttk.Label(header_frame, text="MORAI & ROS Categorized Control Dashboard", style="Header.TLabel")
        title_label.pack(anchor="w")
        
        subtitle_label = ttk.Label(header_frame, text="System, Sensor, Perception, Localization, Planning, and Control sections", style="SubHeader.TLabel")
        subtitle_label.pack(anchor="w", pady=(2, 0))

        # Sensor Status & Vehicle Mode Control Frame
        top_ctrl_bar = tk.Frame(self, bg=self.colors["card_bg"], bd=1, relief="solid", highlightbackground="#313244", highlightthickness=1)
        top_ctrl_bar.pack(fill="x", padx=15, pady=5)

        # 1. Sensor Feed Status
        sensor_box = tk.Frame(top_ctrl_bar, bg=self.colors["card_bg"])
        sensor_box.pack(side="left", padx=10, pady=6)

        sensor_title_lbl = tk.Label(sensor_box, text="Sensors:", font=("sans-serif", 9, "bold"), fg=self.colors["accent"], bg=self.colors["card_bg"])
        sensor_title_lbl.pack(side="left", padx=(0, 10))

        for sensor_name in SENSOR_TOPIC_MAP.keys():
            box = tk.Frame(sensor_box, bg=self.colors["card_bg"])
            box.pack(side="left", padx=6)

            s_name_lbl = tk.Label(box, text=f"{sensor_name}:", font=("sans-serif", 8), fg=self.colors["fg"], bg=self.colors["card_bg"])
            s_name_lbl.pack(side="left", padx=(0, 2))

            s_status_lbl = tk.Label(box, text="[ OFF ]", font=("sans-serif", 8, "bold"), fg="#ffffff", bg="#45475a", padx=5, pady=1)
            s_status_lbl.pack(side="left")
            self.sensor_status_labels[sensor_name] = s_status_lbl

        # Separator Line
        sep = tk.Frame(top_ctrl_bar, bg="#45475a", width=1, height=24)
        sep.pack(side="left", padx=15, fill="y")

        # 2. Scenario loading is now handled via launch cards in the System category.

        # 3. Vehicle Mode Control Buttons (Manual / Auto / Parking)
        mode_box = tk.Frame(top_ctrl_bar, bg=self.colors["card_bg"])
        mode_box.pack(side="right", padx=15, pady=6)

        mode_lbl = tk.Label(mode_box, text="Vehicle Mode:", font=("sans-serif", 9, "bold"), fg=self.colors["orange"], bg=self.colors["card_bg"])
        mode_lbl.pack(side="left", padx=(0, 8))

        self.btn_manual = tk.Button(
            mode_box,
            text="Manual",
            font=("sans-serif", 9, "bold"),
            bg=self.colors["accent"],
            fg="#11111b",
            bd=0,
            padx=10,
            pady=3,
            command=lambda: self.set_drive_mode("MANUAL")
        )
        self.btn_manual.pack(side="left", padx=3)

        self.btn_auto = tk.Button(
            mode_box,
            text="Auto",
            font=("sans-serif", 9, "bold"),
            bg=self.colors["button_bg"],
            fg=self.colors["fg"],
            bd=0,
            padx=10,
            pady=3,
            command=lambda: self.set_drive_mode("AUTO")
        )
        self.btn_auto.pack(side="left", padx=3)

        self.btn_parking = tk.Button(
            mode_box,
            text="Parking [P]",
            font=("sans-serif", 9, "bold"),
            bg="#e65100",
            fg="#ffffff",
            bd=0,
            padx=12,
            pady=3,
            command=lambda: self.toggle_parking_mode()
        )
        self.btn_parking.pack(side="left", padx=(8, 0))

        # Toolbar Frame (Batch Process Actions)
        toolbar_frame = ttk.Frame(self, padding=(15, 2))
        toolbar_frame.pack(fill="x")

        start_all_btn = ttk.Button(toolbar_frame, text="Start All Processes", style="Start.TButton", command=self.start_all)
        start_all_btn.pack(side="left", padx=(0, 5))

        stop_all_btn = ttk.Button(toolbar_frame, text="Stop All Processes", style="Stop.TButton", command=self.stop_all)
        stop_all_btn.pack(side="left", padx=5)

        restart_all_btn = ttk.Button(toolbar_frame, text="Restart All", style="Restart.TButton", command=self.restart_all)
        restart_all_btn.pack(side="left", padx=5)

        clear_logs_btn = ttk.Button(toolbar_frame, text="Clear Logs", command=self.clear_all_logs)
        clear_logs_btn.pack(side="left", padx=5)

        toggle_logs_btn = ttk.Button(toolbar_frame, text="Toggle Logs", command=self.toggle_logs)
        toggle_logs_btn.pack(side="left", padx=5)

        add_cmd_btn = ttk.Button(toolbar_frame, text="+ Add Custom Command", command=self.add_custom_cmd_dialog)
        add_cmd_btn.pack(side="right", padx=5)

        # Category Filter Bar (All / System / Sensor / Perception / Localization / Planning / Control)
        filter_bar = tk.Frame(self, bg=self.colors["bg"])
        filter_bar.pack(fill="x", padx=15, pady=(5, 0))

        filter_lbl = tk.Label(filter_bar, text="Filter Category:", font=("sans-serif", 9, "bold"), fg=self.colors["subtext"], bg=self.colors["bg"])
        filter_lbl.pack(side="left", padx=(0, 10))

        self.filter_buttons = {}
        all_btn = tk.Button(
            filter_bar,
            text="All",
            font=("sans-serif", 9, "bold"),
            bg=self.colors["accent"],
            fg="#11111b",
            bd=0,
            padx=10,
            pady=3,
            command=lambda: self.set_category_filter("All")
        )
        all_btn.pack(side="left", padx=3)
        self.filter_buttons["All"] = all_btn

        for cat in CATEGORIES:
            btn = tk.Button(
                filter_bar,
                text=cat["name"],
                font=("sans-serif", 9, "bold"),
                bg=self.colors["button_bg"],
                fg=self.colors["fg"],
                bd=0,
                padx=10,
                pady=3,
                command=lambda c=cat["id"]: self.set_category_filter(c)
            )
            btn.pack(side="left", padx=3)
            self.filter_buttons[cat["id"]] = btn

        # Main Split Frame
        self.main_paned = ttk.PanedWindow(self, orient="horizontal")
        self.main_paned.pack(fill="both", expand=True, padx=15, pady=10)

        # Left Container: Categorized Command Cards Frame (Give it more priority)
        left_container = ttk.Frame(self.main_paned, style="Card.TFrame", width=600)
        self.main_paned.add(left_container, weight=3)

        canvas = tk.Canvas(left_container, bg=self.colors["card_bg"], highlightthickness=0)
        scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=canvas.yview)
        self.cmd_scroll_frame = ttk.Frame(canvas, style="Card.TFrame")

        self.cmd_scroll_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=self.cmd_scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        # Build Category Sections
        for cat in CATEGORIES:
            sec_frame = tk.Frame(self.cmd_scroll_frame, bg=self.colors["card_bg"])
            # grid placement is handled dynamically by set_category_filter

            # Category Header Bar
            cat_header = tk.Frame(sec_frame, bg=self.colors["card_header"], bd=1, relief="solid", highlightbackground=cat["color"], highlightthickness=1)
            cat_header.pack(fill="x", padx=5, pady=(5, 2))

            badge_lbl = tk.Label(cat_header, text=f" [{cat['name'].upper()}] ", font=("sans-serif", 9, "bold"), fg="#11111b", bg=cat["color"])
            badge_lbl.pack(side="left", padx=8, pady=6)

            desc_lbl = tk.Label(cat_header, text=cat["desc"], font=("sans-serif", 9, "italic"), fg=self.colors["subtext"], bg=self.colors["card_header"])
            desc_lbl.pack(side="left", padx=5)

            count_lbl = tk.Label(cat_header, text="0 Active", font=("sans-serif", 9, "bold"), fg=self.colors["subtext"], bg=self.colors["card_header"])
            count_lbl.pack(side="right", padx=10)
            self.category_count_labels[cat["id"]] = count_lbl

            # Container for command cards inside this category
            cards_container = tk.Frame(sec_frame, bg=self.colors["card_bg"])
            cards_container.pack(fill="x", expand=True)

            self.category_section_frames[cat["id"]] = {
                "section": sec_frame,
                "container": cards_container,
                "header": cat_header
            }

        # Right Container: Output Log Terminal Notebook (Hidden by default)
        self.right_container = ttk.Frame(self.main_paned, width=300)
        self.logs_visible = False
        # Not added to paned window initially

        self.log_notebook = ttk.Notebook(self.right_container)
        self.log_notebook.pack(fill="both", expand=True)

        main_log_frame = ttk.Frame(self.log_notebook)
        self.log_notebook.add(main_log_frame, text="All Logs")
        
        self.main_log_text = scrolledtext.ScrolledText(
            main_log_frame,
            bg=self.colors["log_bg"],
            fg=self.colors["log_fg"],
            insertbackground="#ffffff",
            font=("monospace", 9),
            wrap="none",
            width=50
        )
        self.main_log_text.pack(fill="both", expand=True)

    def _init_default_commands(self):
        for item in DEFAULT_COMMANDS:
            self.add_command_card(
                cmd_id=item["id"],
                name=item["name"],
                category=item["category"],
                cmd_str=item["cmd"],
                desc=item["desc"]
            )

    def toggle_logs(self):
        if self.logs_visible:
            self.main_paned.forget(self.right_container)
            self.logs_visible = False
        else:
            self.main_paned.add(self.right_container, weight=1)
            self.logs_visible = True

    def set_category_filter(self, category_id):
        self.selected_category_filter = category_id
        for c_id, btn in self.filter_buttons.items():
            if c_id == category_id:
                btn.config(bg=self.colors["accent"], fg="#11111b")
            else:
                btn.config(bg=self.colors["button_bg"], fg=self.colors["fg"])

        # 기존 그리드에서 모두 숨기기
        for info in self.category_section_frames.values():
            info["section"].grid_forget()

        # 보여줄 카테고리 필터링
        visible_cats = []
        for cat in CATEGORIES:
            if category_id == "All" or cat["id"] == category_id:
                visible_cats.append(cat["id"])

        # All 모드일 때는 3열, 단일 카테고리일 때는 1열
        cols = 3 if category_id == "All" else 1

        for i, cat_id in enumerate(visible_cats):
            row = i // cols
            col = i % cols
            sec_frame = self.category_section_frames[cat_id]["section"]
            sec_frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")

        # 컬럼 너비 균등 분배
        for c in range(3):
            self.cmd_scroll_frame.grid_columnconfigure(c, weight=0, uniform="")
            
        for c in range(cols):
            self.cmd_scroll_frame.grid_columnconfigure(c, weight=1, uniform="col")

    def add_command_card(self, cmd_id, name, category, cmd_str, desc=""):
        tab_frame = ttk.Frame(self.log_notebook)
        short_title = f"[{category[:3]}] {name}"
        self.log_notebook.add(tab_frame, text=short_title)

        log_text = scrolledtext.ScrolledText(
            tab_frame,
            bg=self.colors["log_bg"],
            fg=self.colors["log_fg"],
            insertbackground="#ffffff",
            font=("monospace", 9),
            wrap="none"
        )
        log_text.pack(fill="both", expand=True)
        self.log_widgets[cmd_id] = log_text

        handler = ProcessHandler(
            cmd_id=cmd_id,
            name=name,
            category=category,
            cmd_str=cmd_str,
            log_callback=self._enqueue_log,
            status_callback=self._update_status_ui
        )
        self.processes[cmd_id] = handler

        target_container = self.category_section_frames.get(category, {}).get("container", self.cmd_scroll_frame)

        card = tk.Frame(target_container, bg=self.colors["card_bg"], bd=1, relief="solid", highlightbackground="#313244", highlightthickness=1)
        card.pack(fill="x", padx=10, pady=4, expand=True)

        top_row = tk.Frame(card, bg=self.colors["card_bg"])
        top_row.pack(fill="x", padx=10, pady=(6, 2))

        lbl_name = tk.Label(top_row, text=name, font=("sans-serif", 10, "bold"), fg=self.colors["fg"], bg=self.colors["card_bg"])
        lbl_name.pack(side="left")

        status_lbl = tk.Label(top_row, text="[STOPPED]", font=("sans-serif", 9, "bold"), fg=self.colors["red"], bg=self.colors["card_bg"])
        status_lbl.pack(side="right")
        self.status_labels[cmd_id] = status_lbl

        entry_row = tk.Frame(card, bg=self.colors["card_bg"])
        entry_row.pack(fill="x", padx=10, pady=2)

        cmd_var = tk.StringVar(value=cmd_str)
        cmd_entry = tk.Entry(entry_row, textvariable=cmd_var, bg=self.colors["entry_bg"], fg=self.colors["fg"], insertbackground="#ffffff", font=("monospace", 9), bd=0, relief="flat")
        cmd_entry.pack(fill="x", side="left", expand=True, ipady=3, padx=(0, 5))
        self.cmd_entries[cmd_id] = cmd_var

        def on_cmd_change(*args):
            handler.cmd_str = cmd_var.get()
        cmd_var.trace_add("write", on_cmd_change)

        btn_row = tk.Frame(card, bg=self.colors["card_bg"])
        btn_row.pack(fill="x", padx=10, pady=(2, 6))

        if desc:
            lbl_desc = tk.Label(btn_row, text=desc, font=("sans-serif", 8, "italic"), fg=self.colors["subtext"], bg=self.colors["card_bg"])
            lbl_desc.pack(side="left")

        def on_start_click():
            if self.is_parking_active and category == "Control":
                self.disable_parking_mode()
            handler.start()

        start_btn = ttk.Button(btn_row, text="Start", style="Start.TButton", command=on_start_click)
        start_btn.pack(side="right", padx=(4, 0))

        stop_btn = ttk.Button(btn_row, text="Stop", style="Stop.TButton", command=lambda: handler.stop())
        stop_btn.pack(side="right", padx=4)

        restart_btn = ttk.Button(btn_row, text="Restart", style="Restart.TButton", command=lambda: (handler.stop(), self.after(500, handler.start)))
        restart_btn.pack(side="right", padx=4)

    def _enqueue_log(self, cmd_id, text):
        self.log_queue.put((cmd_id, text))

    def _process_log_queue(self):
        while not self.log_queue.empty():
            cmd_id, text = self.log_queue.get_nowait()
            timestamp = time.strftime("[%H:%M:%S] ")
            formatted_line = f"{timestamp}{text}"
            
            if cmd_id in self.log_widgets:
                w = self.log_widgets[cmd_id]
                w.insert(tk.END, formatted_line)
                w.see(tk.END)

            self.main_log_text.insert(tk.END, formatted_line)
            self.main_log_text.see(tk.END)

        self.after(100, self._process_log_queue)

    def _update_status_ui(self, cmd_id, status, pid):
        if cmd_id not in self.status_labels:
            return

        lbl = self.status_labels[cmd_id]
        if status == "RUNNING":
            lbl.config(text=f"[RUNNING (PID: {pid})]", fg=self.colors["green"])
        else:
            lbl.config(text="[STOPPED]", fg=self.colors["red"])

        self._update_category_counts()

    def _update_category_counts(self):
        cat_active = {c["id"]: 0 for c in CATEGORIES}
        for h in self.processes.values():
            if h.is_running and h.category in cat_active:
                cat_active[h.category] += 1

        for cat_id, count in cat_active.items():
            if cat_id in self.category_count_labels:
                lbl = self.category_count_labels[cat_id]
                if count > 0:
                    lbl.config(text=f"{count} Active", fg=self.colors["green"])
                else:
                    lbl.config(text="0 Active", fg=self.colors["subtext"])

    def _update_sensor_status_ui(self):
        now = time.time()
        for sensor_name, last_seen in self.sensor_last_seen.items():
            if sensor_name in self.sensor_status_labels:
                lbl = self.sensor_status_labels[sensor_name]
                if now - last_seen < 2.0:
                    lbl.config(text="[ ON ]", bg="#2e7d32", fg="#ffffff")
                else:
                    lbl.config(text="[ OFF ]", bg="#45475a", fg="#a6adc8")

        self.after(500, self._update_sensor_status_ui)

    def _send_ctrl_command_once(self, ctrl_mode, gear, brake=0.0, accel=0.0, steer=0.0):
        cmd = CtrlCmd()
        cmd.ctrl_mode = ctrl_mode
        cmd.gear = gear
        cmd.cmd_type = 1
        cmd.brake = float(brake)
        cmd.accel = float(accel)
        cmd.steer = float(steer)
        cmd.velocity = 0.0
        cmd.acceleration = 0.0

        if self.cmd_pub and self.cmd_pub.impl:
            try:
                self.cmd_pub.publish(cmd)
            except Exception:
                pass

        header = b'#MoraiCtrlCmd$'
        tail = b'\r\n'
        udp_packet = struct.pack(
            '<14si3i3b5f2s',
            header, 23, 0, 0, 0,
            int(ctrl_mode), int(gear), 1,
            0.0, 0.0, float(accel), float(brake), float(steer),
            tail
        )

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(udp_packet, ('127.0.0.1', 9093))
            sock.close()
        except Exception:
            pass

    def set_drive_mode(self, mode_name):
        if mode_name == "PARKING":
            self.enable_parking_mode()
            return

        if self.is_parking_active:
            self.disable_parking_mode()

        self.current_vehicle_mode = mode_name
        self._update_mode_buttons_ui()

        timestamp = time.strftime("[%H:%M:%S] ")
        if mode_name == "MANUAL":
            ctrl_mode = 1  # 1: Keyboard Mode
            gear = 4       # 4: Drive (D)
            log_msg = f"{timestamp}[MODE] Switching to MANUAL Mode (ctrl_mode=1 [Keyboard], gear=4 [D])...\n"
        else:  # AUTO
            ctrl_mode = 2  # 2: Auto Mode
            gear = 4       # 4: Drive (D)
            log_msg = f"{timestamp}[MODE] Switching to AUTO Mode (ctrl_mode=2 [Auto], gear=4 [D])...\n"

        self.main_log_text.insert(tk.END, log_msg)
        self.main_log_text.see(tk.END)

        self._send_ctrl_command_once(ctrl_mode=ctrl_mode, gear=gear, brake=0.0, accel=0.0, steer=0.0)

    def _update_mode_buttons_ui(self):
        if self.is_parking_active:
            self.btn_manual.config(bg=self.colors["button_bg"], fg=self.colors["fg"])
            self.btn_auto.config(bg=self.colors["button_bg"], fg=self.colors["fg"])
            self.btn_parking.config(text="[P] PARKING ACTIVE", bg="#c62828", fg="#ffffff")
        elif self.current_vehicle_mode == "MANUAL":
            self.btn_manual.config(bg=self.colors["accent"], fg="#11111b")
            self.btn_auto.config(bg=self.colors["button_bg"], fg=self.colors["fg"])
            self.btn_parking.config(text="Parking [P]", bg="#e65100", fg="#ffffff")
        elif self.current_vehicle_mode == "AUTO":
            self.btn_manual.config(bg=self.colors["button_bg"], fg=self.colors["fg"])
            self.btn_auto.config(bg=self.colors["green"], fg="#11111b")
            self.btn_parking.config(text="Parking [P]", bg="#e65100", fg="#ffffff")

    def run_one_shot_cmd(self, cmd_str, name):
        timestamp = time.strftime("[%H:%M:%S] ")
        log_msg = f"{timestamp}[OneShot] Executing {name}: {cmd_str}\n"
        self.main_log_text.insert(tk.END, log_msg)
        self.main_log_text.see(tk.END)
        
        full_cmd = get_bash_prefix() + cmd_str
        
        def run_task():
            try:
                subprocess.Popen(full_cmd, shell=True, executable='/bin/bash')
            except Exception as e:
                self.main_log_text.insert(tk.END, f"{timestamp}[OneShot ERROR] {e}\n")
                self.main_log_text.see(tk.END)
                
        threading.Thread(target=run_task, daemon=True).start()

    def toggle_parking_mode(self):
        if not self.is_parking_active:
            self.enable_parking_mode()
        else:
            self.disable_parking_mode()

    def enable_parking_mode(self):
        self.is_parking_active = True
        self.current_vehicle_mode = "PARKING"
        self._update_mode_buttons_ui()
        
        timestamp = time.strftime("[%H:%M:%S] ")
        log_msg = f"{timestamp}[ACTION] Enabling Parking Mode (20Hz -> /cmd & UDP 9093: ctrl_mode=2 [AutoMode], gear=1 [P], brake=1.0)...\n"
        self.main_log_text.insert(tk.END, log_msg)
        self.main_log_text.see(tk.END)

        # Stop Control category nodes if running
        for handler in self.processes.values():
            if handler.category == "Control" and handler.is_running:
                self.main_log_text.insert(tk.END, f"{timestamp}[PARKING] Stopping {handler.name} to prevent command conflicts...\n")
                handler.stop()

        def parking_worker():
            cmd = CtrlCmd()
            cmd.ctrl_mode = 2  # 2: AutoMode (Required for MORAI to process gear=1 [P] and brake=1.0)
            cmd.gear = 1       # 1: Parking (P)
            cmd.cmd_type = 1   # 1: Throttle Mode
            cmd.brake = 1.0    # Apply full brake
            cmd.accel = 0.0
            cmd.steer = 0.0
            cmd.velocity = 0.0
            cmd.acceleration = 0.0

            header = b'#MoraiCtrlCmd$'
            tail = b'\r\n'
            udp_packet = struct.pack(
                '<14si3i3b5f2s',
                header,
                23,
                0, 0, 0,
                2,   # ctrl_mode = 2 (AutoMode)
                1,   # gear = 1 (P)
                1,   # cmd_type = 1
                0.0, # velocity
                0.0, # acceleration
                0.0, # accel
                1.0, # brake
                0.0, # steer
                tail
            )

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            while self.is_parking_active and not rospy.is_shutdown():
                if self.cmd_pub and self.cmd_pub.impl:
                    try:
                        self.cmd_pub.publish(cmd)
                    except Exception:
                        pass

                try:
                    sock.sendto(udp_packet, ('127.0.0.1', 9093))
                except Exception:
                    pass

                time.sleep(0.05)  # 20Hz

            sock.close()

        self.parking_thread = threading.Thread(target=parking_worker, daemon=True)
        self.parking_thread.start()

    def disable_parking_mode(self):
        if self.is_parking_active:
            self.is_parking_active = False
            self.current_vehicle_mode = "MANUAL"
            self._update_mode_buttons_ui()
            timestamp = time.strftime("[%H:%M:%S] ")
            log_msg = f"{timestamp}[ACTION] Released Parking Mode. Switched to Manual Mode.\n"
            self.main_log_text.insert(tk.END, log_msg)
            self.main_log_text.see(tk.END)
            self._send_ctrl_command_once(ctrl_mode=1, gear=4, brake=0.0, accel=0.0, steer=0.0)

    def start_all(self):
        if self.is_parking_active:
            self.disable_parking_mode()
        for cmd_id, handler in self.processes.items():
            handler.start()

    def stop_all(self):
        if self.is_parking_active:
            self.disable_parking_mode()
        for cmd_id, handler in self.processes.items():
            handler.stop()

    def restart_all(self):
        self.stop_all()
        self.after(1000, self.start_all)

    def clear_all_logs(self):
        self.main_log_text.delete("1.0", tk.END)
        for w in self.log_widgets.values():
            w.delete("1.0", tk.END)

    def add_custom_cmd_dialog(self):
        dialog = tk.Toplevel(self)
        dialog.title("Add Custom Command")
        dialog.geometry("480x320")
        dialog.configure(bg=self.colors["bg"])
        dialog.transient(self)
        dialog.grab_set()

        tk.Label(dialog, text="Command Category:", font=("sans-serif", 10, "bold"), fg=self.colors["fg"], bg=self.colors["bg"]).pack(anchor="w", padx=20, pady=(15, 2))
        cat_var = tk.StringVar(value="Control")
        cat_combo = ttk.Combobox(dialog, textvariable=cat_var, values=[c["id"] for c in CATEGORIES], state="readonly")
        cat_combo.pack(fill="x", padx=20, pady=(0, 10))

        tk.Label(dialog, text="Command Name:", font=("sans-serif", 10, "bold"), fg=self.colors["fg"], bg=self.colors["bg"]).pack(anchor="w", padx=20, pady=(5, 2))
        name_entry = tk.Entry(dialog, bg=self.colors["entry_bg"], fg=self.colors["fg"], font=("monospace", 10), insertbackground="#ffffff")
        name_entry.pack(fill="x", padx=20, pady=(0, 10))

        tk.Label(dialog, text="Command Line:", font=("sans-serif", 10, "bold"), fg=self.colors["fg"], bg=self.colors["bg"]).pack(anchor="w", padx=20, pady=(5, 2))
        cmd_entry = tk.Entry(dialog, bg=self.colors["entry_bg"], fg=self.colors["fg"], font=("monospace", 10), insertbackground="#ffffff")
        cmd_entry.pack(fill="x", padx=20, pady=(0, 10))

        tk.Label(dialog, text="Description (Optional):", font=("sans-serif", 10, "bold"), fg=self.colors["fg"], bg=self.colors["bg"]).pack(anchor="w", padx=20, pady=(5, 2))
        desc_entry = tk.Entry(dialog, bg=self.colors["entry_bg"], fg=self.colors["fg"], font=("monospace", 10), insertbackground="#ffffff")
        desc_entry.pack(fill="x", padx=20, pady=(0, 15))

        def on_add():
            category = cat_var.get().strip()
            name = name_entry.get().strip()
            cmd_str = cmd_entry.get().strip()
            desc = desc_entry.get().strip()

            if not name or not cmd_str:
                messagebox.showwarning("Input Error", "Please provide both Name and Command Line.", parent=dialog)
                return

            cmd_id = f"custom_{int(time.time())}"
            self.add_command_card(cmd_id, name, category, cmd_str, desc)
            dialog.destroy()

        btn_frame = tk.Frame(dialog, bg=self.colors["bg"])
        btn_frame.pack(fill="x", padx=20, pady=10)

        ttk.Button(btn_frame, text="Add Command", style="Start.TButton", command=on_add).pack(side="right")
        ttk.Button(btn_frame, text="Cancel", command=dialog.destroy).pack(side="right", padx=10)

    def on_closing(self):
        if self.is_parking_active:
            self.disable_parking_mode()
        if any(h.is_running for h in self.processes.values()):
            if messagebox.askokcancel("Quit Launcher", "Active ROS processes are running. Stop all processes and exit?"):
                self.stop_all()
                self.destroy()
        else:
            self.destroy()

def main():
    app = ROSLauncherGUI()
    app.mainloop()

if __name__ == "__main__":
    main()
