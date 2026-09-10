#!/usr/bin/env python3
import argparse
import copy
import csv
import heapq
import json
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure
import numpy as np
import yaml


class MapZoneEditor:
    GLOBAL_PLANNERS = ("mgeo", "astar", "hybrid_astar", "dijkstra", "custom")
    LOCAL_PLANNERS = ("none", "dwa", "teb", "tcp", "rule_avoid", "custom")
    CONTROLLERS = ("mpc", "stanley", "pure_pursuit", "custom")
    BEHAVIORS = ("NORMAL", "PARKING", "STOP", "AVOID", "RECOVERY", "CUSTOM")

    def __init__(self, root, link_file, config_file):
        self.root = root
        self.link_file = os.path.abspath(os.path.expanduser(link_file))
        self.config_file = os.path.abspath(os.path.expanduser(config_file))
        with open(self.link_file, "r", encoding="utf-8") as stream:
            raw = json.load(stream)
        self.links = []
        for index, link in enumerate(raw):
            points = np.asarray(link.get("points", []), dtype=float)
            if len(points) >= 2:
                self.links.append({"id": str(link.get("idx", link.get("id", index))),
                                   "from": str(link.get("from_node_idx", "")),
                                   "to": str(link.get("to_node_idx", "")),
                                   "xy": points[:, :2]})
        self.config = self.load_config()
        self.zones = list(self.config.get("policies", self.config.get("zones", [])))
        self.selected_links = set()
        self.polygon = []
        self.trigger_point = None
        self.start = None
        self.goal = None
        self.waypoints = []
        self.preview_path = None
        self.obstacles = None
        self.driven_trajectory = None
        self.undo_stack = []
        self.redo_stack = []
        self.mode = "links"
        self.build_ui()
        self.redraw()

    def load_config(self):
        if not os.path.exists(self.config_file):
            return {"version": 1, "default_planner": "mgeo", "zones": []}
        with open(self.config_file, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {"version": 1, "zones": []}

    def build_ui(self):
        self.root.title("Teacher Map Studio - Planner Zone Editor")
        self.root.geometry("1600x920")
        panel_shell = ttk.Frame(self.root); panel_shell.pack(side=tk.LEFT, fill=tk.Y)
        panel_canvas = tk.Canvas(panel_shell, width=390, highlightthickness=0)
        panel_scroll = ttk.Scrollbar(panel_shell, orient=tk.VERTICAL, command=panel_canvas.yview)
        panel = ttk.Frame(panel_canvas, padding=10)
        panel.bind("<Configure>", lambda _e: panel_canvas.configure(scrollregion=panel_canvas.bbox("all")))
        panel_canvas.create_window((0, 0), window=panel, anchor="nw", width=380)
        panel_canvas.configure(yscrollcommand=panel_scroll.set)
        panel_canvas.pack(side=tk.LEFT, fill=tk.Y); panel_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        canvas_frame = ttk.Frame(self.root); canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.name = tk.StringVar(value="zone")
        self.planner = tk.StringVar(value="astar")
        self.local_planner = tk.StringVar(value="dwa")
        self.controller = tk.StringVar(value="stanley")
        self.behavior = tk.StringVar(value="NORMAL")
        self.geometry_type = tk.StringVar(value="path_segment")
        self.fallback = tk.StringVar(value="mgeo")
        self.speed = tk.StringVar(value="")
        self.priority = tk.StringVar(value="10")
        self.params = tk.StringVar(value='{"heuristic_weight": 1.0, "inflation_radius": 1.0}')
        self.condition = tk.StringVar(value='{"when": "enter", "if": "always"}')
        self.transition_enabled = tk.BooleanVar(value=True)
        self.transition_distance = tk.StringVar(value="0.7")
        self.transition_hysteresis = tk.StringVar(value="0.5")
        self.retries = tk.StringVar(value="3")
        self.on_failure = tk.StringVar(value="stop")
        self.default_planner = tk.StringVar(value=self.config.get("default_planner", "mgeo"))
        self.show_map = tk.BooleanVar(value=True); self.show_policies = tk.BooleanVar(value=True)
        self.show_preview = tk.BooleanVar(value=True); self.show_waypoints = tk.BooleanVar(value=True)
        self.show_obstacles = tk.BooleanVar(value=True); self.show_driven = tk.BooleanVar(value=True)
        ttk.Label(panel, text="LAYERS").pack(anchor="w")
        for label, variable in (("MGeo Map", self.show_map), ("Planner Policies", self.show_policies),
                                ("Preview Path", self.show_preview), ("Start/Goal/Waypoints", self.show_waypoints),
                                ("Obstacles", self.show_obstacles), ("Driven Trajectory", self.show_driven)):
            ttk.Checkbutton(panel, text=label, variable=variable, command=self.redraw).pack(anchor="w")
        ttk.Separator(panel).pack(fill=tk.X, pady=6)
        fields = (("Policy name", self.name), ("Speed m/s (optional)", self.speed),
                  ("Priority", self.priority), ("Planner parameters (JSON)", self.params))
        for label, var in fields:
            ttk.Label(panel, text=label).pack(anchor="w")
            ttk.Entry(panel, textvariable=var, width=42).pack(fill=tk.X, pady=(0, 7))
        for label, var, values in (
            ("Geometry", self.geometry_type, ("zone", "path_segment", "point_trigger")),
            ("Global planner", self.planner, self.GLOBAL_PLANNERS),
            ("Local planner", self.local_planner, self.LOCAL_PLANNERS),
            ("Controller", self.controller, self.CONTROLLERS),
            ("Behavior", self.behavior, self.BEHAVIORS),
            ("Fallback global planner", self.fallback, self.GLOBAL_PLANNERS),
            ("Default planner", self.default_planner, self.GLOBAL_PLANNERS)):
            ttk.Label(panel, text=label).pack(anchor="w")
            ttk.Combobox(panel, textvariable=var, values=values,
                         state="normal", width=38).pack(fill=tk.X, pady=(0, 7))
        ttk.Label(panel, text="Switch condition (JSON)").pack(anchor="w")
        ttk.Entry(panel, textvariable=self.condition, width=42).pack(fill=tk.X, pady=(0, 7))
        ttk.Checkbutton(panel, text="Smooth planner transition",
                        variable=self.transition_enabled).pack(anchor="w")
        for label, variable in (("Transition distance m", self.transition_distance),
                                ("Trajectory hysteresis s", self.transition_hysteresis),
                                ("Failure retries", self.retries),
                                ("On failure", self.on_failure)):
            ttk.Label(panel, text=label).pack(anchor="w")
            ttk.Entry(panel, textvariable=variable).pack(fill=tk.X, pady=(0, 4))
        ttk.Button(panel, text="Select MGeo links", command=lambda: self.set_mode("links")).pack(fill=tk.X)
        ttk.Button(panel, text="Draw polygon", command=lambda: self.set_mode("polygon")).pack(fill=tk.X)
        ttk.Button(panel, text="Set point trigger", command=lambda: self.set_mode("trigger")).pack(fill=tk.X)
        ttk.Button(panel, text="Set START", command=lambda: self.set_mode("start")).pack(fill=tk.X)
        ttk.Button(panel, text="Set GOAL", command=lambda: self.set_mode("goal")).pack(fill=tk.X)
        ttk.Button(panel, text="Add WAYPOINT", command=lambda: self.set_mode("waypoint")).pack(fill=tk.X)
        ttk.Button(panel, text="Preview MGeo graph path", command=self.preview).pack(fill=tk.X)
        ttk.Button(panel, text="Load obstacle XY", command=self.load_obstacles).pack(fill=tk.X)
        ttk.Button(panel, text="Validate collision", command=self.validate_collision).pack(fill=tk.X)
        ttk.Button(panel, text="Load driven trajectory", command=self.load_trajectory).pack(fill=tk.X)
        ttk.Button(panel, text="Clear selection", command=self.clear_selection).pack(fill=tk.X)
        ttk.Button(panel, text="Add / update zone", command=self.add_zone).pack(fill=tk.X, pady=(8, 12))
        self.listbox = tk.Listbox(panel, width=48, height=20); self.listbox.pack(fill=tk.BOTH)
        self.listbox.bind("<<ListboxSelect>>", self.select_zone)
        ttk.Button(panel, text="Delete zone", command=self.delete_zone).pack(fill=tk.X)
        ttk.Button(panel, text="Save planner zones", command=self.save).pack(fill=tk.X, pady=(12, 0))
        ttk.Button(panel, text="Open YAML / JSON", command=self.open_config).pack(fill=tk.X)
        ttk.Button(panel, text="Save as YAML / JSON", command=self.save_as).pack(fill=tk.X)
        row = ttk.Frame(panel); row.pack(fill=tk.X)
        ttk.Button(row, text="Undo", command=self.undo).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text="Redo", command=self.redo).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.status = ttk.Label(panel, text="Link mode: click roads to toggle selection")
        self.status.pack(fill=tk.X, pady=8)
        fig = Figure(figsize=(11, 8), dpi=100); self.ax = fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, canvas_frame).update()
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("scroll_event", self.on_scroll)

    def on_scroll(self, event):
        if event.xdata is None or event.ydata is None:
            return
        scale = 1.0 / 1.25 if event.button == "up" else 1.25
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        x_ratio = (event.xdata - x_min) / max(x_max - x_min, 1.0e-9)
        y_ratio = (event.ydata - y_min) / max(y_max - y_min, 1.0e-9)
        width = (x_max - x_min) * scale
        height = (y_max - y_min) * scale
        self.ax.set_xlim(event.xdata - width * x_ratio,
                         event.xdata + width * (1.0 - x_ratio))
        self.ax.set_ylim(event.ydata - height * y_ratio,
                         event.ydata + height * (1.0 - y_ratio))
        self.canvas.draw_idle()

    def set_mode(self, mode):
        self.mode = mode
        if mode == "polygon": self.polygon = []
        messages = {"links": "Click roads to build a path segment",
                    "polygon": "Click polygon vertices; right click closes",
                    "trigger": "Click a point trigger", "start": "Click preview START",
                    "goal": "Click preview GOAL", "waypoint": "Click preview WAYPOINT"}
        self.status.configure(text=messages.get(mode, mode))
        self.redraw()

    def clear_selection(self):
        self.snapshot(); self.selected_links.clear(); self.polygon = []; self.trigger_point = None; self.redraw()

    def snapshot(self):
        self.undo_stack.append(copy.deepcopy((self.zones, self.selected_links,
                                               self.polygon, self.trigger_point,
                                               self.start, self.goal, self.waypoints)))
        self.undo_stack = self.undo_stack[-50:]; self.redo_stack.clear()

    def restore(self, state):
        (self.zones, self.selected_links, self.polygon, self.trigger_point,
         self.start, self.goal, self.waypoints) = copy.deepcopy(state)
        self.redraw()

    def undo(self):
        if not self.undo_stack: return
        self.redo_stack.append(copy.deepcopy((self.zones, self.selected_links,
                                              self.polygon, self.trigger_point,
                                              self.start, self.goal, self.waypoints)))
        self.restore(self.undo_stack.pop())

    def redo(self):
        if not self.redo_stack: return
        self.undo_stack.append(copy.deepcopy((self.zones, self.selected_links,
                                              self.polygon, self.trigger_point,
                                              self.start, self.goal, self.waypoints)))
        self.restore(self.redo_stack.pop())

    @staticmethod
    def distance_to_polyline(point, xy):
        a, b = xy[:-1], xy[1:]; ab = b - a
        denom = np.sum(ab * ab, axis=1); denom[denom < 1e-9] = 1.0
        t = np.clip(np.sum((point - a) * ab, axis=1) / denom, 0.0, 1.0)
        projection = a + ab * t[:, None]
        return float(np.min(np.linalg.norm(projection - point, axis=1)))

    def on_click(self, event):
        if event.xdata is None: return
        point = np.asarray([event.xdata, event.ydata])
        if self.mode == "links":
            self.snapshot()
            link = min(self.links, key=lambda item: self.distance_to_polyline(point, item["xy"]))
            if link["id"] in self.selected_links: self.selected_links.remove(link["id"])
            else: self.selected_links.add(link["id"])
        elif self.mode == "polygon":
            self.snapshot()
            if event.button == 3 and len(self.polygon) >= 3: self.polygon.append(self.polygon[0])
            else: self.polygon.append([float(point[0]), float(point[1])])
        elif self.mode == "trigger": self.snapshot(); self.trigger_point = point.tolist()
        elif self.mode == "start": self.snapshot(); self.start = point.tolist(); self.mode = "links"
        elif self.mode == "goal": self.snapshot(); self.goal = point.tolist(); self.mode = "links"
        elif self.mode == "waypoint": self.snapshot(); self.waypoints.append(point.tolist())
        self.redraw()

    def add_zone(self):
        try:
            parameters = json.loads(self.params.get() or "{}")
            condition = json.loads(self.condition.get() or "{}")
        except ValueError as error:
            messagebox.showerror("Invalid JSON", str(error)); return
        self.snapshot()
        zone = {"name": self.name.get().strip() or "policy", "enabled": True,
                "geometry_type": self.geometry_type.get(),
                "priority": int(self.priority.get()),
                "global_planner": self.planner.get().strip(),
                "local_planner": self.local_planner.get().strip(),
                "controller": self.controller.get().strip(),
                "behavior": self.behavior.get().strip(),
                "fallback": {"global_planner": self.fallback.get().strip(),
                             "retries": int(self.retries.get()),
                             "on_failure": self.on_failure.get().strip()},
                "speed_mps": None if not self.speed.get().strip() else float(self.speed.get()),
                "link_ids": sorted(self.selected_links), "polygon": self.polygon,
                "trigger": self.trigger_point, "condition": condition,
                "transition": {"enabled": bool(self.transition_enabled.get()),
                               "distance_m": float(self.transition_distance.get()),
                               "hysteresis_sec": float(self.transition_hysteresis.get())},
                "parameters": parameters}
        selected = self.listbox.curselection()
        if selected: self.zones[selected[0]] = zone
        else: self.zones.append(zone)
        self.redraw()

    def select_zone(self, _event=None):
        selected = self.listbox.curselection()
        if not selected: return
        zone = self.zones[selected[0]]
        self.name.set(zone.get("name", "policy")); self.geometry_type.set(zone.get("geometry_type", "zone"))
        self.planner.set(zone.get("global_planner", zone.get("planner", "mgeo")))
        self.local_planner.set(zone.get("local_planner", "none")); self.controller.set(zone.get("controller", "mpc")); self.behavior.set(zone.get("behavior", "NORMAL"))
        fallback = zone.get("fallback", {}); fallback = {"global_planner": fallback} if isinstance(fallback, str) else fallback
        self.fallback.set(fallback.get("global_planner", "mgeo")); self.retries.set(str(fallback.get("retries", 3))); self.on_failure.set(fallback.get("on_failure", "stop")); self.priority.set(str(zone.get("priority", 0)))
        speed = zone.get("speed_mps", zone.get("speed_kph")); self.speed.set("" if speed is None else str(speed))
        self.params.set(json.dumps(zone.get("parameters", {}), ensure_ascii=False))
        self.condition.set(json.dumps(zone.get("condition", {}), ensure_ascii=False))
        transition = zone.get("transition", {}); self.transition_enabled.set(transition.get("enabled", True)); self.transition_distance.set(str(transition.get("distance_m", .7))); self.transition_hysteresis.set(str(transition.get("hysteresis_sec", .5)))
        self.selected_links = set(map(str, zone.get("link_ids", []))); self.polygon = list(zone.get("polygon", [])); self.trigger_point = zone.get("trigger"); self.redraw()

    def delete_zone(self):
        selected = self.listbox.curselection()
        if selected: self.snapshot(); del self.zones[selected[0]]; self.selected_links.clear(); self.polygon = []; self.trigger_point = None; self.redraw()

    def save(self):
        data = {"version": 1, "default_planner": self.default_planner.get().strip(),
                "link_file": self.link_file,
                "preview": {"start": self.start, "goal": self.goal, "waypoints": self.waypoints},
                "policies": sorted(self.zones, key=lambda z: -int(z.get("priority", 0)))}
        with open(self.config_file, "w", encoding="utf-8") as stream:
            if self.config_file.lower().endswith(".json"): json.dump(data, stream, indent=2, ensure_ascii=False)
            else: yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
        messagebox.showinfo("Teacher Map Studio", "Planner zones saved")

    def nearest_node(self, point):
        candidates = []
        for link in self.links:
            candidates.append((float(np.linalg.norm(link["xy"][0] - point)), link["from"]))
            candidates.append((float(np.linalg.norm(link["xy"][-1] - point)), link["to"]))
        return min(candidates)[1]

    def graph_path(self, start, goal):
        adjacency = {}
        for link in self.links:
            length = float(np.linalg.norm(np.diff(link["xy"], axis=0), axis=1).sum())
            adjacency.setdefault(link["from"], []).append((link["to"], length, link))
        queue = [(0.0, start)]; cost = {start: 0.0}; parent = {}
        while queue:
            distance, node = heapq.heappop(queue)
            if node == goal: break
            if distance != cost.get(node): continue
            for nxt, weight, link in adjacency.get(node, []):
                candidate = distance + weight
                if candidate < cost.get(nxt, float("inf")):
                    cost[nxt] = candidate; parent[nxt] = (node, link); heapq.heappush(queue, (candidate, nxt))
        if goal not in parent and goal != start: raise ValueError("No directed MGeo path")
        links = []; node = goal
        while node != start:
            node, link = parent[node]; links.append(link)
        links.reverse(); return np.vstack([link["xy"] for link in links]), cost.get(goal, 0.0)

    def preview(self):
        if self.start is None or self.goal is None:
            messagebox.showwarning("Preview", "Set START and GOAL first"); return
        try:
            points = [self.start] + list(self.waypoints) + [self.goal]
            parts = []; total = 0.0
            for first, second in zip(points[:-1], points[1:]):
                path, length = self.graph_path(self.nearest_node(np.asarray(first)), self.nearest_node(np.asarray(second)))
                parts.append(path); total += length
            self.preview_path = np.vstack(parts)
            self.status.configure(text="Preview path %.2f m | graph search complete" % total)
            self.redraw()
        except ValueError as error:
            self.preview_path = None; self.status.configure(text="PLANNING FAILURE: %s" % error); self.redraw()

    @staticmethod
    def load_xy_file(path):
        if path.lower().endswith(".json"):
            with open(path, "r", encoding="utf-8") as stream: data = json.load(stream)
            if isinstance(data, dict): data = data.get("points", data.get("obstacles", data.get("trajectory", [])))
            return np.asarray(data, dtype=float)[:, :2]
        points = []
        with open(path, "r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            for row in reader: points.append([float(row["x"]), float(row["y"])])
        return np.asarray(points, dtype=float)

    def load_obstacles(self):
        path = filedialog.askopenfilename(filetypes=(("XY data", "*.csv *.json"),))
        if path: self.obstacles = self.load_xy_file(path); self.redraw()

    def load_trajectory(self):
        path = filedialog.askopenfilename(filetypes=(("XY trajectory", "*.csv *.json"),))
        if path: self.driven_trajectory = self.load_xy_file(path); self.redraw()

    def validate_collision(self):
        if self.preview_path is None or self.obstacles is None or len(self.obstacles) == 0:
            self.status.configure(text="Collision check needs Preview Path and obstacle XY"); return
        clearance = float(min(np.min(np.linalg.norm(self.obstacles - point, axis=1)) for point in self.preview_path))
        footprint_radius = float(json.loads(self.params.get() or "{}").get("footprint_radius_m", 1.0))
        result = "COLLISION" if clearance < footprint_radius else "CLEAR"
        self.status.configure(text="%s | min clearance %.3f m | footprint %.3f m" % (result, clearance, footprint_radius))

    def save_as(self):
        path = filedialog.asksaveasfilename(defaultextension=".yaml", filetypes=(("YAML", "*.yaml"), ("JSON", "*.json")))
        if path: self.config_file = path; self.save()

    def open_config(self):
        path = filedialog.askopenfilename(filetypes=(("Policy", "*.yaml *.yml *.json"),))
        if not path: return
        self.snapshot(); self.config_file = path
        with open(path, "r", encoding="utf-8") as stream:
            data = json.load(stream) if path.lower().endswith(".json") else (yaml.safe_load(stream) or {})
        self.default_planner.set(data.get("default_planner", "mgeo")); self.zones = list(data.get("policies", data.get("zones", [])))
        preview = data.get("preview", {}); self.start = preview.get("start"); self.goal = preview.get("goal"); self.waypoints = preview.get("waypoints", []); self.redraw()

    def redraw(self):
        self.ax.clear()
        selected = [link["xy"] for link in self.links if link["id"] in self.selected_links]
        normal = [link["xy"] for link in self.links if link["id"] not in self.selected_links]
        if self.show_map.get(): self.ax.add_collection(LineCollection(normal, colors="#52616b", linewidths=.7, alpha=.55))
        if selected and self.show_policies.get(): self.ax.add_collection(LineCollection(selected, colors="#00e5ff", linewidths=3.2))
        palette = {"mgeo": "#00c853", "astar": "#ff9100", "hybrid_astar": "#ff1744", "dijkstra": "#ffd600", "tcp": "#d500f9"}
        for zone in self.zones if self.show_policies.get() else []:
            polygon = np.asarray(zone.get("polygon", []), dtype=float)
            if len(polygon) >= 3:
                self.ax.fill(polygon[:, 0], polygon[:, 1], color=palette.get(zone.get("global_planner", zone.get("planner")), "#2979ff"), alpha=.2)
            trigger = zone.get("trigger")
            if trigger is not None: self.ax.scatter(trigger[0], trigger[1], marker="*", s=120, color="#ff1744")
        if self.polygon:
            poly = np.asarray(self.polygon); self.ax.plot(poly[:, 0], poly[:, 1], "o-", color="#ff1744")
        if self.trigger_point is not None: self.ax.scatter(*self.trigger_point, marker="*", s=160, color="#ff1744")
        if self.start is not None and self.show_waypoints.get(): self.ax.scatter(*self.start, marker="o", s=120, color="#00e676", label="START")
        if self.goal is not None and self.show_waypoints.get(): self.ax.scatter(*self.goal, marker="X", s=130, color="#ff1744", label="GOAL")
        if self.waypoints and self.show_waypoints.get():
            waypoint = np.asarray(self.waypoints); self.ax.scatter(waypoint[:,0], waypoint[:,1], marker="D", s=70, color="#40c4ff", label="WAYPOINT")
        if self.preview_path is not None and self.show_preview.get(): self.ax.plot(self.preview_path[:,0], self.preview_path[:,1], color="#ffffff", lw=4, alpha=.9, label="PREVIEW")
        if self.obstacles is not None and self.show_obstacles.get(): self.ax.scatter(self.obstacles[:,0], self.obstacles[:,1], s=18, color="#ff1744", alpha=.8, label="OBSTACLE")
        if self.driven_trajectory is not None and self.show_driven.get(): self.ax.plot(self.driven_trajectory[:,0], self.driven_trajectory[:,1], color="#00e5ff", lw=2, alpha=.8, label="ACTUAL")
        all_xy = np.vstack([link["xy"] for link in self.links]); self.ax.set_xlim(all_xy[:,0].min(), all_xy[:,0].max()); self.ax.set_ylim(all_xy[:,1].min(), all_xy[:,1].max())
        self.ax.set_aspect("equal"); self.ax.grid(True, alpha=.15); self.canvas.draw_idle()
        self.listbox.delete(0, tk.END)
        for zone in self.zones:
            self.listbox.insert(tk.END, "%s | %s/%s | %s | P%s" %
                                (zone["name"], zone.get("global_planner", zone.get("planner", "mgeo")),
                                 zone.get("local_planner", "none"), zone.get("behavior", "NORMAL"), zone.get("priority", 0)))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--link-file", required=True); parser.add_argument("--config", required=True)
    args, _ = parser.parse_known_args(); root = tk.Tk(); MapZoneEditor(root, args.link_file, args.config); root.mainloop()
if __name__ == "__main__": main()
