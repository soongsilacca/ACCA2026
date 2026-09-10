#!/usr/bin/env python3
"""Interactive global-path and teacher-rule editor."""

import argparse
import csv
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import yaml


class TeacherRouteEditor:
    ACTIONS = ("DRIVE", "STOP", "AVOID")
    MODULES = ("mgeo", "tcp", "rule_avoid", "custom")

    def __init__(self, root, config_path):
        self.root = root
        self.config_path = os.path.abspath(os.path.expanduser(config_path))
        self.config = self.load_yaml(self.config_path)
        self.csv_path = os.path.expanduser(self.config.get(
            "global_path_csv", "/home/acca/acca_ws/global_path/global_path.csv"
        ))
        self.rows, self.xy = self.load_csv(self.csv_path)
        self.s = self.stations(self.xy)
        self.rules = list(self.config.get("rules", []))
        self.start_index = 0
        self.end_index = max(len(self.xy) - 1, 0)
        self.selected_point = None
        self.build_ui()
        self.redraw()

    @staticmethod
    def load_yaml(path):
        if not os.path.exists(path):
            return {"version": 1, "rules": []}
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {"version": 1, "rules": []}

    @staticmethod
    def load_csv(path):
        with open(path, "r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.reader(stream))
        numeric = []
        first_data = 0
        for index, row in enumerate(rows):
            try:
                numeric.append((float(row[0]), float(row[1])))
                first_data = index
                break
            except (ValueError, IndexError):
                continue
        for row in rows[first_data + 1:]:
            try:
                numeric.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                pass
        if len(numeric) < 2:
            raise ValueError("global path CSV needs at least two XY rows")
        return rows, np.asarray(numeric, dtype=float)

    @staticmethod
    def stations(xy):
        return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]

    def build_ui(self):
        self.root.title("Teacher Route Studio")
        self.root.geometry("1500x900")
        left = ttk.Frame(self.root, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.action = tk.StringVar(value="DRIVE")
        self.module = tk.StringVar(value="mgeo")
        self.name = tk.StringVar(value="segment")
        self.speed = tk.StringVar(value="")
        self.offset = tk.StringVar(value="0.0")
        self.point_x = tk.StringVar()
        self.point_y = tk.StringVar()

        for label, variable, values in (
            ("Action", self.action, self.ACTIONS),
            ("Module", self.module, self.MODULES),
        ):
            ttk.Label(left, text=label).pack(anchor="w")
            ttk.Combobox(left, textvariable=variable, values=values,
                         state="normal", width=24).pack(fill=tk.X, pady=(0, 6))
        for label, variable in (("Name", self.name), ("Speed km/h", self.speed),
                                ("Avoid offset m", self.offset)):
            ttk.Label(left, text=label).pack(anchor="w")
            ttk.Entry(left, textvariable=variable).pack(fill=tk.X, pady=(0, 6))

        ttk.Button(left, text="Set segment START (click)",
                   command=lambda: self.set_click_mode("start")).pack(fill=tk.X)
        ttk.Button(left, text="Set segment END (click)",
                   command=lambda: self.set_click_mode("end")).pack(fill=tk.X)
        ttk.Button(left, text="Add / update rule",
                   command=self.add_rule).pack(fill=tk.X, pady=(4, 10))

        self.rule_list = tk.Listbox(left, width=42, height=16)
        self.rule_list.pack(fill=tk.BOTH)
        self.rule_list.bind("<<ListboxSelect>>", self.select_rule)
        ttk.Button(left, text="Delete rule", command=self.delete_rule).pack(fill=tk.X)

        ttk.Separator(left).pack(fill=tk.X, pady=10)
        ttk.Label(left, text="Path point editor (click point)").pack(anchor="w")
        ttk.Entry(left, textvariable=self.point_x).pack(fill=tk.X)
        ttk.Entry(left, textvariable=self.point_y).pack(fill=tk.X)
        ttk.Button(left, text="Apply X/Y", command=self.apply_point).pack(fill=tk.X)
        ttk.Button(left, text="Insert after", command=self.insert_point).pack(fill=tk.X)
        ttk.Button(left, text="Delete point", command=self.delete_point).pack(fill=tk.X)
        ttk.Separator(left).pack(fill=tk.X, pady=10)
        ttk.Button(left, text="Save YAML + CSV", command=self.save).pack(fill=tk.X)
        ttk.Button(left, text="Save as...", command=self.save_as).pack(fill=tk.X)
        self.status = ttk.Label(left, text="Click the map to select a path point")
        self.status.pack(fill=tk.X, pady=8)

        figure = Figure(figsize=(10, 8), dpi=100)
        self.axes = figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.click_mode = "point"

    def set_click_mode(self, mode):
        self.click_mode = mode
        self.status.configure(text="Click path for segment %s" % mode.upper())

    def nearest(self, x, y):
        return int(np.argmin(np.sum((self.xy - [x, y]) ** 2, axis=1)))

    def on_click(self, event):
        if event.xdata is None:
            return
        index = self.nearest(event.xdata, event.ydata)
        if self.click_mode == "start":
            self.start_index = index
            self.click_mode = "point"
        elif self.click_mode == "end":
            self.end_index = index
            self.click_mode = "point"
        else:
            self.selected_point = index
            self.point_x.set("%.6f" % self.xy[index, 0])
            self.point_y.set("%.6f" % self.xy[index, 1])
        self.redraw()

    def add_rule(self):
        start, end = sorted((self.start_index, self.end_index))
        rule = {"name": self.name.get().strip() or "segment",
                "enabled": True, "start_s": float(self.s[start]),
                "end_s": float(self.s[end]), "action": self.action.get(),
                "module": self.module.get().strip() or "custom",
                "speed_kph": None if not self.speed.get().strip() else float(self.speed.get()),
                "lateral_offset_m": float(self.offset.get() or 0.0)}
        selected = self.rule_list.curselection()
        if selected:
            self.rules[selected[0]] = rule
        else:
            self.rules.append(rule)
        self.redraw()

    def select_rule(self, _event=None):
        selected = self.rule_list.curselection()
        if not selected:
            return
        rule = self.rules[selected[0]]
        self.name.set(rule.get("name", "segment")); self.action.set(rule.get("action", "DRIVE"))
        self.module.set(rule.get("module", "mgeo")); self.speed.set("" if rule.get("speed_kph") is None else str(rule["speed_kph"]))
        self.offset.set(str(rule.get("lateral_offset_m", 0.0)))
        self.start_index = int(np.argmin(abs(self.s - float(rule["start_s"]))))
        self.end_index = int(np.argmin(abs(self.s - float(rule["end_s"]))))
        self.redraw()

    def delete_rule(self):
        selected = self.rule_list.curselection()
        if selected:
            del self.rules[selected[0]]
            self.redraw()

    def apply_point(self):
        if self.selected_point is None: return
        self.xy[self.selected_point] = [float(self.point_x.get()), float(self.point_y.get())]
        self.s = self.stations(self.xy); self.redraw()

    def insert_point(self):
        if self.selected_point is None: return
        index = self.selected_point + 1
        prev = self.xy[self.selected_point]
        nxt = self.xy[index] if index < len(self.xy) else prev
        self.xy = np.insert(self.xy, index, (prev + nxt) * 0.5, axis=0)
        self.selected_point = index; self.s = self.stations(self.xy); self.redraw()

    def delete_point(self):
        if self.selected_point is None or len(self.xy) <= 2: return
        self.xy = np.delete(self.xy, self.selected_point, axis=0)
        self.selected_point = None; self.s = self.stations(self.xy); self.redraw()

    def redraw(self):
        self.axes.clear(); self.axes.plot(self.xy[:, 0], self.xy[:, 1], color="#546e7a", lw=2)
        colors = {"DRIVE": "#00c853", "STOP": "#ff1744", "AVOID": "#ff9100"}
        for rule in self.rules:
            mask = (self.s >= float(rule["start_s"])) & (self.s <= float(rule["end_s"]))
            self.axes.plot(self.xy[mask, 0], self.xy[mask, 1], lw=6,
                           color=colors.get(rule.get("action"), "#00b0ff"), alpha=.8)
        self.axes.scatter(*self.xy[self.start_index], c="cyan", s=70)
        self.axes.scatter(*self.xy[self.end_index], c="magenta", s=70)
        if self.selected_point is not None: self.axes.scatter(*self.xy[self.selected_point], c="yellow", s=100)
        self.axes.axis("equal"); self.axes.grid(True, alpha=.25); self.canvas.draw_idle()
        self.rule_list.delete(0, tk.END)
        for rule in self.rules:
            self.rule_list.insert(tk.END, "%s | %s | %s | %.1f-%.1fm" %
                                  (rule["name"], rule["action"], rule["module"], rule["start_s"], rule["end_s"]))

    def write_csv(self, path):
        with open(path, "w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream); writer.writerow(["x", "y"]); writer.writerows(self.xy.tolist())

    def save(self):
        self.write_csv(self.csv_path)
        data = {"version": 1, "global_path_csv": self.csv_path, "rules": self.rules}
        with open(self.config_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
        messagebox.showinfo("Teacher Route Studio", "Saved YAML and global path CSV")

    def save_as(self):
        path = filedialog.asksaveasfilename(defaultextension=".yaml", filetypes=[("YAML", "*.yaml")])
        if path:
            self.config_path = path; self.save()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--config", required=True)
    args, _ = parser.parse_known_args(); root = tk.Tk(); TeacherRouteEditor(root, args.config); root.mainloop()


if __name__ == "__main__":
    main()
