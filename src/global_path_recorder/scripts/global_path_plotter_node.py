#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path


class GlobalPathPlotterNode:
    def __init__(self):
        rospy.init_node("global_path_plotter_node")

        self.path_topic = rospy.get_param("~path_topic", "/global_path")
        self.update_hz = float(rospy.get_param("~update_hz", 2.0))
        self.save_png = bool(rospy.get_param("~save_png", True))
        self.png_path = os.path.expanduser(
            rospy.get_param("~png_path", "/tmp/global_path_plot.png")
        )
        self.equal_axis = bool(rospy.get_param("~equal_axis", True))

        self.lock = threading.Lock()
        self.xs = []
        self.ys = []
        self.dirty = False

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], "b-", linewidth=1.5)
        self.start_point, = self.ax.plot([], [], "go", markersize=6)
        self.end_point, = self.ax.plot([], [], "ro", markersize=6)
        self.ax.set_title("Global Path")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        if self.equal_axis:
            self.ax.axis("equal")

        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.update_hz, 0.1)),
            self.timer_callback,
        )

        rospy.loginfo(
            "global_path_plotter_node ready. topic=%s png=%s",
            self.path_topic,
            self.png_path if self.save_png else "disabled",
        )

    def path_callback(self, msg):
        xs = [pose.pose.position.x for pose in msg.poses]
        ys = [pose.pose.position.y for pose in msg.poses]

        with self.lock:
            self.xs = xs
            self.ys = ys
            self.dirty = True

    def timer_callback(self, _event):
        with self.lock:
            if not self.dirty:
                return
            xs = list(self.xs)
            ys = list(self.ys)
            self.dirty = False

        self.update_plot(xs, ys)

    def update_plot(self, xs, ys):
        self.line.set_data(xs, ys)

        if xs and ys:
            self.start_point.set_data([xs[0]], [ys[0]])
            self.end_point.set_data([xs[-1]], [ys[-1]])
            self.ax.relim()
            self.ax.autoscale_view()
            if self.equal_axis:
                self.ax.axis("equal")
        else:
            self.start_point.set_data([], [])
            self.end_point.set_data([], [])

        self.ax.set_title("Global Path (%d points)" % len(xs))

        if self.save_png:
            directory = os.path.dirname(self.png_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            self.fig.savefig(self.png_path, dpi=140, bbox_inches="tight")


if __name__ == "__main__":
    try:
        GlobalPathPlotterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
