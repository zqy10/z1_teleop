#!/usr/bin/env python3
"""
Communication verification with saved plots (buffer before/after, packager path).

Repo root:
  source /opt/ros/noetic/setup.bash && source ./teleop/ws/devel/setup.bash
  roscore   # separate terminal

  python3 ./teleop_sim/sim_verify_communication.py --mode buffer
  python3 ./teleop_sim/sim_verify_communication.py --mode packager

Figures: teleop_sim/figures/comm_buffer.png , teleop_sim/figures/comm_packager.png
"""

from __future__ import annotations

import argparse
import os
import random
import sys
import threading
import time

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_TELEOP = os.path.join(_REPO, "teleop", "scripts")
if _TELEOP not in sys.path:
    sys.path.insert(0, _TELEOP)

import numpy as np
import rospy
from geometry_msgs.msg import Pose

from teleop_msgs.msg import TeleopFrame

from communication import JitterBufferNode, MasterArmDataPackager

from sim_plotting import configure_matplotlib, figure_dir, save_current_figure


def _run_buffer():
    rospy.init_node("sim_verify_comm_buffer", anonymous=True)
    out_topic = rospy.get_param("~buffer_output_topic", "/comm_verify_buffered")
    depth = int(rospy.get_param("~buffer_depth", 30))
    rospy.set_param("~buffer_depth", depth)

    JitterBufferNode("/network/master_frame", out_topic)

    pub = rospy.Publisher("/network/master_frame", TeleopFrame, queue_size=100)
    lock = threading.Lock()
    injected = []
    buffered = []

    def out_cb(msg: TeleopFrame):
        with lock:
            buffered.append((rospy.Time.now().to_sec(), float(msg.seq_num), float(msg.pose[0])))

    rospy.Subscriber(out_topic, TeleopFrame, out_cb, queue_size=50)

    n = 45
    seq_order = list(range(1, n + 1))
    random.shuffle(seq_order)
    rospy.sleep(0.3)
    t0 = rospy.Time.now().to_sec()
    for s in seq_order:
        f = TeleopFrame()
        f.header.stamp = rospy.Time.now()
        f.seq_num = s
        f.pose = [0.1, 0.2, 0.25, 0.0, 0.0, float(s) * 0.001]
        f.velocity = [0.0] * 6
        f.wrench = [0.0] * 6
        t = rospy.Time.now().to_sec()
        with lock:
            injected.append((t, float(s), float(f.pose[0])))
        pub.publish(f)
        time.sleep(random.uniform(0.0, 0.012))

    rospy.sleep(3.5)

    with lock:
        inj = list(injected)
        buf = list(buffered)

    if len(buf) < n * 0.4:
        rospy.logerr("Too few buffered samples (%s). Raise wait time or lower buffer_depth.", len(buf))
        return

    plt = configure_matplotlib()
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))

    ti = [x[0] - t0 for x in inj]
    si = [x[1] for x in inj]
    xi = [x[2] for x in inj]
    tb = [x[0] - t0 for x in buf]
    sb = [x[1] for x in buf]
    xb = [x[2] for x in buf]

    axes[0, 0].scatter(ti, si, c=si, cmap="viridis", s=12, alpha=0.85)
    axes[0, 0].set_title("Buffer input: seq vs time (arrival order is shuffled)")
    axes[0, 0].set_xlabel("time (s)")
    axes[0, 0].set_ylabel("seq_num")

    axes[0, 1].plot(tb, sb, "b.-", markersize=3)
    axes[0, 1].set_title("Buffer output: seq vs time (should rise with time)")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_ylabel("seq_num")

    axes[1, 0].plot(ti, xi, "C0.-", markersize=3, label="pose[0] at injection")
    axes[1, 0].set_title("Buffer input: pose[0] vs time")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("pose[0] (m)")

    axes[1, 1].plot(tb, xb, "C1.-", markersize=3, label="pose[0] after buffer")
    axes[1, 1].set_title("Buffer output: pose[0] vs time")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].set_ylabel("pose[0] (m)")

    out_path = os.path.join(figure_dir(), "comm_buffer.png")
    save_current_figure(out_path)
    rospy.loginfo("Saved figure: %s", out_path)


def _run_packager():
    rospy.init_node("sim_verify_comm_packager", anonymous=True)
    rospy.set_param("/master/enable_sim", True)
    rospy.set_param("/master/latency_low", 0.06)
    rospy.set_param("/master/latency_high", 0.14)

    MasterArmDataPackager()

    lock = threading.Lock()
    desired = []
    network = []

    def net_cb(msg: TeleopFrame):
        with lock:
            network.append((rospy.Time.now().to_sec(), float(msg.pose[0])))

    rospy.Subscriber("/network/master_frame", TeleopFrame, net_cb, queue_size=50)
    pub = rospy.Publisher("/master/desired_pose", Pose, queue_size=10)

    rospy.sleep(0.5)
    t0 = rospy.Time.now().to_sec()
    for i in range(80):
        p = Pose()
        p.position.x = 0.42 + 0.04 * np.sin(2 * np.pi * 0.25 * (rospy.Time.now().to_sec() - t0))
        p.position.y = 0.0
        p.position.z = 0.35
        p.orientation.w = 1.0
        with lock:
            desired.append((rospy.Time.now().to_sec(), float(p.position.x)))
        pub.publish(p)
        rospy.sleep(0.03)

    rospy.sleep(2.0)

    with lock:
        des = list(desired)
        net = list(network)

    if len(net) < 10:
        rospy.logerr("Too few /network/master_frame samples (%s).", len(net))
        return

    plt = configure_matplotlib()
    fig, ax = plt.subplots(figsize=(11, 5))
    td = [x[0] - t0 for x in des]
    xd = [x[1] for x in des]
    tn = [x[0] - t0 for x in net]
    xn = [x[1] for x in net]
    ax.plot(td, xd, "C0-", linewidth=1.5, label="/master/desired_pose position.x (sent)")
    ax.plot(tn, xn, "C1-", linewidth=1.2, alpha=0.9, label="/network/master_frame pose[0] (received)")
    ax.set_title("Packager + simulated network delay: desired vs network x(t)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("x (m)")
    ax.legend(loc="upper right")

    out_path = os.path.join(figure_dir(), "comm_packager.png")
    save_current_figure(out_path)
    rospy.loginfo("Saved figure: %s", out_path)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=("buffer", "packager"), required=True)
    args = p.parse_args()
    if args.mode == "buffer":
        _run_buffer()
    else:
        _run_packager()


if __name__ == "__main__":
    main()
