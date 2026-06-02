#!/usr/bin/env python3
"""
Offline admittance verification: simulated force -> target pose, save time-series plot.

Repo root (needs roscore for rospy):
  source /opt/ros/noetic/setup.bash && source ./teleop/ws/devel/setup.bash
  roscore
  python3 ./teleop_sim/sim_verify_admittance.py

Figure: teleop_sim/figures/admittance_offline.png
"""

from __future__ import annotations

import os
import sys

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SCRIPTS = os.path.join(_REPO, "teleop", "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import numpy as np
import rospy

from master_arm import AdmittanceController, WorkspaceLimiter

from sim_plotting import configure_matplotlib, figure_dir, save_current_figure


def main():
    rospy.init_node("sim_verify_admittance_offline", anonymous=False)
    rate_hz = float(rospy.get_param("~rate_hz", 100.0))
    dt = 1.0 / rate_hz
    duration = float(rospy.get_param("~duration", 4.0))

    mass = float(rospy.get_param("~mass", 1.0))
    damping = float(rospy.get_param("~damping", 20.0))
    stiffness = float(rospy.get_param("~stiffness", 0.0))

    adm = AdmittanceController(mass, damping, stiffness, dt)
    pmin = np.array(rospy.get_param("~workspace_pos_min", [0.22, -0.42, 0.08]), dtype=float)
    pmax = np.array(rospy.get_param("~workspace_pos_max", [0.78, 0.42, 0.62]), dtype=float)
    rpy_max = float(rospy.get_param("~workspace_rpy_abs_max", 1.35))
    lim = WorkspaceLimiter(pmin, pmax, rpy_max, rospy.get_param("~workspace_limit_enable", True))

    current = np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0], dtype=float)
    f_amp = float(rospy.get_param("~force_amp", 10.0))
    f_hz = float(rospy.get_param("~force_hz", 0.5))

    t_list, fz_list, tz_list, tx_list = [], [], [], []
    t0 = rospy.Time.now().to_sec()
    r = rospy.Rate(rate_hz)
    while rospy.Time.now().to_sec() - t0 < duration and not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        fz = f_amp * np.sin(2 * np.pi * f_hz * t)
        force = np.array([0.0, 0.0, fz, 0.0, 0.0, 0.0])
        tgt = lim.clamp(adm.compute_target_pose(current, force))
        t_list.append(t)
        fz_list.append(fz)
        tz_list.append(float(tgt[2]))
        tx_list.append(float(tgt[0]))
        r.sleep()

    plt = configure_matplotlib()
    _, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(t_list, fz_list, "C0-", linewidth=1.2)
    axes[0].set_ylabel("simulated Fz (N)")
    axes[0].set_title("Offline admittance: input force")

    axes[1].plot(t_list, tz_list, "C1-", linewidth=1.5, label="target z (m)")
    axes[1].plot(t_list, tx_list, "C2--", linewidth=1.2, alpha=0.85, label="target x (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("pose (m)")
    axes[1].set_title("Admittance output (clamped target pose)")
    axes[1].legend(loc="upper right")

    out_path = os.path.join(figure_dir(), "admittance_offline.png")
    save_current_figure(out_path)
    rospy.loginfo("Saved figure: %s", out_path)


if __name__ == "__main__":
    main()
