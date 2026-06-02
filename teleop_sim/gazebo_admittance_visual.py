#!/usr/bin/env python3
"""
Gazebo: simulated wrench -> admittance target pose -> visual sphere + IK joint tracking.

Prerequisites (run in order, repo root):
  1. source /opt/ros/noetic/setup.bash && source unitree_ws/devel/setup.bash && source teleop/ws/devel/setup.bash
  2. export Z1_SDK_LIB=.../unitree_ws/src/z1_sdk/lib   (same Python as master_arm)
  3. roslaunch unitree_gazebo z1.launch paused:=false gui:=true
  4. python3 ./teleop_sim/gazebo_admittance_visual.py
     Optional: _plot_duration:=12.0 saves teleop_sim/figures/admittance_gazebo.png and exits.
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import MotorCmd

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SCRIPTS = os.path.join(_REPO, "teleop", "scripts")
_IK_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_IK_DIR, _SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from inverse_kinematics import InverseKinematicsSolver, Z1Kinematics  # noqa: E402

from master_arm import AdmittanceController, WorkspaceLimiter  # noqa: E402

from sim_plotting import configure_matplotlib, figure_dir, save_current_figure  # noqa: E402

PMSM = 0x0A


def _sphere_sdf(model_name: str, radius: float) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>false</static>
    <link name='link'>
      <kinematic>true</kinematic>
      <gravity>false</gravity>
      <collision name='c'><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
      <visual name='v'>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material><ambient>1 0.35 0 1</ambient><diffuse>1 0.45 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _pose6_to_msg(pose6: np.ndarray) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(pose6[0]), float(pose6[1]), float(pose6[2])
    q = tft.quaternion_from_euler(float(pose6[3]), float(pose6[4]), float(pose6[5]))
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q[0], q[1], q[2], q[3]
    return p


class GazeboAdmittanceVisual:
    def __init__(self):
        rospy.init_node("gazebo_admittance_visual", anonymous=False)

        self._plot_duration = float(rospy.get_param("~plot_duration", 0.0))
        self._t0_wall = time.time()
        self._plot_written = False
        self._rec_t = []
        self._rec_eef_z = []
        self._rec_tgt_z = []
        self._rec_fz = []

        self.robot_ns = rospy.get_param("~robot_ns", "z1_gazebo")
        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))
        self.marker_name = rospy.get_param("~marker_model_name", "admittance_target_marker")
        self.sphere_radius = float(rospy.get_param("~sphere_radius", 0.04))
        self.ref_frame = rospy.get_param("~reference_frame", "world")
        self.force_amp = float(rospy.get_param("~force_amp", 12.0))
        self.force_hz = float(rospy.get_param("~force_hz", 0.4))

        dt = 1.0 / self.rate_hz
        mass = float(rospy.get_param("~mass", 0.8))
        damping = float(rospy.get_param("~damping", 25.0))
        stiffness = float(rospy.get_param("~stiffness", 0.0))
        self.adm = AdmittanceController(
            mass, damping, stiffness, dt, max_correction=0.05, max_velocity=0.2, enable_filter=True
        )

        pmin = np.array(rospy.get_param("~workspace_pos_min", [0.22, -0.42, 0.08]), dtype=float)
        pmax = np.array(rospy.get_param("~workspace_pos_max", [0.78, 0.42, 0.62]), dtype=float)
        rpy_max = float(rospy.get_param("~workspace_rpy_abs_max", 1.1))
        self.limiter = WorkspaceLimiter(pmin, pmax, rpy_max, rospy.get_param("~workspace_limit_enable", True))

        kin = Z1Kinematics()
        self.ik = InverseKinematicsSolver(kin, damping=0.02, max_iter=80, tolerance=1e-3)
        self._kin = kin

        self.q = np.zeros(6)
        self._have_js = False
        self._pubs = []
        for j in range(1, 7):
            t = f"/{self.robot_ns}/Joint0{j}_controller/command"
            self._pubs.append(rospy.Publisher(t, MotorCmd, queue_size=1))

        self._pub_eef = rospy.Publisher("~/eef_pose", PoseStamped, queue_size=1)
        self._pub_tgt = rospy.Publisher("~/target_pose", PoseStamped, queue_size=1)

        rospy.Subscriber(f"/{self.robot_ns}/joint_states", JointState, self._js_cb, queue_size=1)

        from gazebo_msgs.srv import SpawnModel, SetModelState
        from gazebo_msgs.msg import ModelState

        self._ModelState = ModelState
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=120.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=120.0)
        self._spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._spawn_marker()

        rospy.Timer(rospy.Duration(dt), self._tick)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "gazebo_admittance_visual: ns=%s marker=%s rate=%.1f Hz (sphere=target, arm tracks via IK); plot_duration=%s",
            self.robot_ns,
            self.marker_name,
            self.rate_hz,
            self._plot_duration,
        )

    def _on_shutdown(self):
        if self._plot_duration <= 0 or self._plot_written or len(self._rec_t) < 5:
            return
        self._save_plot()

    def _save_plot(self):
        if self._plot_written:
            return
        self._plot_written = True
        plt = configure_matplotlib()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        t0 = self._rec_t[0]
        tr = [x - t0 for x in self._rec_t]
        ax1.plot(tr, self._rec_tgt_z, "C1-", linewidth=1.5, label="target z (admittance) [m]")
        ax1.plot(tr, self._rec_eef_z, "C0-", linewidth=1.2, label="eef z (FK) [m]")
        ax1.set_ylabel("z (m)")
        ax1.legend(loc="upper right")
        ax1.set_title("Gazebo: end-effector z vs admittance target z")

        ax2.plot(tr, self._rec_fz, "C2-", linewidth=1.0)
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("Fz (N)")
        ax2.set_title("Simulated force (z)")

        out_path = os.path.join(figure_dir(), "admittance_gazebo.png")
        save_current_figure(out_path)
        rospy.loginfo("Saved figure: %s", out_path)

    def _spawn_marker(self):
        try:
            sdf = _sphere_sdf(self.marker_name, self.sphere_radius)
            self._spawn(
                model_name=self.marker_name,
                model_xml=sdf,
                robot_namespace="",
                initial_pose=Pose(),
                reference_frame=self.ref_frame,
            )
            rospy.loginfo("Spawned Gazebo model '%s'", self.marker_name)
        except Exception as exc:
            rospy.logwarn("Spawn marker '%s' skipped: %s", self.marker_name, exc)

    def _js_cb(self, msg: JointState):
        try:
            mp = {n: p for n, p in zip(msg.name, msg.position)}
            self.q = np.array([mp[f"joint{i}"] for i in range(1, 7)], dtype=float)
            self._have_js = True
        except Exception:
            pass

    def _publish_pose_stamped(self, pub, pose6: np.ndarray, frame_id: str):
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = frame_id
        m.pose = _pose6_to_msg(pose6)
        pub.publish(m)

    def _set_marker_pose(self, pose6: np.ndarray):
        st = self._ModelState()
        st.model_name = self.marker_name
        st.reference_frame = self.ref_frame
        st.pose = _pose6_to_msg(pose6)
        try:
            self._set_state(st)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "set_model_state failed: %s", exc)

    def _tick(self, _evt):
        if not self._have_js:
            return
        t = rospy.Time.now().to_sec()
        cur = self._kin.get_end_effector_pose(self.q)
        force = np.array([0.0, 0.0, self.force_amp * np.sin(2 * np.pi * self.force_hz * t), 0.0, 0.0, 0.0])
        raw = self.adm.compute_target_pose(cur, force)
        tgt = self.limiter.clamp(raw)

        self._publish_pose_stamped(self._pub_eef, cur, self.ref_frame)
        self._publish_pose_stamped(self._pub_tgt, tgt, self.ref_frame)
        self._set_marker_pose(tgt)

        q_cmd = self.ik.solve(tgt, q_guess=self.q)
        for i, pub in enumerate(self._pubs):
            m = MotorCmd()
            m.mode = PMSM
            m.q = float(q_cmd[i])
            m.dq = 0.0
            m.tau = 0.0
            m.Kp = 300.0
            m.Kd = 5.0
            pub.publish(m)

        if self._plot_duration > 0:
            tw = time.time()
            self._rec_t.append(tw)
            self._rec_eef_z.append(float(cur[2]))
            self._rec_tgt_z.append(float(tgt[2]))
            self._rec_fz.append(float(force[2]))
            if not self._plot_written and (tw - self._t0_wall) >= self._plot_duration:
                self._save_plot()
                rospy.signal_shutdown("plot_duration elapsed")


def main():
    GazeboAdmittanceVisual()
    rospy.spin()


if __name__ == "__main__":
    main()
