#!/usr/bin/env python3
"""
Symmetric Z1 teleop arm (master or slave).

  python3 master_arm.py
  python3 slave_arm.py   # thin wrapper calling teleop_main(\"slave\")

ROS params (shared, private ~ unless noted):
  /{master|slave}/enable_sim — sim vs real state source
  ~enable_force_feedback — bilateral wrench (see _admittance_force)
  ~workspace_limit_enable, ~workspace_pos_min/max, ~workspace_rpy_abs_max
  ~mass, ~damping, ~stiffness, ~control_dt
  ~z1_mode, ~z1_ip — motion SDK (real / simulation)
  ~command_arm_in_sim — master only: also call motion in sim (default false)
  ~force_sensor_topic — Wrench topic for this side (default /force_sensor; use distinct names if one roscore + two PCs)

Master-specific: /master/arm_ip, topics under /master_arm/...
Slave-specific: /slave/arm_ip, /slave/teleop_frame, /slave_sim/ or /slave/...
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Optional

import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, WrenchStamped, Wrench
from scipy.signal import butter, lfilter
from teleop_msgs.msg import TeleopFrame


def _prepend_sdk_path():
    p = rospy.get_param("~sdk_lib_path", os.environ.get("Z1_SDK_LIB", ""))
    if p and p not in sys.path:
        sys.path.insert(0, p)


class AdmittanceController:
    def __init__(
        self,
        mass=1.0,
        damping=20.0,
        stiffness=0.0,
        dt=0.02,
        max_correction=0.05,
        max_velocity=0.2,
        enable_filter=True,
    ):
        self.M = mass * np.eye(6)
        self.B = damping * np.eye(6)
        self.K = stiffness * np.eye(6)
        self.M_inv = np.linalg.pinv(self.M)
        self.dt = dt
        self.max_correction = max_correction
        self.max_velocity = max_velocity
        self.dX = np.zeros(6)
        self.dX_dot = np.zeros(6)
        self.dX_ddot = np.zeros(6)
        self.enable_filter = enable_filter
        if enable_filter:
            self._init_filter(cutoff=10.0, fs=1.0 / dt, order=2)

    def _init_filter(self, cutoff, fs, order=2):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype="low")
        self.filter_state = None

    def _apply_filter(self, signal):
        if not self.enable_filter or self.filter_state is None:
            return signal
        filtered, self.filter_state = lfilter(self.b, self.a, signal, zi=self.filter_state, axis=0)
        return filtered

    def compute_correction(self, force):
        force_f = self._apply_filter(force)
        correction_force = force_f - self.B @ self.dX_dot - self.K @ self.dX
        self.dX_ddot = self.M_inv @ correction_force
        self.dX_dot += self.dX_ddot * self.dt
        self.dX += self.dX_dot * self.dt
        self.dX = np.clip(self.dX, -self.max_correction, self.max_correction)
        self.dX_dot = np.clip(self.dX_dot, -self.max_velocity, self.max_velocity)
        return self.dX.copy()

    def compute_target_pose(self, desired_pose, force):
        correction = self.compute_correction(force)
        return desired_pose + correction


class ForceSensorTransform:
    def __init__(self):
        self.R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        self.T = np.eye(6)
        self.T[:3, :3] = self.R
        self.T[3:, 3:] = self.R

    def transform(self, wrench):
        return self.T @ wrench


class WorkspaceLimiter:
    """Clamp Cartesian pose [x,y,z,rx,ry,rz] to a box (position) and symmetric RPY bounds."""

    def __init__(self, pos_min: np.ndarray, pos_max: np.ndarray, rpy_abs_max: Optional[float], enabled: bool):
        self.pos_min = np.array(pos_min, dtype=float)
        self.pos_max = np.array(pos_max, dtype=float)
        self.rpy_abs_max = rpy_abs_max
        self.enabled = enabled

    def clamp(self, pose6: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return pose6
        out = pose6.copy()
        out[:3] = np.clip(out[:3], self.pos_min, self.pos_max)
        if self.rpy_abs_max is not None and self.rpy_abs_max > 0:
            out[3:] = np.clip(out[3:], -self.rpy_abs_max, self.rpy_abs_max)
        return out


def euler_to_quat(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def quat_to_euler(q):
    x, y, z, w = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return roll, pitch, yaw


class Z1MotionInterface:
    """Cartesian motion via ``unitree_arm_sdk`` (real) or first-order hold (simulation)."""

    def __init__(self, mode="real", arm_ip="192.168.120.110"):
        self.mode = mode
        self.arm = None
        if mode == "real":
            _prepend_sdk_path()
            try:
                import unitree_arm_sdk as arm_sdk

                self.arm = arm_sdk.unitreeArm()
                self.arm.SetArmIp(arm_ip)
                self.arm.Init()
                self.arm.Enable()
                rospy.loginfo("Z1 motion SDK connected, IP: %s", arm_ip)
            except Exception as exc:
                rospy.logerr("Z1 motion SDK init failed: %s", exc)
                self.mode = "simulation"
                rospy.logwarn("Falling back to simulation motion model")

        if self.mode == "simulation":
            self.current_pose = np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0])
            rospy.loginfo("Z1 motion interface in simulation mode")

    def connect(self):
        return True

    def disconnect(self):
        if self.mode == "real" and self.arm is not None:
            self.arm.Disable()

    def back_to_start(self):
        if self.mode == "real" and self.arm is not None:
            rospy.loginfo("Arm moving to home posture...")
            self.arm.backToStart()
            rospy.loginfo("Home posture reached")
        else:
            self.current_pose = np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0])
            rospy.loginfo("Sim motion: pose reset")

    def move_cartesian(self, pose, vel=0.5, acc=None):
        if self.mode == "real" and self.arm is not None:
            posture = [pose[3], pose[4], pose[5], pose[0], pose[1], pose[2]]
            self.arm.MoveJ(posture, vel)
        else:
            alpha = 0.25
            self.current_pose = (1 - alpha) * self.current_pose + alpha * np.array(pose)
            rospy.logdebug("Sim move toward %s", pose[:3])
            time.sleep(0.02)


class TeleopZ1Arm:
    """Single implementation for master and slave (symmetric control; role sets topics / references)."""

    def __init__(self, side: str):
        if side not in ("master", "slave"):
            raise ValueError("side must be 'master' or 'slave'")
        self.side = side
        self._ns = "/master" if side == "master" else "/slave"
        _prepend_sdk_path()

        self.enable_sim = rospy.get_param(f"{self._ns}/enable_sim", True)
        self.enable_force_feedback = rospy.get_param("~enable_force_feedback", False)

        mass = rospy.get_param("~mass", 1.0)
        damping = rospy.get_param("~damping", 20.0)
        stiffness = rospy.get_param("~stiffness", 0.0)
        self.dt = rospy.get_param("~control_dt", 0.02)
        self.admittance = AdmittanceController(mass, damping, stiffness, self.dt)
        self.force_transform = ForceSensorTransform()

        ws_en = rospy.get_param("~workspace_limit_enable", True)
        pmin = np.array(rospy.get_param("~workspace_pos_min", [0.22, -0.42, 0.08]), dtype=float)
        pmax = np.array(rospy.get_param("~workspace_pos_max", [0.78, 0.42, 0.62]), dtype=float)
        rpy_max = float(rospy.get_param("~workspace_rpy_abs_max", 1.35))
        self.limiter = WorkspaceLimiter(pmin, pmax, rpy_max, ws_en)

        z1_mode = rospy.get_param("~z1_mode", "real")
        default_ip = (
            "192.168.123.110"
            if side == "master"
            else rospy.get_param("/slave/arm_ip", "192.168.120.110")
        )
        z1_ip = rospy.get_param("~z1_ip", rospy.get_param(f"{self._ns}/arm_ip", default_ip))
        self.motion = Z1MotionInterface(mode=z1_mode, arm_ip=z1_ip)
        self.motion.connect()

        rospy.loginfo("%s arm homing...", side)
        self.motion.back_to_start()
        rospy.sleep(3.0)
        rospy.loginfo("%s arm homing done", side)

        self.command_arm_in_sim = rospy.get_param("~command_arm_in_sim", False)
        self._should_command_arm = (not self.enable_sim) or (side == "slave") or (
            side == "master" and self.command_arm_in_sim
        )

        self.current_pose = np.zeros(6)
        self.latest_wrench = Wrench()
        self._teleop_lock = threading.Lock()
        self._teleop_frame: Optional[TeleopFrame] = None

        self._setup_topics_and_io()

        self.rate = rospy.Rate(int(1.0 / self.dt))
        rospy.loginfo("%s admittance loop dt=%s force_feedback=%s workspace=%s", side, self.dt, self.enable_force_feedback, ws_en)

    def _setup_topics_and_io(self):
        if self.side == "master":
            self._out_pose_topic = "/master/desired_pose"
            self._teleop_sub_topic = "/master/teleop_frame"
            self._arm_topic_prefix = "/master_arm"
        else:
            self._out_pose_topic = "/slave/target_pose"
            self._teleop_sub_topic = "/slave/teleop_frame"
            self._arm_topic_prefix = "/slave_sim" if self.enable_sim else "/slave"

        self.desired_pub = rospy.Publisher(self._out_pose_topic, Pose, queue_size=10)
        self.pose_pub = rospy.Publisher(f"{self._arm_topic_prefix}/tool_pose", PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher(f"{self._arm_topic_prefix}/tool_velocity", TwistStamped, queue_size=10)
        self.wrench_pub = rospy.Publisher(f"{self._arm_topic_prefix}/wrench", WrenchStamped, queue_size=10)

        rospy.Subscriber(self._teleop_sub_topic, TeleopFrame, self._teleop_cb, queue_size=10)
        force_topic = rospy.get_param("~force_sensor_topic", "/force_sensor")
        rospy.Subscriber(force_topic, Wrench, self._force_sensor_cb, queue_size=10)
        rospy.loginfo("%s: subscribing force on %s", self.side, force_topic)

        self.enable_pose = rospy.get_param("~enable_pose", True)
        self.enable_velocity = rospy.get_param("~enable_velocity", True)
        self.enable_wrench = rospy.get_param("~enable_wrench", True)
        self.k_base = rospy.get_param("~k_base", 1.0)
        self.T_base = rospy.get_param("~T_base", 0.5)
        self.pub_freq = rospy.get_param("~f_pub", 100)
        self.step = 0

        if self.enable_sim:
            rospy.Timer(rospy.Duration(1.0 / self.pub_freq), self._sim_timer_cb)
            rospy.loginfo("%s: simulation state on %s/*", self.side, self._arm_topic_prefix.strip("/"))
        else:
            self.last_pos = None
            self.last_time = rospy.Time.now()
            self.rate_hz = rospy.get_param("~state_pub_hz", 100)
            import tf.transformations as tft

            arm_ip = rospy.get_param(f"{self._ns}/arm_ip", "192.168.120.110")
            import unitree_arm_interface

            self.arm = unitree_arm_interface.UnitreeArm(arm_ip, 8882)
            self.arm.init()
            self._tf_quat_from_matrix = tft.quaternion_from_matrix
            self.state_thread = threading.Thread(target=self._real_state_loop, daemon=True)
            self.state_thread.start()
            rospy.loginfo("%s: real hardware state (%s)", self.side, arm_ip)

    def _force_sensor_cb(self, msg):
        self.latest_wrench = msg

    def _teleop_cb(self, msg: TeleopFrame):
        with self._teleop_lock:
            self._teleop_frame = msg

    def _wrench_to6(self, w: Wrench) -> np.ndarray:
        return np.array(
            [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], dtype=float
        )

    def _admittance_force(self) -> np.ndarray:
        wl = self._wrench_to6(self.latest_wrench)
        fl = self.force_transform.transform(wl)
        with self._teleop_lock:
            tfm = self._teleop_frame
        if self.side == "slave":
            if self.enable_force_feedback and tfm is not None:
                wr = np.array(tfm.wrench, dtype=float)
                fr = self.force_transform.transform(wr)
                return fr - fl
            # Unilateral: admit only from local contact wrench (legacy enable_force behavior).
            return fl
        # master
        if self.enable_force_feedback and tfm is not None:
            wr = np.array(tfm.wrench, dtype=float)
            fr = self.force_transform.transform(wr)
            return fl - fr
        return fl

    def _reference_pose(self) -> np.ndarray:
        if self.side == "slave":
            with self._teleop_lock:
                tfm = self._teleop_frame
            if tfm is not None:
                return np.array(tfm.pose, dtype=float)
            return self.current_pose.copy()
        return self.current_pose.copy()

    def _sim_timer_cb(self, _event):
        self.step += 1
        t = self.step * 0.01
        x = self.k_base * math.sin(2 * math.pi * self.T_base * t)
        yaw = 0.5 * math.sin(2 * math.pi * 0.3 * t)
        pitch = 0.2 * math.sin(2 * math.pi * 0.5 * t)
        roll = 0.1 * math.sin(2 * math.pi * 0.7 * t)

        self.current_pose = np.array([0.5 + x, 0.0, 0.3, roll, pitch, yaw])
        if self.enable_pose:
            p_msg = PoseStamped()
            p_msg.header.stamp = rospy.Time.now()
            p_msg.header.frame_id = "base_link"
            p_msg.pose.position.x = float(self.current_pose[0])
            p_msg.pose.position.y = float(self.current_pose[1])
            p_msg.pose.position.z = float(self.current_pose[2])
            q = tf_trans.quaternion_from_euler(roll, pitch, yaw)
            p_msg.pose.orientation.x = q[0]
            p_msg.pose.orientation.y = q[1]
            p_msg.pose.orientation.z = q[2]
            p_msg.pose.orientation.w = q[3]
            self.pose_pub.publish(p_msg)

        if self.enable_velocity:
            v_msg = TwistStamped()
            v_msg.header.stamp = rospy.Time.now()
            v_msg.header.frame_id = "base_link"
            v_msg.twist.linear.x = self.k_base * 2 * math.pi * self.T_base * math.cos(2 * math.pi * self.T_base * t)
            self.vel_pub.publish(v_msg)

        if self.enable_wrench:
            w_msg = WrenchStamped()
            w_msg.header.stamp = rospy.Time.now()
            w_msg.header.frame_id = "tool_link"
            w_msg.wrench.force.z = x * 10.0
            self.latest_wrench = w_msg.wrench
            self.wrench_pub.publish(w_msg)

    def _real_state_loop(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.arm.sendRecv()
            current_time = rospy.Time.now()
            T_matrix = np.array(self.arm.armState.getF_T_EE())

            quat = self._tf_quat_from_matrix(T_matrix)
            roll, pitch, yaw = quat_to_euler([quat[0], quat[1], quat[2], quat[3]])
            self.current_pose = np.array(
                [T_matrix[0, 3], T_matrix[1, 3], T_matrix[2, 3], roll, pitch, yaw]
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = current_time
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = T_matrix[0, 3]
            pose_msg.pose.position.y = T_matrix[1, 3]
            pose_msg.pose.position.z = T_matrix[2, 3]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(pose_msg)

            vel_msg = TwistStamped()
            vel_msg.header.stamp = current_time
            vel_msg.header.frame_id = "base_link"
            current_pos = np.array([T_matrix[0, 3], T_matrix[1, 3], T_matrix[2, 3]])
            if self.last_pos is not None:
                dt = (current_time - self.last_time).to_sec()
                if dt > 0:
                    linear_vel = (current_pos - self.last_pos) / dt
                    vel_msg.twist.linear.x = linear_vel[0]
                    vel_msg.twist.linear.y = linear_vel[1]
                    vel_msg.twist.linear.z = linear_vel[2]
            self.last_pos = current_pos
            self.last_time = current_time
            self.vel_pub.publish(vel_msg)

            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = current_time
            wrench_msg.header.frame_id = "tool_link"
            wrench_msg.wrench = self.latest_wrench
            self.wrench_pub.publish(wrench_msg)
            r.sleep()

    def _publish_pose6(self, topic_pose: Pose, pose6: np.ndarray):
        topic_pose.position.x = float(pose6[0])
        topic_pose.position.y = float(pose6[1])
        topic_pose.position.z = float(pose6[2])
        q = euler_to_quat(pose6[3], pose6[4], pose6[5])
        topic_pose.orientation.x = q[0]
        topic_pose.orientation.y = q[1]
        topic_pose.orientation.z = q[2]
        topic_pose.orientation.w = q[3]

    def _has_slave_reference(self) -> bool:
        with self._teleop_lock:
            return self._teleop_frame is not None

    def run(self):
        out_msg = Pose()
        while not rospy.is_shutdown():
            if self.side == "slave" and not self._has_slave_reference():
                self.rate.sleep()
                continue
            ref = self._reference_pose()
            force = self._admittance_force()
            target = self.admittance.compute_target_pose(ref, force)
            target = self.limiter.clamp(target)
            self._publish_pose6(out_msg, target)
            self.desired_pub.publish(out_msg)
            if self._should_command_arm:
                self.motion.move_cartesian(target)
            self.rate.sleep()


def teleop_main(side: str):
    node_name = "master_arm" if side == "master" else "slave_arm"
    rospy.init_node(node_name, anonymous=False)
    TeleopZ1Arm(side).run()


def main():
    teleop_main("master")


if __name__ == "__main__":
    main()
