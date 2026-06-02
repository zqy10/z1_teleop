#!/usr/bin/env python3
"""
Teleoperation networking: frame packing and jitter buffers.

Run on the master PC:
  python3 communication.py --role master

Run on the slave PC:
  python3 communication.py --role slave

Requires a sourced Catkin workspace that provides ``teleop_msgs`` (e.g. ``teleop/ws/devel/setup.bash``).
"""

from __future__ import annotations

import argparse
import heapq
import random
import threading

import rospy
import numpy as np
import tf.transformations as tf_trans
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, WrenchStamped

from teleop_msgs.msg import TeleopFrame


class MasterArmDataPackager:
    """Packages /master/desired_pose + /master_arm/wrench into /network/master_frame."""

    def __init__(self):
        self.state_lock = threading.Lock()
        self.desired_pose = np.zeros(6)
        self.master_wrench = np.zeros(6)
        self.seq_counter = 0
        self.enable_sim = rospy.get_param("/master/enable_sim", True)
        self.latency_low = float(
            rospy.get_param("/master/latency_low", rospy.get_param("latency_low", 0.01))
        )
        self.latency_high = float(
            rospy.get_param("/master/latency_high", rospy.get_param("latency_high", 0.2))
        )
        wrench_topic = rospy.get_param("~master_wrench_topic", "/master_arm/wrench")

        rospy.Subscriber("/master/desired_pose", Pose, self.desired_pose_cb)
        rospy.Subscriber(wrench_topic, WrenchStamped, self.master_wrench_cb)
        self.net_pub = rospy.Publisher("/network/master_frame", TeleopFrame, queue_size=50)
        rospy.Timer(rospy.Duration(0.01), self.pack_and_send_cb)

    def desired_pose_cb(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rx, ry, rz = tf_trans.euler_from_quaternion(q)
        with self.state_lock:
            self.desired_pose = np.array(
                [msg.position.x, msg.position.y, msg.position.z, rx, ry, rz]
            )

    def master_wrench_cb(self, msg):
        with self.state_lock:
            self.master_wrench = np.array(
                [
                    msg.wrench.force.x,
                    msg.wrench.force.y,
                    msg.wrench.force.z,
                    msg.wrench.torque.x,
                    msg.wrench.torque.y,
                    msg.wrench.torque.z,
                ]
            )

    def pack_and_send_cb(self, _event):
        self.seq_counter += 1
        frame = TeleopFrame()
        frame.header.stamp = rospy.Time.now()
        frame.seq_num = self.seq_counter
        with self.state_lock:
            frame.pose = self.desired_pose.tolist()
            frame.velocity = [0.0] * 6
            frame.wrench = self.master_wrench.tolist()

        if self.enable_sim:
            delay = random.uniform(self.latency_low, self.latency_high)
            threading.Timer(delay, self._delayed_publish, args=(frame,)).start()
        else:
            self.net_pub.publish(frame)

    def _delayed_publish(self, frame_msg):
        self.net_pub.publish(frame_msg)


class SlaveArmDataPackager:
    """Packages slave arm state into /network/slave_frame."""

    def __init__(self):
        self.state_lock = threading.Lock()
        self.pose = np.zeros(6)
        self.velocity = np.zeros(6)
        self.wrench = np.zeros(6)
        self.seq_counter = 0
        self.enable_sim = rospy.get_param("/slave/enable_sim", True)
        self.latency_low = float(rospy.get_param("/slave/latency_low", 0.01))
        self.latency_high = float(rospy.get_param("/slave/latency_high", 0.2))

        if self.enable_sim:
            rospy.Subscriber("/slave_sim/tool_pose", PoseStamped, self.pose_cb)
            rospy.Subscriber("/slave_sim/tool_velocity", TwistStamped, self.vel_cb)
            rospy.Subscriber("/slave_sim/wrench", WrenchStamped, self.wrench_cb)
        else:
            rospy.Subscriber("/slave/tool_pose", PoseStamped, self.pose_cb)
            rospy.Subscriber("/slave/tool_velocity", TwistStamped, self.vel_cb)
            rospy.Subscriber("/slave/wrench", WrenchStamped, self.wrench_cb)

        self.net_pub = rospy.Publisher("/network/slave_frame", TeleopFrame, queue_size=50)
        rospy.Timer(rospy.Duration(0.01), self.pack_and_send_cb)

    def pose_cb(self, msg):
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        rx, ry, rz = tf_trans.euler_from_quaternion(q)
        with self.state_lock:
            self.pose = np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, rx, ry, rz]
            )

    def vel_cb(self, msg):
        with self.state_lock:
            self.velocity = np.array(
                [
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z,
                    msg.twist.angular.x,
                    msg.twist.angular.y,
                    msg.twist.angular.z,
                ]
            )

    def wrench_cb(self, msg):
        with self.state_lock:
            self.wrench = np.array(
                [
                    msg.wrench.force.x,
                    msg.wrench.force.y,
                    msg.wrench.force.z,
                    msg.wrench.torque.x,
                    msg.wrench.torque.y,
                    msg.wrench.torque.z,
                ]
            )

    def pack_and_send_cb(self, _event):
        self.seq_counter += 1
        frame = TeleopFrame()
        frame.header.stamp = rospy.Time.now()
        frame.seq_num = self.seq_counter
        with self.state_lock:
            frame.pose = self.pose.tolist()
            frame.velocity = self.velocity.tolist()
            frame.wrench = self.wrench.tolist()

        if self.enable_sim:
            delay = random.uniform(self.latency_low, self.latency_high)
            threading.Timer(delay, self._delayed_publish, args=(frame,)).start()
        else:
            self.net_pub.publish(frame)

    def _delayed_publish(self, frame_msg):
        self.net_pub.publish(frame_msg)


class JitterBufferNode:
    """Reorders TeleopFrame streams by seq_num and caps jitter."""

    def __init__(self, sub_topic: str, pub_topic: str):
        self.target_depth = rospy.get_param("~buffer_depth", 30)
        self.heap = []
        self.is_playing = False
        self.last_seq = -1
        self.last_valid_frame = TeleopFrame()
        self.lock = threading.Lock()
        self.pub = rospy.Publisher(pub_topic, TeleopFrame, queue_size=50)

        rospy.Subscriber(sub_topic, TeleopFrame, self.network_cb)
        rospy.Timer(rospy.Duration(0.01), self.playout_cb)

    def network_cb(self, msg):
        with self.lock:
            if msg.seq_num <= self.last_seq:
                rospy.logdebug("Dropped stale packet %s", msg.seq_num)
                return
            heapq.heappush(self.heap, (msg.seq_num, msg))
            if not self.is_playing and len(self.heap) >= self.target_depth:
                self.is_playing = True
                rospy.loginfo("Buffer filled. Starting playout.")

    def playout_cb(self, _event):
        output = None
        with self.lock:
            if not self.is_playing or not self.heap:
                output = self.last_valid_frame
            else:
                seq, output = heapq.heappop(self.heap)
                self.last_seq = seq
                self.last_valid_frame = output
        if output and output.seq_num > 0:
            self.pub.publish(output)


def _parse_args():
    p = argparse.ArgumentParser(description="Teleop communication bridge (ROS)")
    p.add_argument(
        "--role",
        choices=("master", "slave"),
        required=True,
        help="master: outbound master frames + inbound slave buffer; "
        "slave: outbound slave frames + inbound master buffer",
    )
    return p.parse_args()


def main():
    args = _parse_args()
    rospy.init_node("teleop_communication", anonymous=False)

    if args.role == "master":
        rospy.loginfo("teleop_communication: role=master (packager + slave-frame buffer)")
        MasterArmDataPackager()
        JitterBufferNode("/network/slave_frame", "/master/teleop_frame")
    else:
        rospy.loginfo("teleop_communication: role=slave (packager + master-frame buffer)")
        SlaveArmDataPackager()
        JitterBufferNode("/network/master_frame", "/slave/teleop_frame")

    rospy.spin()


if __name__ == "__main__":
    main()
