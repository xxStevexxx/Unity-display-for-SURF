#!/usr/bin/env python3
"""ROS 2 node that publishes the current ``base_link → link6`` pose.

Listens to TF transforms forwarded by ``robot_state_publisher``
(which itself consumes ``/joint_states``) and extracts the
``(x, y, z, roll, pitch, yaw)`` pose of the end‑effector in the
**world frame**.  The result is published on ``/link6_pose`` as a
``geometry_msgs/PoseStamped`` at 10 Hz.

Usage
-----

.. code-block:: bash

    ros2 run piper_tools piper_fk_publisher.py  \\
        --base-frame base_link  \\
        --tip-frame  link6     \\
        --rate       10.0
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

DEFAULT_BASE_FRAME = "base_link"
DEFAULT_TIP_FRAME = "link6"
DEFAULT_RATE = 10.0


def _quat_to_euler(qx, qy, qz, qw):
    """Convert quaternion to intrinsic ZYX Euler angles (radians)."""
    # Roll  (rotation about X)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation about Y)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw   (rotation about Z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class FKPublisher(Node):
    """Continuously publishes the current ``link6`` pose."""

    def __init__(self, base_frame: str, tip_frame: str, rate_hz: float):
        super().__init__("piper_fk_publisher")
        self._base_frame = base_frame
        self._tip_frame = tip_frame

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(PoseStamped, "/link6_pose", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_pose)

        self.get_logger().info(
            f"FK publisher ready: {base_frame!r} → {tip_frame!r} "
            f"@{rate_hz:.1f} Hz → /link6_pose"
        )

    def _publish_pose(self):
        try:
            when = self.get_clock().now() - rclpy.duration.Duration(
                seconds=0.01
            )
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._tip_frame,
                when,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            return  # TF not available yet — skip this cycle

        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self._base_frame

        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z

        q = tf.transform.rotation
        roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)
        msg.pose.orientation.x = roll
        msg.pose.orientation.y = pitch
        msg.pose.orientation.z = yaw
        msg.pose.orientation.w = 0.0  # sentinel

        self._pub.publish(msg)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="TF‑based forward‑kinematics publisher for link6",
    )
    parser.add_argument(
        "--base-frame",
        default=DEFAULT_BASE_FRAME,
        help="TF parent frame (default: %(default)s)",
    )
    parser.add_argument(
        "--tip-frame",
        default=DEFAULT_TIP_FRAME,
        help="TF child frame (default: %(default)s)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE,
        help="Publish rate in Hz (default: %(default).1f)",
    )

    # ROS 2 args are stripped by rclpy.init
    # Pre-parse our own flags.
    known, _ = parser.parse_known_args(argv)

    rclpy.init(args=argv)
    node = FKPublisher(known.base_frame, known.tip_frame, known.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
