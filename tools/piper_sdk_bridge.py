#!/usr/bin/env python3
"""ROS2 bridge from the project joint command topic to a real Piper arm.

The node subscribes to the same topics used by the Unity simulation:

* /enable_cmd
* /joint_ctrl_cmd

It sends ramp-limited joint commands through piper_sdk, so sparse or abrupt
targets become smooth joint-space motion on the real arm. It also publishes the
same feedback topics consumed by the web UI:

* /joint_states_feedback
* /joint_states
* /arm_status
* /link6_pose
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any, Optional

from piper_sdk_console import (
    JOINT_LIMITS_DEG,
    bring_up_can,
    connect_piper,
    get_joint_degrees,
    import_piper_sdk,
    make_piper,
    try_call,
    unwrap_message,
    validate_joints,
    values_from_attrs,
)


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
FULL_JOINT_NAMES = [*JOINT_NAMES, "gripper"]
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


class _Bag:
    pass


class FakePiper:
    def __init__(self) -> None:
        self.joints_raw = [0] * 6
        self.gripper_raw = 0

    def GetArmJointMsgs(self) -> Any:
        msg = _Bag()
        msg.joint_state = _Bag()
        for index, value in enumerate(self.joints_raw, start=1):
            setattr(msg.joint_state, f"joint_{index}", value)
        return msg

    def GetArmGripperMsgs(self) -> Any:
        msg = _Bag()
        msg.gripper_state = _Bag()
        msg.gripper_state.grippers_angle = self.gripper_raw
        msg.gripper_state.grippers_effort = 0
        return msg

    def GetArmHighSpdInfoMsgs(self) -> Any:
        msg = _Bag()
        for index in range(1, 7):
            motor = _Bag()
            motor.motor_speed = 0
            motor.effort = 0
            setattr(msg, f"motor_{index}", motor)
        return msg

    def GetArmStatus(self) -> Any:
        msg = _Bag()
        msg.arm_status = _Bag()
        msg.arm_status.ctrl_mode = 1
        msg.arm_status.arm_status = 1
        msg.arm_status.mode_feed = 1
        msg.arm_status.teach_status = 0
        msg.arm_status.motion_status = 1
        msg.arm_status.trajectory_num = 0
        msg.arm_status.err_code = 0
        msg.arm_status.err_status = _Bag()
        return msg

    def GetArmEndPoseMsgs(self) -> Any:
        msg = _Bag()
        msg.end_pose = _Bag()
        msg.end_pose.X_axis = 0
        msg.end_pose.Y_axis = 0
        msg.end_pose.Z_axis = 0
        msg.end_pose.RX_axis = 0
        msg.end_pose.RY_axis = 0
        msg.end_pose.RZ_axis = 0
        return msg

    def JointCtrl(self, *raw: int) -> None:
        self.joints_raw = [int(value) for value in raw[:6]]

    def GripperCtrl(self, opening_raw: int, *_: Any) -> None:
        self.gripper_raw = int(opening_raw)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def shortest_delta_deg(current: float, target: float) -> float:
    delta = (target - current + 180.0) % 360.0 - 180.0
    return delta


def make_time_msg(node: Any):
    return node.get_clock().now().to_msg()


def get_nested_attr(obj: Any, dotted: str, default: Any = None) -> Any:
    cur = obj
    for part in dotted.split("."):
        if not hasattr(cur, part):
            return default
        cur = getattr(cur, part)
    return cur


def call_first(obj: Any, candidates: list[tuple[str, tuple[Any, ...]]]) -> Any:
    last_exc: Optional[Exception] = None
    for method_name, args in candidates:
        method = getattr(obj, method_name, None)
        if method is None:
            continue
        try:
            return method(*args)
        except TypeError as exc:
            last_exc = exc
            continue
    if last_exc is not None:
        raise last_exc
    raise RuntimeError(f"none of these SDK methods exist: {[name for name, _ in candidates]}")


def get_gripper_meters(piper: Any) -> float:
    method = getattr(piper, "GetArmGripperMsgs", None)
    if method is None:
        return 0.0
    try:
        msg = method()
    except Exception:
        return 0.0
    state = unwrap_message(msg, ["gripper_state", "gripper"])
    value = get_nested_attr(state, "grippers_angle")
    if value is None:
        return 0.0
    return float(value) / 1_000_000.0


def get_motor_feedback(piper: Any) -> tuple[list[float], list[float]]:
    method = getattr(piper, "GetArmHighSpdInfoMsgs", None)
    if method is None:
        return [0.0] * 6, [0.0] * 6
    try:
        msg = method()
    except Exception:
        return [0.0] * 6, [0.0] * 6

    velocities = []
    efforts = []
    for index in range(1, 7):
        motor = get_nested_attr(msg, f"motor_{index}")
        velocities.append(float(getattr(motor, "motor_speed", 0.0)) / 1000.0 if motor is not None else 0.0)
        efforts.append(float(getattr(motor, "effort", 0.0)) / 1000.0 if motor is not None else 0.0)
    return velocities, efforts


def get_end_pose_rpy(piper: Any) -> Optional[tuple[float, float, float, float, float, float]]:
    method = getattr(piper, "GetArmEndPoseMsgs", None)
    if method is None:
        return None
    try:
        msg = method()
    except Exception:
        return None

    pose = unwrap_message(msg, ["end_pose", "arm_end_pose"])
    values = values_from_attrs(pose, ["X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"])
    if values is None:
        return None

    x = float(values[0]) / 1_000_000.0
    y = float(values[1]) / 1_000_000.0
    z = float(values[2]) / 1_000_000.0
    roll = float(values[3]) / 1000.0 * DEG_TO_RAD
    pitch = float(values[4]) / 1000.0 * DEG_TO_RAD
    yaw = float(values[5]) / 1000.0 * DEG_TO_RAD
    return x, y, z, roll, pitch, yaw


def set_motion_mode(piper: Any, speed_percent: int) -> None:
    speed = int(clamp(speed_percent, 1, 100))
    call_first(
        piper,
        [
            ("MotionCtrl_2", (0x01, 0x01, speed)),
            ("ModeCtrl", (0x01, 0x01, speed, 0x00)),
        ],
    )


def send_enable(piper: Any, enable: bool) -> None:
    if enable:
        call_first(
            piper,
            [
                ("EnableArm", (7, 0x02)),
                ("EnableArm", (7,)),
            ],
        )
    else:
        call_first(
            piper,
            [
                ("DisableArm", (7, 0x01)),
                ("DisableArm", (7,)),
            ],
        )


def send_joint_command(piper: Any, degrees: list[float], speed_percent: int, dry_run: bool) -> None:
    validate_joints(degrees)
    if dry_run:
        return

    set_motion_mode(piper, speed_percent)
    raw = [int(round(value * 1000.0)) for value in degrees]
    try_call(piper, "JointCtrl", *raw)


def send_gripper_command(piper: Any, opening_m: float, effort: float, dry_run: bool) -> None:
    if dry_run:
        return

    opening_raw = int(round(clamp(opening_m, 0.0, 0.1) * 1_000_000.0))
    effort_raw = int(round(clamp(effort, 0.0, 5.0) * 1000.0))
    try_call(piper, "GripperCtrl", opening_raw, effort_raw, 0x03, 0x00)


class PiperSdkSmoothBridge:
    def __init__(self, args: argparse.Namespace):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from piper_msgs.msg import PiperStatusMsg
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.JointState = JointState
        self.Bool = Bool
        self.PiperStatusMsg = PiperStatusMsg
        self.PoseStamped = PoseStamped

        self.args = args
        self.node = _Node("piper_sdk_smooth_bridge")
        self.piper = self._connect_sdk()
        self.enabled = False
        self.target_joints_deg: Optional[list[float]] = None
        self.command_joints_deg = self._read_initial_joints()
        self.last_command_time: Optional[float] = None
        self.last_tick_time = time.monotonic()
        self.speed_percent = int(clamp(args.speed, 1, 100))
        self.gripper_target_m = 0.0
        self.gripper_effort = 1.0

        self.joint_feedback_pub = self.node.create_publisher(JointState, "/joint_states_feedback", 10)
        self.joint_states_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.arm_status_pub = self.node.create_publisher(PiperStatusMsg, "/arm_status", 10)
        self.link_pose_pub = self.node.create_publisher(PoseStamped, "/link6_pose", 10)

        self.node.create_subscription(JointState, "/joint_ctrl_cmd", self._on_joint_command, 10)
        self.node.create_subscription(Bool, "/enable_cmd", self._on_enable_command, 10)
        self.node.create_timer(1.0 / args.rate, self._tick)
        self.node.create_timer(1.0 / args.feedback_rate, self._publish_feedback)

        if args.auto_enable:
            self._set_enabled(True)

        self.node.get_logger().info(
            "Piper SDK smooth bridge ready: "
            f"can={args.can}, rate={args.rate:.1f}Hz, max_speed={args.max_speed_deg:.1f}deg/s, "
            f"max_step={args.max_step_deg:.2f}deg, dry_run={args.dry_run}"
        )

    def _connect_sdk(self) -> Any:
        if self.args.dry_run:
            self.node.get_logger().warn("dry-run enabled: SDK commands will not be sent")
            return FakePiper()

        can_name = bring_up_can(self.args.can, self.args.bitrate, self.args.skip_can_setup)
        symbols = import_piper_sdk()
        piper = make_piper(symbols, can_name, self.args.dh_is_offset, self.args.judge_can)
        connect_piper(piper)
        if not self.args.no_slave and hasattr(piper, "MasterSlaveConfig"):
            piper.MasterSlaveConfig(0xFC, 0, 0, 0)
        return piper

    def _read_initial_joints(self) -> list[float]:
        joints = get_joint_degrees(self.piper)
        if joints is None:
            self.node.get_logger().warn("could not read SDK joints on startup; using zeros")
            return [0.0] * 6
        validate_joints(joints)
        return joints

    def _on_enable_command(self, msg: Any) -> None:
        self._set_enabled(bool(msg.data))

    def _set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        if self.args.dry_run:
            self.node.get_logger().warn(f"dry-run enable={enabled}")
            return

        try:
            send_enable(self.piper, enabled)
            self.node.get_logger().info("arm enabled" if enabled else "arm disabled")
        except Exception as exc:
            self.node.get_logger().error(f"enable command failed: {exc}")

    def _on_joint_command(self, msg: Any) -> None:
        target = self.target_joints_deg or list(self.command_joints_deg)
        by_name = {name: float(value) for name, value in zip(msg.name, msg.position)}
        for index, name in enumerate(JOINT_NAMES):
            if name in by_name:
                target[index] = by_name[name] * RAD_TO_DEG

        validate_joints(target)
        self.target_joints_deg = target
        self.last_command_time = time.monotonic()

        if len(msg.velocity) >= 7 and msg.velocity[6] > 0.0:
            self.speed_percent = int(clamp(float(msg.velocity[6]), 1, 100))
        if "gripper" in by_name:
            self.gripper_target_m = clamp(by_name["gripper"], 0.0, 0.1)
        elif len(msg.position) >= 7:
            self.gripper_target_m = clamp(float(msg.position[6]), 0.0, 0.1)
        if len(msg.effort) >= 7 and msg.effort[6] > 0.0:
            self.gripper_effort = float(msg.effort[6])

        if self.args.auto_enable and not self.enabled:
            self._set_enabled(True)

        self.node.get_logger().info(
            "target joints deg="
            + ", ".join(f"{value:.2f}" for value in target)
            + f" speed={self.speed_percent}%"
        )

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.001, min(0.2, now - self.last_tick_time))
        self.last_tick_time = now

        if self.last_command_time is not None and self.args.command_timeout > 0.0:
            if now - self.last_command_time > self.args.command_timeout:
                self.target_joints_deg = None
                self.last_command_time = None
                feedback = get_joint_degrees(self.piper)
                if feedback is not None:
                    self.command_joints_deg = feedback
                self.node.get_logger().warn("command timeout; holding current joint state")
                return

        if not self.enabled or self.target_joints_deg is None:
            return

        next_joints = list(self.command_joints_deg)
        max_step = min(self.args.max_step_deg, self.args.max_speed_deg * dt)
        reached = True
        for index, target in enumerate(self.target_joints_deg):
            delta = shortest_delta_deg(next_joints[index], target)
            step = clamp(delta, -max_step, max_step)
            if abs(delta) > self.args.position_tolerance_deg:
                reached = False
            next_joints[index] = clamp(
                next_joints[index] + step,
                JOINT_LIMITS_DEG[index][0],
                JOINT_LIMITS_DEG[index][1],
            )

        try:
            send_joint_command(self.piper, next_joints, self.speed_percent, self.args.dry_run)
            if self.args.send_gripper:
                send_gripper_command(self.piper, self.gripper_target_m, self.gripper_effort, self.args.dry_run)
            self.command_joints_deg = next_joints
            if reached:
                self.target_joints_deg = list(next_joints)
        except Exception as exc:
            self.node.get_logger().error(f"SDK joint command failed: {exc}")

    def _publish_feedback(self) -> None:
        feedback_deg = list(self.command_joints_deg) if self.args.dry_run else get_joint_degrees(self.piper)
        if feedback_deg is None:
            feedback_deg = list(self.command_joints_deg)
        gripper = get_gripper_meters(self.piper)
        velocities, efforts = get_motor_feedback(self.piper)

        msg = self.JointState()
        msg.header.stamp = make_time_msg(self.node)
        msg.name = list(FULL_JOINT_NAMES)
        msg.position = [value * DEG_TO_RAD for value in feedback_deg] + [gripper]
        msg.velocity = list(velocities) + [0.0]
        msg.effort = list(efforts) + [self.gripper_effort]
        self.joint_feedback_pub.publish(msg)
        if self.args.publish_joint_states:
            self.joint_states_pub.publish(msg)

        self._publish_arm_status()
        self._publish_link_pose()

    def _publish_arm_status(self) -> None:
        msg = self.PiperStatusMsg()
        status_method = getattr(self.piper, "GetArmStatus", None)
        status = None
        if status_method is not None:
            try:
                status = unwrap_message(status_method(), ["arm_status"])
            except Exception:
                status = None

        msg.ctrl_mode = int(getattr(status, "ctrl_mode", 1))
        msg.arm_status = int(getattr(status, "arm_status", 1 if self.enabled else 0))
        msg.mode_feedback = int(getattr(status, "mode_feed", getattr(status, "mode_feedback", 1)))
        msg.teach_status = int(getattr(status, "teach_status", 0))
        msg.motion_status = int(getattr(status, "motion_status", 1 if self.enabled else 0))
        msg.trajectory_num = int(getattr(status, "trajectory_num", 0))
        msg.err_code = int(getattr(status, "err_code", 0))

        err = getattr(status, "err_status", None)
        for index in range(1, 7):
            setattr(msg, f"joint_{index}_angle_limit", bool(getattr(err, f"joint_{index}_angle_limit", False)))
            setattr(
                msg,
                f"communication_status_joint_{index}",
                bool(getattr(err, f"communication_status_joint_{index}", True)),
            )

        self.arm_status_pub.publish(msg)

    def _publish_link_pose(self) -> None:
        pose = get_end_pose_rpy(self.piper)
        if pose is None:
            return

        x, y, z, roll, pitch, yaw = pose
        msg = self.PoseStamped()
        msg.header.stamp = make_time_msg(self.node)
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = roll
        msg.pose.orientation.y = pitch
        msg.pose.orientation.z = yaw
        msg.pose.orientation.w = 0.0
        self.link_pose_pub.publish(msg)

    def spin(self) -> None:
        self.rclpy.spin(self.node)

    def destroy(self) -> None:
        self.node.destroy_node()


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="SocketCAN interface name")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate")
    parser.add_argument("--skip-can-setup", action="store_true", help="Do not configure SocketCAN")
    parser.add_argument("--judge-can", action="store_true", help="Let SDK check CAN health during init")
    parser.add_argument("--dh-is-offset", type=int, choices=(0, 1), default=1, help="SDK DH offset flag")
    parser.add_argument("--no-slave", action="store_true", help="Do not send MasterSlaveConfig on startup")
    parser.add_argument("--dry-run", action="store_true", help="Run ROS bridge without sending SDK commands")
    parser.add_argument("--auto-enable", action="store_true", help="Enable arm on startup / first command")
    parser.add_argument("--speed", type=int, default=5, help="Default Piper speed percent")
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop rate in Hz")
    parser.add_argument("--feedback-rate", type=float, default=20.0, help="Feedback publish rate in Hz")
    parser.add_argument("--max-speed-deg", type=float, default=12.0, help="Max joint speed in deg/s")
    parser.add_argument("--max-step-deg", type=float, default=1.0, help="Max joint step per loop in deg")
    parser.add_argument("--position-tolerance-deg", type=float, default=0.05, help="Target tolerance in deg")
    parser.add_argument("--command-timeout", type=float, default=5.0, help="Hold current state after no commands for N seconds; 0 disables")
    parser.add_argument("--frame-id", default="base_link", help="Frame id for /link6_pose")
    parser.add_argument("--no-joint-states", action="store_false", dest="publish_joint_states", help="Do not publish /joint_states")
    parser.add_argument("--send-gripper", action="store_true", help="Forward gripper value from /joint_ctrl_cmd")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    import rclpy

    rclpy.init(args=None)
    bridge = None
    try:
        bridge = PiperSdkSmoothBridge(args)
        bridge.spin()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if bridge is not None:
            bridge.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
