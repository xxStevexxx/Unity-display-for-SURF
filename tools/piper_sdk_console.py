#!/usr/bin/env python3
"""Small interactive Piper SDK smoke-test console.

This script is intentionally conservative: it can bring up a SocketCAN
interface, connect through piper_sdk, print feedback, and send small explicit
commands typed by the operator.
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Iterable, Optional


CAN_BITRATE = 1_000_000
JOINT_LIMITS_DEG = [
    (-150.0, 150.0),
    (0.0, 180.0),
    (-170.0, 0.0),
    (-100.0, 100.0),
    (-70.0, 70.0),
    (-120.0, 120.0),
]


@dataclass
class PiperSdkSymbols:
    interface_cls: Any
    log_level: Optional[Any]


def run(cmd: list[str], *, sudo: bool = False, check: bool = True) -> subprocess.CompletedProcess[str]:
    actual = cmd
    if sudo and os.geteuid() != 0:
        actual = ["sudo", *cmd]

    return subprocess.run(actual, check=check, text=True, capture_output=True)


def command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def list_can_interfaces() -> list[str]:
    sys_class_net = "/sys/class/net"
    if not os.path.isdir(sys_class_net):
        return []
    return sorted(name for name in os.listdir(sys_class_net) if name.startswith("can"))


def interface_exists(name: str) -> bool:
    return os.path.exists(f"/sys/class/net/{name}")


def interface_is_up(name: str) -> bool:
    try:
        with open(f"/sys/class/net/{name}/operstate", "r", encoding="utf-8") as handle:
            return handle.read().strip() in {"up", "unknown"}
    except OSError:
        return False


def auto_select_can(requested: str) -> str:
    if interface_exists(requested):
        return requested

    interfaces = list_can_interfaces()
    if len(interfaces) == 1:
        print(f"[can] requested {requested!r} not found; using detected {interfaces[0]!r}")
        return interfaces[0]

    if interfaces:
        joined = ", ".join(interfaces)
        raise RuntimeError(f"CAN interface {requested!r} not found. Detected: {joined}. Pass --can <name>.")

    raise RuntimeError("No SocketCAN interface found. Plug in the Piper CAN adapter first.")


def bring_up_can(can_name: str, bitrate: int, skip_can_setup: bool) -> str:
    if skip_can_setup:
        return auto_select_can(can_name)

    if not command_exists("ip"):
        raise RuntimeError("'ip' command not found. Install iproute2 first.")

    selected = auto_select_can(can_name)
    if interface_is_up(selected):
        print(f"[can] {selected} already up")
        return selected

    print(f"[can] bringing up {selected} at {bitrate} bps")
    run(["ip", "link", "set", selected, "down"], sudo=True, check=False)
    run(["ip", "link", "set", selected, "type", "can", "bitrate", str(bitrate)], sudo=True)
    run(["ip", "link", "set", selected, "up"], sudo=True)
    return selected


def import_piper_sdk() -> PiperSdkSymbols:
    try:
        import piper_sdk  # type: ignore
    except ImportError as exc:
        raise RuntimeError("piper_sdk is not installed. Install with: pip3 install piper_sdk") from exc

    interface_cls = getattr(piper_sdk, "C_PiperInterface_V2", None)
    if interface_cls is None:
        interface_cls = getattr(piper_sdk, "C_PiperInterface", None)
    if interface_cls is None:
        raise RuntimeError("piper_sdk does not export C_PiperInterface or C_PiperInterface_V2")

    log_level = getattr(piper_sdk, "LogLevel", None)
    return PiperSdkSymbols(interface_cls=interface_cls, log_level=log_level)


def make_piper(symbols: PiperSdkSymbols, can_name: str, dh_is_offset: int, judge_flag: bool) -> Any:
    kwargs = {
        "can_name": can_name,
        "judge_flag": judge_flag,
        "can_auto_init": True,
        "dh_is_offset": dh_is_offset,
        "start_sdk_joint_limit": True,
        "start_sdk_gripper_limit": True,
        "log_to_file": False,
        "log_file_path": None,
    }
    if symbols.log_level is not None:
        kwargs["logger_level"] = getattr(symbols.log_level, "WARNING", None)

    try:
        return symbols.interface_cls(**kwargs)
    except TypeError:
        return symbols.interface_cls(can_name, judge_flag)


def connect_piper(piper: Any) -> None:
    print("[sdk] connecting")
    piper.ConnectPort()
    time.sleep(0.05)
    if hasattr(piper, "SearchPiperFirmwareVersion"):
        piper.SearchPiperFirmwareVersion()
        time.sleep(0.05)
    if hasattr(piper, "EnableFkCal"):
        try_call(piper, "EnableFkCal")


def try_call(obj: Any, method_name: str, *args: Any) -> Any:
    method = getattr(obj, method_name, None)
    if method is None:
        raise RuntimeError(f"SDK method {method_name} is not available")
    return method(*args)


def read_attr(obj: Any, names: Iterable[str]) -> Optional[Any]:
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def unwrap_message(obj: Any, payload_names: Iterable[str]) -> Any:
    for name in payload_names:
        value = read_attr(obj, [name])
        if value is not None:
            return value
    return obj


def values_from_attrs(obj: Any, attr_names: list[str]) -> Optional[list[Any]]:
    values = []
    for name in attr_names:
        if not hasattr(obj, name):
            return None
        values.append(getattr(obj, name))
    return values


def format_scaled(values: Iterable[Any], scale: float, precision: int = 3) -> str:
    return "[" + ", ".join(f"{float(value) * scale:.{precision}f}" for value in values) + "]"


def get_joint_degrees(piper: Any) -> Optional[list[float]]:
    msg = try_call(piper, "GetArmJointMsgs")
    state = unwrap_message(msg, ["joint_state", "joint_states", "arm_joint_msgs"])
    values = values_from_attrs(state, [f"joint_{i}" for i in range(1, 7)])
    if values is None:
        return None
    return [float(value) / 1000.0 for value in values]


def print_joints(piper: Any) -> None:
    joints = get_joint_degrees(piper)
    if joints is None:
        print(try_call(piper, "GetArmJointMsgs"))
        return
    print("joints deg:", "[" + ", ".join(f"{value:.3f}" for value in joints) + "]")


def print_pose(piper: Any) -> None:
    msg = try_call(piper, "GetArmEndPoseMsgs")
    pose = unwrap_message(msg, ["end_pose", "arm_end_pose"])
    values = values_from_attrs(pose, ["X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis"])
    if values is None:
        print(msg)
        return
    xyz_m = [float(value) / 1_000_000.0 for value in values[:3]]
    rpy_deg = [float(value) / 1000.0 for value in values[3:]]
    print(f"pose xyz(m): {format_scaled(values[:3], 1 / 1_000_000.0)} rpy(deg): {format_scaled(values[3:], 1 / 1000.0)}")


def print_status(piper: Any) -> None:
    for method_name in ("GetArmStatus", "GetArmEnableStatus", "GetArmGripperMsgs", "GetCanFps"):
        method = getattr(piper, method_name, None)
        if method is None:
            continue
        try:
            print(f"{method_name}: {method()}")
        except Exception as exc:
            print(f"{method_name}: ERROR {exc}")


def print_fk(piper: Any) -> None:
    fk = try_call(piper, "GetFK", "feedback")
    print(fk)


def validate_joints(degrees: list[float]) -> None:
    if len(degrees) != 6:
        raise ValueError("expected 6 joint angles")
    for index, value in enumerate(degrees):
        low, high = JOINT_LIMITS_DEG[index]
        if value < low or value > high:
            raise ValueError(f"joint{index + 1}={value:.3f} deg outside [{low}, {high}]")


def send_mode_j(piper: Any, speed_percent: int) -> None:
    speed = max(1, min(100, int(speed_percent)))
    try_call(piper, "ModeCtrl", 0x01, 0x01, speed, 0x00)


def send_joints(piper: Any, degrees: list[float], speed_percent: int) -> None:
    validate_joints(degrees)
    send_mode_j(piper, speed_percent)
    raw = [int(round(value * 1000.0)) for value in degrees]
    try_call(piper, "JointCtrl", *raw)


def send_gripper(piper: Any, opening_mm: float, effort_nm: float) -> None:
    opening_raw = int(round(max(0.0, min(100.0, opening_mm)) * 1000.0))
    effort_raw = int(round(max(0.0, min(5.0, effort_nm)) * 1000.0))
    try_call(piper, "GripperCtrl", opening_raw, effort_raw, 0x03, 0x00)


def print_help() -> None:
    print(
        """
Commands:
  help                         Show this help
  status                       Print arm status, enable flags, gripper, CAN FPS
  joints                       Print joint feedback in degrees
  pose                         Print end pose feedback in meters/degrees
  fk                           Print SDK FK data, if supported
  firmware                     Print firmware version
  slave                        Switch arm to slave mode for feedback
  enable                       Enable all motors
  disable                      Disable all motors
  estop                        Emergency stop
  resume                       Resume from emergency stop
  modej [speed]                Set CAN MOVE J mode, default speed from --speed
  home                         Send all joints to 0 deg at current speed
  joints d1 d2 d3 d4 d5 d6     Send absolute joint angles in degrees
  jog n delta                  Add delta degrees to joint n (1-6)
  grip mm [effort]             Set gripper opening in mm, effort N*m default 1.0
  gripzero                     Set current gripper position as zero
  watch [hz]                   Repeatedly print joints and pose; Ctrl-C to stop
  quit                         Disable nothing, just exit the console
"""
    )


def repl(piper: Any, default_speed: int) -> None:
    speed_percent = default_speed
    print_help()
    while True:
        try:
            line = input("piper> ").strip()
        except EOFError:
            print()
            return
        except KeyboardInterrupt:
            print()
            continue

        if not line:
            continue

        try:
            parts = shlex.split(line)
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in {"q", "quit", "exit"}:
                return
            if cmd in {"h", "help", "?"}:
                print_help()
            elif cmd == "status":
                print_status(piper)
            elif cmd == "joints":
                if args:
                    degrees = [float(arg) for arg in args]
                    send_joints(piper, degrees, speed_percent)
                    print("sent joints deg:", degrees)
                else:
                    print_joints(piper)
            elif cmd == "pose":
                print_pose(piper)
            elif cmd == "fk":
                print_fk(piper)
            elif cmd == "firmware":
                print(try_call(piper, "GetPiperFirmwareVersion"))
            elif cmd == "slave":
                try_call(piper, "MasterSlaveConfig", 0xFC, 0, 0, 0)
                print("sent slave mode config")
            elif cmd == "enable":
                try_call(piper, "EnableArm", 7, 0x02)
                print("enabled all motors")
            elif cmd == "disable":
                try_call(piper, "DisableArm", 7, 0x01)
                print("disabled all motors")
            elif cmd == "estop":
                try_call(piper, "EmergencyStop", 0x01)
                print("emergency stop sent")
            elif cmd == "resume":
                try_call(piper, "EmergencyStop", 0x02)
                print("emergency stop resume sent")
            elif cmd == "modej":
                if args:
                    speed_percent = max(1, min(100, int(float(args[0]))))
                send_mode_j(piper, speed_percent)
                print(f"MOVE J mode, speed={speed_percent}%")
            elif cmd == "home":
                send_joints(piper, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed_percent)
                print("sent home joints")
            elif cmd == "jog":
                if len(args) != 2:
                    raise ValueError("usage: jog n delta")
                joint_index = int(args[0]) - 1
                if joint_index < 0 or joint_index >= 6:
                    raise ValueError("joint index must be 1..6")
                current = get_joint_degrees(piper)
                if current is None:
                    raise RuntimeError("could not read current joints")
                current[joint_index] += float(args[1])
                send_joints(piper, current, speed_percent)
                print("sent joints deg:", "[" + ", ".join(f"{value:.3f}" for value in current) + "]")
            elif cmd == "grip":
                if not args:
                    raise ValueError("usage: grip mm [effort]")
                opening_mm = float(args[0])
                effort = float(args[1]) if len(args) > 1 else 1.0
                send_gripper(piper, opening_mm, effort)
                print(f"sent gripper opening={opening_mm:.3f} mm effort={effort:.3f} N*m")
            elif cmd == "gripzero":
                try_call(piper, "GripperCtrl", 0, 0, 0x03, 0xAE)
                print("sent gripper zero command")
            elif cmd == "watch":
                hz = float(args[0]) if args else 2.0
                delay = 1.0 / max(0.1, hz)
                print("watching; Ctrl-C to stop")
                try:
                    while True:
                        print_joints(piper)
                        print_pose(piper)
                        time.sleep(delay)
                except KeyboardInterrupt:
                    print()
            else:
                print(f"unknown command: {cmd}")
        except Exception as exc:
            print(f"ERROR: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="SocketCAN interface name, default: can0")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE, help="CAN bitrate, default: 1000000")
    parser.add_argument("--skip-can-setup", action="store_true", help="Do not run ip link setup")
    parser.add_argument("--judge-can", action="store_true", help="Let SDK check CAN health during init")
    parser.add_argument("--dh-is-offset", type=int, choices=(0, 1), default=1, help="SDK DH offset flag")
    parser.add_argument("--speed", type=int, default=10, help="Default MOVE J speed percent")
    parser.add_argument("--no-slave", action="store_true", help="Do not send MasterSlaveConfig on startup")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        can_name = bring_up_can(args.can, args.bitrate, args.skip_can_setup)
        symbols = import_piper_sdk()
        piper = make_piper(symbols, can_name, args.dh_is_offset, args.judge_can)
        connect_piper(piper)
        if not args.no_slave and hasattr(piper, "MasterSlaveConfig"):
            piper.MasterSlaveConfig(0xFC, 0, 0, 0)
            time.sleep(0.05)
        print(f"[ready] connected on {can_name}. Default speed={args.speed}%")
        repl(piper, max(1, min(100, args.speed)))
        return 0
    except KeyboardInterrupt:
        print()
        return 130
    except Exception as exc:
        print(f"fatal: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
