#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import sys
import time
from pathlib import Path


def parse_pose_xyz(text: str) -> tuple[float, float, float]:
    in_position = False
    values: dict[str, float] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if stripped == "position:":
            in_position = True
            continue
        if stripped == "orientation:":
            in_position = False
            continue
        if in_position:
            match = re.match(r"([xyz]):\s*([-+0-9.eE]+)", stripped)
            if match:
                values[match.group(1)] = float(match.group(2))

    if not {"x", "y", "z"}.issubset(values):
        raise ValueError("could not parse pose.position.x/y/z")

    return values["x"], values["y"], values["z"]


def approach_target(
    xyz: tuple[float, float, float],
    standoff: float,
    z_offset: float,
) -> tuple[float, float, float]:
    x, y, z = xyz
    z += z_offset

    if standoff > 0.0:
        xy_norm = math.hypot(x, y)
        if xy_norm > 1e-6:
            x -= standoff * x / xy_norm
            y -= standoff * y / xy_norm
        else:
            x -= standoff

    return x, y, z


def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def read_pose_once(topic: str, timeout_sec: float) -> tuple[float, float, float]:
    completed = subprocess.run(
        ["ros2", "topic", "echo", topic, "--once"],
        capture_output=True,
        text=True,
        timeout=timeout_sec,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())
    return parse_pose_xyz(completed.stdout)


def run_planner(args: argparse.Namespace, target: tuple[float, float, float]) -> int:
    planner_script = Path(__file__).with_name("piper_moveit_pose_planner.py")
    x, y, z = target
    command = [
        sys.executable,
        str(planner_script),
        "--x",
        f"{x:.6f}",
        "--y",
        f"{y:.6f}",
        "--z",
        f"{z:.6f}",
        "--position-only",
        "--position-tolerance",
        str(args.position_tolerance),
        "--planning-time",
        str(args.planning_time),
        "--attempts",
        str(args.attempts),
        "--velocity-scaling",
        str(args.velocity_scaling),
        "--acceleration-scaling",
        str(args.acceleration_scaling),
        "--planner",
        args.planner,
    ]
    return subprocess.run(command, check=False).returncode


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Continuously replan to a detected object pose when it moves."
    )
    parser.add_argument("--pose-topic", default="/detected_object_pose")
    parser.add_argument("--pose-timeout", type=float, default=10.0)
    parser.add_argument("--poll-period", type=float, default=1.0)
    parser.add_argument("--min-move", type=float, default=0.03)
    parser.add_argument("--max-plans", type=int, default=0)
    parser.add_argument("--approach-standoff", type=float, default=0.25)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--position-tolerance", type=float, default=0.03)
    parser.add_argument("--planning-time", type=float, default=20.0)
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--velocity-scaling", type=float, default=0.25)
    parser.add_argument("--acceleration-scaling", type=float, default=0.25)
    parser.add_argument("--planner", default="RRTConnect")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    last_planned_target: tuple[float, float, float] | None = None
    plan_count = 0

    print(
        f"Auto planning from {args.pose_topic}. "
        f"min_move={args.min_move:.3f} m, poll_period={args.poll_period:.2f} s"
    )

    while args.max_plans <= 0 or plan_count < args.max_plans:
        try:
            object_xyz = read_pose_once(args.pose_topic, args.pose_timeout)
            target_xyz = approach_target(object_xyz, args.approach_standoff, args.z_offset)
        except subprocess.TimeoutExpired:
            print(f"Timed out waiting for {args.pose_topic}", file=sys.stderr)
            time.sleep(args.poll_period)
            continue
        except Exception as exc:
            print(f"Could not read pose: {exc}", file=sys.stderr)
            time.sleep(args.poll_period)
            continue

        moved = (
            last_planned_target is None
            or distance(target_xyz, last_planned_target) >= args.min_move
        )
        if not moved:
            time.sleep(args.poll_period)
            continue

        print(
            "Planning to updated target xyz="
            f"({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f})"
        )
        return_code = run_planner(args, target_xyz)
        if return_code == 0:
            last_planned_target = target_xyz
            plan_count += 1
        else:
            print(f"Planner failed with exit code {return_code}", file=sys.stderr)

        time.sleep(args.poll_period)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
