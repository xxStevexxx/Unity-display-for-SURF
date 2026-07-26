#!/usr/bin/env python3
import math
import re
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: parse_pose_target.py APPROACH_STANDOFF Z_OFFSET", file=sys.stderr)
        return 2

    standoff = float(sys.argv[1])
    z_offset = float(sys.argv[2])
    text = sys.stdin.read().splitlines()

    in_position = False
    values: dict[str, float] = {}
    for line in text:
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
        print("Could not parse pose.position.x/y/z", file=sys.stderr)
        return 1

    x = values["x"]
    y = values["y"]
    z = values["z"] + z_offset

    if standoff > 0.0:
        xy_norm = math.hypot(x, y)
        if xy_norm > 1e-6:
            x -= standoff * x / xy_norm
            y -= standoff * y / xy_norm
        else:
            x -= standoff

    print(f"{x:.6f} {y:.6f} {z:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
