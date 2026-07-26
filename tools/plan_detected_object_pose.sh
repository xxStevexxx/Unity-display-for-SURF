#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

POSE_TOPIC="${POSE_TOPIC:-/detected_object_pose}"
POSE_TIMEOUT="${POSE_TIMEOUT:-10.0}"
APPROACH_STANDOFF="${APPROACH_STANDOFF:-0.10}"
Z_OFFSET="${Z_OFFSET:-0.05}"
PLANNER="${PLANNER:-RRTConnect}"
POSITION_TOLERANCE="${POSITION_TOLERANCE:-0.02}"
PLANNING_TIME="${PLANNING_TIME:-8.0}"
ATTEMPTS="${ATTEMPTS:-3}"
VELOCITY_SCALING="${VELOCITY_SCALING:-0.25}"
ACCELERATION_SCALING="${ACCELERATION_SCALING:-0.25}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/ros2_ws/install/setup.bash"
fi

if [[ -n "${PIPER_ROS_ROOT:-}" && -f "${PIPER_ROS_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${PIPER_ROS_ROOT}/install/setup.bash"
fi

set -u

echo "Planning near ${POSE_TOPIC}"
echo "  pose timeout: ${POSE_TIMEOUT} s"
echo "  standoff: ${APPROACH_STANDOFF} m"
echo "  z offset: ${Z_OFFSET} m"
echo "  planner:  ${PLANNER}"

POSE_YAML="$(
  timeout "${POSE_TIMEOUT}" ros2 topic echo --once "${POSE_TOPIC}"
)" || {
  echo "Failed to receive one pose from ${POSE_TOPIC} within ${POSE_TIMEOUT} seconds" >&2
  exit 1
}

read -r TARGET_X TARGET_Y TARGET_Z <<< "$(
  printf '%s\n' "${POSE_YAML}" | python3 "${PROJECT_ROOT}/tools/parse_pose_target.py" \
    "${APPROACH_STANDOFF}" "${Z_OFFSET}"
)"

echo "  target xyz: ${TARGET_X}, ${TARGET_Y}, ${TARGET_Z}"

python3 "${PROJECT_ROOT}/tools/piper_moveit_pose_planner.py" \
  --x "${TARGET_X}" \
  --y "${TARGET_Y}" \
  --z "${TARGET_Z}" \
  --position-only \
  --position-tolerance "${POSITION_TOLERANCE}" \
  --planning-time "${PLANNING_TIME}" \
  --attempts "${ATTEMPTS}" \
  --velocity-scaling "${VELOCITY_SCALING}" \
  --acceleration-scaling "${ACCELERATION_SCALING}" \
  --planner "${PLANNER}"
