#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

POSE_TOPIC="${POSE_TOPIC:-/detected_object_pose}"
POSE_TIMEOUT="${POSE_TIMEOUT:-10.0}"
POLL_PERIOD="${POLL_PERIOD:-1.0}"
MIN_MOVE="${MIN_MOVE:-0.03}"
MAX_PLANS="${MAX_PLANS:-0}"
APPROACH_STANDOFF="${APPROACH_STANDOFF:-0.25}"
Z_OFFSET="${Z_OFFSET:-0.00}"
PLANNER="${PLANNER:-RRTConnect}"
POSITION_TOLERANCE="${POSITION_TOLERANCE:-0.03}"
PLANNING_TIME="${PLANNING_TIME:-20.0}"
ATTEMPTS="${ATTEMPTS:-10}"
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

set -u

python3 "${PROJECT_ROOT}/tools/auto_plan_detected_object_pose.py" \
  --pose-topic "${POSE_TOPIC}" \
  --pose-timeout "${POSE_TIMEOUT}" \
  --poll-period "${POLL_PERIOD}" \
  --min-move "${MIN_MOVE}" \
  --max-plans "${MAX_PLANS}" \
  --approach-standoff "${APPROACH_STANDOFF}" \
  --z-offset "${Z_OFFSET}" \
  --position-tolerance "${POSITION_TOLERANCE}" \
  --planning-time "${PLANNING_TIME}" \
  --attempts "${ATTEMPTS}" \
  --velocity-scaling "${VELOCITY_SCALING}" \
  --acceleration-scaling "${ACCELERATION_SCALING}" \
  --planner "${PLANNER}"
