#!/usr/bin/env bash
set -euo pipefail

SURF_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PIPER_ROS_ROOT="${PIPER_ROS_ROOT:-/home/Light/Projects/Osw_Surf_VR/code/piper_ros}"
DOCKER_IMAGE="${PIPER_DOCKER_IMAGE:-piper-ros:humble}"
CONTAINER_NAME="${PIPER_REAL_CONTAINER:-piper_real_bridge}"
CAN_NAME="${CAN_NAME:-can0}"
SPEED_PERCENT="${SPEED_PERCENT:-5}"
MAX_SPEED_DEG="${MAX_SPEED_DEG:-12}"
SKIP_INSTALL="${SKIP_INSTALL:-0}"
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  tools/piper_real_bridge_docker.sh [options] [-- extra bridge args]

Starts the real-arm ROS2 bridge:
  /joint_ctrl_cmd + /enable_cmd -> piper_sdk -> CAN -> real Piper

Options:
  --can NAME          SocketCAN interface name. Default: can0
  --speed N          Piper SDK speed percent. Default: 5
  --max-speed-deg N  Smooth joint speed limit in deg/s. Default: 12
  --image IMAGE      Docker image. Default: piper-ros:humble
  --name NAME        Container name. Default: piper_real_bridge
  --skip-install     Do not install/update piper_sdk in the container
  -h, --help         Show this help

Examples:
  tools/piper_real_bridge_docker.sh --can can_piper --speed 5
  tools/piper_real_bridge_docker.sh --can can_piper -- --dry-run
  tools/piper_real_bridge_docker.sh --can can_piper -- --auto-enable --max-speed-deg 8
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --can)
      CAN_NAME="${2:?missing value for --can}"
      shift 2
      ;;
    --speed)
      SPEED_PERCENT="${2:?missing value for --speed}"
      shift 2
      ;;
    --max-speed-deg)
      MAX_SPEED_DEG="${2:?missing value for --max-speed-deg}"
      shift 2
      ;;
    --image)
      DOCKER_IMAGE="${2:?missing value for --image}"
      shift 2
      ;;
    --name)
      CONTAINER_NAME="${2:?missing value for --name}"
      shift 2
      ;;
    --skip-install)
      SKIP_INSTALL=1
      shift
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "docker not found" >&2
  exit 1
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

quoted_extra=""
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  printf -v quoted_extra ' %q' "${EXTRA_ARGS[@]}"
fi

inner_script=$(cat <<'EOS'
set -euo pipefail
cd /surf

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -f /ws/piper_ros/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /ws/piper_ros/install/setup.bash
fi

if [[ "${SKIP_INSTALL}" != "1" ]]; then
  sdk_site=/tmp/piper-sdk-site
  mkdir -p "${sdk_site}"
  python3 -m pip install --upgrade --target "${sdk_site}" piper_sdk python-can
  export PYTHONPATH="${sdk_site}:${PYTHONPATH:-}"
elif [[ -d /tmp/piper-sdk-site ]]; then
  export PYTHONPATH="/tmp/piper-sdk-site:${PYTHONPATH:-}"
fi

extra_args=()
if [[ -n "${EXTRA_ARGS:-}" ]]; then
  eval "extra_args=(${EXTRA_ARGS})"
fi

exec /surf/tools/piper_sdk_bridge.py \
  --can "${CAN_NAME}" \
  --speed "${SPEED_PERCENT}" \
  --max-speed-deg "${MAX_SPEED_DEG}" \
  "${extra_args[@]}"
EOS
)

run_inside_existing() {
  docker exec "${tty_args[@]}" \
    -e CAN_NAME="${CAN_NAME}" \
    -e SPEED_PERCENT="${SPEED_PERCENT}" \
    -e MAX_SPEED_DEG="${MAX_SPEED_DEG}" \
    -e SKIP_INSTALL="${SKIP_INSTALL}" \
    -e EXTRA_ARGS="${quoted_extra}" \
    -w /surf \
    "$1" \
    bash -lc "${inner_script}"
}

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  run_inside_existing "${CONTAINER_NAME}"
  exit $?
fi

if docker ps --format '{{.Names}}' | grep -qx 'piper_unity_humble'; then
  echo "[docker] using existing piper_unity_humble container"
  run_inside_existing "piper_unity_humble"
  exit $?
fi

docker run --rm "${tty_args[@]}" \
  --name "${CONTAINER_NAME}" \
  --network host \
  --privileged \
  --ipc host \
  -e CAN_NAME="${CAN_NAME}" \
  -e SPEED_PERCENT="${SPEED_PERCENT}" \
  -e MAX_SPEED_DEG="${MAX_SPEED_DEG}" \
  -e SKIP_INSTALL="${SKIP_INSTALL}" \
  -e EXTRA_ARGS="${quoted_extra}" \
  -v "${SURF_ROOT}:/surf" \
  -v "${PIPER_ROS_ROOT}:/ws/piper_ros" \
  -w /surf \
  "${DOCKER_IMAGE}" \
  bash -lc "${inner_script}"
