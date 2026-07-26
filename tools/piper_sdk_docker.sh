#!/usr/bin/env bash
set -euo pipefail

SURF_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_IMAGE="${PIPER_DOCKER_IMAGE:-piper-ros:humble}"
CONTAINER_NAME="${PIPER_SDK_CONTAINER:-piper_sdk_test}"
CAN_NAME="${CAN_NAME:-can0}"
SPEED_PERCENT="${SPEED_PERCENT:-5}"
SKIP_INSTALL="${SKIP_INSTALL:-0}"
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  tools/piper_sdk_docker.sh [options] [-- extra console args]

Options:
  --can NAME          SocketCAN interface name. Default: can0
  --speed N          Default SDK console speed percent. Default: 5
  --image IMAGE      Docker image. Default: piper-ros:humble
  --name NAME        Container name. Default: piper_sdk_test
  --skip-install     Do not install/update piper_sdk in the container venv
  -h, --help         Show this help

Environment:
  PIPER_DOCKER_IMAGE     Override Docker image
  PIPER_SDK_CONTAINER    Override container name
  CAN_NAME               Override CAN interface
  SPEED_PERCENT          Override default speed percent
  SKIP_INSTALL=1         Skip pip install

Examples:
  tools/piper_sdk_docker.sh --can can0 --speed 5
  tools/piper_sdk_docker.sh --can can_piper -- --no-slave
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

if [[ "${SKIP_INSTALL}" != "1" ]]; then
  sdk_site=/tmp/piper-sdk-site
  mkdir -p "${sdk_site}"
  python3 -m pip install --upgrade --target "${sdk_site}" piper_sdk python-can
  export PYTHONPATH="${sdk_site}:${PYTHONPATH:-}"
else
  if [[ -d /tmp/piper-sdk-site ]]; then
    export PYTHONPATH="/tmp/piper-sdk-site:${PYTHONPATH:-}"
  fi
fi

extra_args=()
if [[ -n "${EXTRA_ARGS:-}" ]]; then
  eval "extra_args=(${EXTRA_ARGS})"
fi

exec /surf/tools/piper_sdk_console.py --can "${CAN_NAME}" --speed "${SPEED_PERCENT}" "${extra_args[@]}"
EOS
)

run_inside_existing() {
  docker exec "${tty_args[@]}" \
    -e CAN_NAME="${CAN_NAME}" \
    -e SPEED_PERCENT="${SPEED_PERCENT}" \
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
  -e SKIP_INSTALL="${SKIP_INSTALL}" \
  -e EXTRA_ARGS="${quoted_extra}" \
  -v "${SURF_ROOT}:/surf" \
  -w /surf \
  "${DOCKER_IMAGE}" \
  bash -lc "${inner_script}"
