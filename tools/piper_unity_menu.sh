#!/usr/bin/env bash
set -euo pipefail

SURF_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PIPER_ROS_ROOT="${PIPER_ROS_ROOT:-/home/Light/Projects/Osw_Surf_VR/code/piper_ros}"
UNITY_ROS_IP="${UNITY_ROS_IP:-127.0.0.1}"
UNITY_ROS_PORT="${UNITY_ROS_PORT:-10000}"
SPEED_PERCENT="${SPEED_PERCENT:-30}"
GRIPPER_METERS="${GRIPPER_METERS:-0.0}"
GRIPPER_EFFORT="${GRIPPER_EFFORT:-1.0}"
PIPER_CAN_NAME="${PIPER_CAN_NAME:-can_piper}"
REAL_ARM_SPEED_PERCENT="${REAL_ARM_SPEED_PERCENT:-5}"
REAL_ARM_MAX_SPEED_DEG="${REAL_ARM_MAX_SPEED_DEG:-8}"
REAL_ARM_DRY_RUN="${REAL_ARM_DRY_RUN:-0}"
SKIP_SDK_INSTALL="${SKIP_SDK_INSTALL:-0}"

declare -a CHILD_PIDS=()

cleanup() {
  for pid in "${CHILD_PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

run_docker() {
  if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
    xhost +local:docker >/dev/null 2>&1 || true
  fi

  local tty_args=(-i)
  if [[ -t 0 && -t 1 ]]; then
    tty_args=(-it)
  fi

  if docker ps --format '{{.Names}}' | grep -qx 'piper_unity_humble'; then
    docker exec "${tty_args[@]}" \
      -e UNITY_ROS_IP="${UNITY_ROS_IP}" \
      -e UNITY_ROS_PORT="${UNITY_ROS_PORT}" \
      -w /ws/piper_ros \
      piper_unity_humble \
      bash -lc "/surf/tools/piper_unity_menu.sh --inside"
    return
  fi

  docker run --rm "${tty_args[@]}" \
    --name piper_unity_humble \
    --network host \
    --privileged \
    --ipc host \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e UNITY_ROS_IP="${UNITY_ROS_IP}" \
    -e UNITY_ROS_PORT="${UNITY_ROS_PORT}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${SURF_ROOT}:/surf" \
    -v "${PIPER_ROS_ROOT}:/ws/piper_ros" \
    -w /ws/piper_ros \
    piper-ros:humble \
    bash -lc "/surf/tools/piper_unity_menu.sh --inside"
}

source_ros() {
  set +u
  if [[ -x /opt/ros/humble/bin/ros2 && -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi

  if command -v ros2 >/dev/null 2>&1 && [[ -f "${PIPER_ROS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${PIPER_ROS_ROOT}/install/setup.bash"
  fi
  set -u
}

require_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not found. Use: ${0} --docker"
    return 1
  fi
}

publish_enable() {
  local value="$1"
  ros2 topic pub --once /enable_cmd std_msgs/msg/Bool "{data: ${value}}"
}

publish_joint_command() {
  local position_csv="$1"
  ros2 topic pub --once /joint_ctrl_cmd sensor_msgs/msg/JointState \
    "{name: [joint1, joint2, joint3, joint4, joint5, joint6, gripper], position: [${position_csv}, ${GRIPPER_METERS}], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ${SPEED_PERCENT}], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ${GRIPPER_EFFORT}]}"
}

home_arm() {
  publish_joint_command "0.0, 0.0, 0.0, 0.0, 0.0, 0.0"
}

set_all_joints_degrees() {
  read -r -p "输入 6 个关节角度，单位 deg，用空格分隔: " line
  python3 - "$line" <<'PY'
import math
import sys

parts = sys.argv[1].split()
if len(parts) != 6:
    raise SystemExit("需要刚好 6 个角度")
values = [str(math.radians(float(part))) for part in parts]
print(", ".join(values))
PY
}

start_ros_tcp_endpoint() {
  if ! ros2 pkg prefix ros_tcp_endpoint >/dev/null 2>&1; then
    echo "缺少 ros_tcp_endpoint。请把 Unity Robotics ROS-TCP-Endpoint 加到 Humble 工作区后重新 colcon build。"
    echo "Unity 侧默认连 ${UNITY_ROS_IP}:${UNITY_ROS_PORT}。"
    return 1
  fi

  ros2 run ros_tcp_endpoint default_server_endpoint \
    --ros-args -p ROS_IP:="${UNITY_ROS_IP}" -p ROS_TCP_PORT:="${UNITY_ROS_PORT}" &
  CHILD_PIDS+=("$!")
  echo "ROS TCP Endpoint 已启动: ${UNITY_ROS_IP}:${UNITY_ROS_PORT} pid=${CHILD_PIDS[-1]}"
}

start_moveit() {
  _start_moveit_common piper_moveit.launch.py "MoveIt（含 RViz）"
}

start_moveit_headless() {
  _start_moveit_common move_group.launch.py "MoveIt move_group（无 RViz）"
}

_start_moveit_common() {
  local launch_file="$1"
  local label="$2"
  local package="piper_with_gripper_moveit"
  local log_file="/tmp/piper_moveit_$(date +%Y%m%d_%H%M%S).log"
  read -r -p "启动有夹爪 MoveIt? [Y/n] " answer
  if [[ "${answer,,}" == "n" ]]; then
    package="piper_no_gripper_moveit"
  fi

  if ! pgrep -f piper_moveit_to_unity_bridge >/dev/null 2>&1; then
    start_moveit_to_unity_bridge
    sleep 1
  fi

  LC_NUMERIC=en_US.UTF-8 ros2 launch "${package}" "${launch_file}" use_sim_time:=false >"${log_file}" 2>&1 &
  CHILD_PIDS+=("$!")
  echo "${label} 已启动: ${package}/${launch_file} pid=${CHILD_PIDS[-1]}"
  echo "日志: ${log_file}"
  if [[ "${launch_file}" == "piper_moveit.launch.py" ]]; then
    echo "如果 RViz 窗口没有出现，先看日志: tail -f ${log_file}"
  else
    echo "查看日志: tail -f ${log_file}"
  fi
}

start_moveit_to_unity_bridge() {
  python3 "${SURF_ROOT}/tools/piper_moveit_to_unity_bridge.py" &
  CHILD_PIDS+=("$!")
  echo "MoveIt -> Unity action bridge 已启动 pid=${CHILD_PIDS[-1]}"
}

ensure_piper_sdk_pythonpath() {
  if [[ "${SKIP_SDK_INSTALL}" == "1" ]]; then
    if [[ -d /tmp/piper-sdk-site ]]; then
      export PYTHONPATH="/tmp/piper-sdk-site:${PYTHONPATH:-}"
    fi
    return
  fi

  local sdk_site="/tmp/piper-sdk-site"
  mkdir -p "${sdk_site}"
  python3 -m pip install --upgrade --target "${sdk_site}" piper_sdk python-can
  export PYTHONPATH="${sdk_site}:${PYTHONPATH:-}"
}

start_real_arm_smooth_bridge() {
  local can_name speed max_speed dry_run_args=()

  read -r -p "CAN 接口名 [${PIPER_CAN_NAME}]: " can_name
  can_name="${can_name:-${PIPER_CAN_NAME}}"

  read -r -p "真机速度百分比 [${REAL_ARM_SPEED_PERCENT}]: " speed
  speed="${speed:-${REAL_ARM_SPEED_PERCENT}}"

  read -r -p "最大关节速度 deg/s [${REAL_ARM_MAX_SPEED_DEG}]: " max_speed
  max_speed="${max_speed:-${REAL_ARM_MAX_SPEED_DEG}}"

  if [[ "${REAL_ARM_DRY_RUN}" == "1" ]]; then
    dry_run_args=(--dry-run)
  else
    read -r -p "只 dry-run 不发真机命令? [y/N] " answer
    if [[ "${answer,,}" == "y" ]]; then
      dry_run_args=(--dry-run)
    fi
  fi

  ensure_piper_sdk_pythonpath

  python3 "${SURF_ROOT}/tools/piper_sdk_bridge.py" \
    --can "${can_name}" \
    --speed "${speed}" \
    --max-speed-deg "${max_speed}" \
    "${dry_run_args[@]}" &
  CHILD_PIDS+=("$!")

  echo "真机平滑桥已启动 pid=${CHILD_PIDS[-1]}"
  echo "订阅 /joint_ctrl_cmd 和 /enable_cmd，发布 /joint_states_feedback、/joint_states、/arm_status、/link6_pose。"
  echo "注意：真机桥运行时，不要同时让 Unity 仿真发布 /joint_states_feedback。"
}

show_topics() {
  ros2 topic list | grep -E '(^/joint_ctrl_cmd$|^/joint_states_feedback$|^/joint_states$|^/arm_status$|^/enable_cmd$|^/link6_pose$)' || true
}

echo_status_once() {
  echo "等待 /arm_status，一次后返回..."
  ros2 topic echo --once /arm_status piper_msgs/msg/PiperStatusMsg
}

monitor_topic() {
  local topic="$1"
  local msg_type="${2:-}"

  echo
  echo "实时监看 ${topic}"
  echo "按 Ctrl+C 停止监看并返回菜单。"
  echo

  set +e
  if [[ -n "${msg_type}" ]]; then
    ros2 topic echo "${topic}" "${msg_type}"
  else
    ros2 topic echo "${topic}"
  fi
  set -e
}

monitor_topic_hz() {
  local topic="$1"

  echo
  echo "实时频率 ${topic}"
  echo "按 Ctrl+C 停止监看并返回菜单。"
  echo

  set +e
  ros2 topic hz "${topic}"
  set -e
}

find_free_port() {
  local preferred="$1"
  python3 - "${preferred}" <<'PY'
import socket
import sys

port = int(sys.argv[1])
for candidate in range(port, port + 100):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", candidate))
        except OSError:
            continue
        print(candidate)
        raise SystemExit(0)

raise SystemExit(f"no free port found from {port} to {port + 99}")
PY
}

start_web_pose_server() {
  local host="${WEB_POSE_HOST:-0.0.0.0}"
  local requested_port="${WEB_POSE_PORT:-8765}"
  local port

  port="$(find_free_port "${requested_port}")"
  if [[ "${port}" != "${requested_port}" ]]; then
    echo "端口 ${requested_port} 已被占用，改用 ${port}。"
  fi

  python3 "${SURF_ROOT}/tools/piper_web_pose_server.py" --host "${host}" --port "${port}" &
  CHILD_PIDS+=("$!")
  echo "Web 位姿控制服务器已启动: http://${host}:${port} pid=${CHILD_PIDS[-1]}"
  echo "本机浏览器通常打开: http://localhost:${port}"
  echo "保持这个菜单窗口打开；退出菜单会停止 Web 服务。"
  echo "页面会检查 /link6_pose、/joint_states、/joint_states_feedback、/arm_status。"
  echo "如需指定端口，可设置环境变量 WEB_POSE_PORT，例如 WEB_POSE_PORT=8766 tools/piper_unity_menu.sh"
}

start_fk_publisher() {
  python3 "${SURF_ROOT}/tools/piper_fk_publisher.py" &
  CHILD_PIDS+=("$!")
  echo "FK 坐标发布器已启动 pid=${CHILD_PIDS[-1]}"
  echo "发布 /link6_pose 话题 (base_link → link6 正运动学)。"
}

run_pose_planner_interactive() {
  local x y z roll pitch yaw planner

  read -r -p "输入目标位置 x y z (米)，用空格分隔: " x y z
  if [[ -z "${x}" || -z "${y}" || -z "${z}" ]]; then
    echo "需要提供 x y z 三个坐标"
    return 1
  fi

  read -r -p "输入姿态 roll pitch yaw (度)，用空格分隔，默认 0 0 0: " roll pitch yaw
  roll="${roll:-0.0}"
  pitch="${pitch:-0.0}"
  yaw="${yaw:-0.0}"

  read -r -p "规划器 (默认 RRT): " planner
  planner="${planner:-RRT}"

  python3 "${SURF_ROOT}/tools/piper_moveit_pose_planner.py" \
    --x "${x}" --y "${y}" --z "${z}" \
    --roll "${roll}" --pitch "${pitch}" --yaw "${yaw}" \
    --planner "${planner}"
}

monitor_custom_topic() {
  local topic
  local msg_type

  read -r -p "输入 topic 名，比如 /joint_states_feedback: " topic
  if [[ -z "${topic}" ]]; then
    echo "topic 不能为空"
    return
  fi

  read -r -p "输入消息类型，可留空自动识别，比如 sensor_msgs/msg/JointState: " msg_type
  monitor_topic "${topic}" "${msg_type}"
}

monitor_menu() {
  while true; do
    echo
    echo "实时监看:"
    echo "  1) /arm_status"
    echo "  2) /joint_states"
    echo "  3) /joint_states_feedback"
    echo "  4) /link6_pose"
    echo "  5) /joint_ctrl_cmd"
    echo "  6) /enable_cmd"
    echo "  7) /joint_states 频率"
    echo "  8) 自定义 topic"
    echo "  b) 返回主菜单"
    read -r -p "> " choice

    case "${choice}" in
      1) monitor_topic /arm_status piper_msgs/msg/PiperStatusMsg ;;
      2) monitor_topic /joint_states sensor_msgs/msg/JointState ;;
      3) monitor_topic /joint_states_feedback sensor_msgs/msg/JointState ;;
      4) monitor_topic /link6_pose geometry_msgs/msg/PoseStamped ;;
      5) monitor_topic /joint_ctrl_cmd sensor_msgs/msg/JointState ;;
      6) monitor_topic /enable_cmd std_msgs/msg/Bool ;;
      7) monitor_topic_hz /joint_states ;;
      8) monitor_custom_topic ;;
      b|B|back) break ;;
      *) echo "未知选项" ;;
    esac
  done
}

arm_control_menu() {
  while true; do
    echo
    echo "机械臂控制:"
    echo "  1) 使能"
    echo "  2) 失能"
    echo "  3) 回零"
    echo "  4) 设置速度百分比"
    echo "  5) 设置 6 关节角度 deg"
    echo "  b) 返回主菜单"
    read -r -p "> " choice

    case "${choice}" in
      1) publish_enable true ;;
      2) publish_enable false ;;
      3) home_arm ;;
      4)
        read -r -p "速度百分比 1-100: " SPEED_PERCENT
        echo "速度已设为 ${SPEED_PERCENT}%"
        ;;
      5)
        radians="$(set_all_joints_degrees)"
        publish_joint_command "${radians}"
        ;;
      b|B|back) break ;;
      *) echo "未知选项" ;;
    esac
  done
}

debug_menu() {
  while true; do
    echo
    echo "检查/调试:"
    echo "  1) 查看一次 arm_status"
    echo "  2) 查看关键 Piper topics"
    echo "  3) 实时监看 topic 值"
    echo "  b) 返回主菜单"
    read -r -p "> " choice

    case "${choice}" in
      1) echo_status_once ;;
      2) show_topics ;;
      3) monitor_menu ;;
      b|B|back) break ;;
      *) echo "未知选项" ;;
    esac
  done
}

service_menu() {
  while true; do
    echo
    echo "服务/高级启动:"
    echo "  1) ROS TCP Endpoint（Unity 连接）"
    echo "  2) MoveIt RViz（自动带 action bridge）"
    echo "  3) MoveIt move_group 无 RViz（自动带 action bridge）"
    echo "  4) MoveIt -> Unity action bridge"
    echo "  5) FK 坐标发布器（/link6_pose）"
    echo "  6) Web 坐标控制页面"
    echo "  7) 真机平滑桥（piper_sdk -> CAN）"
    echo "  b) 返回主菜单"
    read -r -p "> " choice

    case "${choice}" in
      1) start_ros_tcp_endpoint ;;
      2) start_moveit ;;
      3) start_moveit_headless ;;
      4) start_moveit_to_unity_bridge ;;
      5) start_fk_publisher ;;
      6) start_web_pose_server ;;
      7) start_real_arm_smooth_bridge ;;
      b|B|back) break ;;
      *) echo "未知选项" ;;
    esac
  done
}

quick_start() {
  echo "启动基础链路：Endpoint + action bridge + FK。"
  echo "Endpoint 启动后请让 Unity 进入 Play；如果 Unity 已连接失败，停止后重新 Play。"
  start_ros_tcp_endpoint
  start_moveit_to_unity_bridge
  start_fk_publisher
}

menu_loop() {
  require_ros2
  echo "Piper Unity Simulation ROS TCP 菜单"
  echo "piper_ros: ${PIPER_ROS_ROOT}"
  echo "Unity ROS TCP: ${UNITY_ROS_IP}:${UNITY_ROS_PORT}"

  while true; do
    echo
    echo "常用顺序：1 -> Unity Play -> 2 或 3"
    echo
    echo "  1) 快速启动基础链路"
    echo "  2) 启动 MoveIt RViz"
    echo "  3) 启动 Web 坐标控制页面"
    echo "  4) MoveIt RRT 位姿规划"
    echo "  5) 机械臂控制"
    echo "  6) 检查/调试"
    echo "  7) 服务/高级启动"
    echo "  8) 启动真机平滑桥"
    echo "  q) 退出"
    read -r -p "> " choice

    case "${choice}" in
      1) quick_start ;;
      2) start_moveit ;;
      3) start_web_pose_server ;;
      4) run_pose_planner_interactive ;;
      5) arm_control_menu ;;
      6) debug_menu ;;
      7) service_menu ;;
      8) start_real_arm_smooth_bridge ;;
      q|Q|quit|exit) break ;;
      *) echo "未知选项" ;;
    esac
  done
}

case "${1:-}" in
  --docker)
    run_docker
    ;;
  --inside)
    PIPER_ROS_ROOT="/ws/piper_ros"
    source_ros
    menu_loop
    ;;
  *)
    source_ros
    menu_loop
    ;;
esac
