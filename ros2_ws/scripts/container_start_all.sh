#!/usr/bin/env bash
#
# container_start_all.sh — 一个终端搞定所有三个进程：
#   1. jaka launch  (后台，日志 → /tmp/launch.log)
#   2. 等待 MoveGroup ready（保持完整 OctoMap 碰撞检测）
#   3. 等待 initial_scan 全部 6 个方向扫完（可关掉，见 WAIT_INITIAL_SCAN）
#   4. TSP 节点     (前台，Ctrl+C 停整套)
#
# 用法（容器内）：
#   bash /root/ros2_ws/scripts/container_start_all.sh
#
# 环境变量：
#   GAZEBO_GUI=true|false       是否显示 Gazebo GUI，默认 false
#   WORLD_FILE=<sdf>            默认 exploration_scene_shelf.sdf（书架）
#   WAIT_INITIAL_SCAN=true|false  TSP 起来前是否等 initial_scan 走完，默认 true
#   INITIAL_SCAN_WAIT_SEC=<sec>   等 initial_scan 的超时，默认 600
#   START_SEMANTIC=true|false     是否启动 SegFormer 语义分割节点，默认 false
#
# 如果只想跑 launch，另开终端手动跑 TSP，用 container_start_jaka.sh 即可。

set -euo pipefail

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
WS="$(readlink -f "${SCRIPT_DIR}/..")"
MOVE_GROUP_WAIT_SEC="${MOVE_GROUP_WAIT_SEC:-60}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
WORLD_FILE="${WORLD_FILE:-exploration_scene_shelf.sdf}"
WAIT_INITIAL_SCAN="${WAIT_INITIAL_SCAN:-true}"
INITIAL_SCAN_WAIT_SEC="${INITIAL_SCAN_WAIT_SEC:-600}"
START_SEMANTIC="${START_SEMANTIC:-false}"

echo "==> Killing residual ROS processes..."
pkill -9 -f gz             || true
pkill -9 -f rviz           || true
pkill -9 -f robot_state_publisher || true
pkill -9 -f parameter_bridge      || true
pkill -9 -f move_group     || true
pkill -9 -f spawner        || true
pkill -9 -f octomap        || true
pkill -9 -f motion_planner || true
pkill -9 -f known_free_validator || true
pkill -9 -f motion_metrics_reporter || true
pkill -9 -f environment_bridge   || true
pkill -9 -f decision_bridge      || true
pkill -9 -f bootstrap      || true
pkill -9 -f octomap_to_json      || true
pkill -9 -f image_relay    || true
pkill -9 -f self_filter    || true
pkill -9 -f semantic_inference_node || true
pkill -9 -f hppt_boundary_ros    || true
sleep 3

echo "==> Sourcing workspace..."
# ROS setup 脚本会引用未定义变量；临时关掉 nounset 只在 source 期间
set +u
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
set -u

echo "==> [1/3] Launching jaka stack in background (gazebo_gui=${GAZEBO_GUI}, world_file=${WORLD_FILE}, start_semantic=${START_SEMANTIC}, log → /tmp/launch.log)"
rm -f /tmp/launch.log
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
    auto_bootstrap:=false gazebo_gui:="${GAZEBO_GUI}" \
    world_file:="${WORLD_FILE}" start_semantic:="${START_SEMANTIC}" > /tmp/launch.log 2>&1 &
JAKA_PID=$!
echo "    jaka launch pid=${JAKA_PID}"

# 停止时把 jaka 也一起收掉
cleanup() {
    echo ""
    echo "==> Stopping everything..."
    kill "${JAKA_PID}"       2>/dev/null || true
    pkill -9 -f gz           2>/dev/null || true
    pkill -9 -f rviz         2>/dev/null || true
    pkill -9 -f move_group   2>/dev/null || true
    pkill -9 -f octomap      2>/dev/null || true
    pkill -9 -f motion_planner        2>/dev/null || true
    pkill -9 -f known_free_validator  2>/dev/null || true
    pkill -9 -f motion_metrics_reporter 2>/dev/null || true
    pkill -9 -f environment_bridge    2>/dev/null || true
    pkill -9 -f decision_bridge       2>/dev/null || true
    pkill -9 -f self_filter           2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f parameter_bridge      2>/dev/null || true
    pkill -9 -f spawner               2>/dev/null || true
    pkill -9 -f octomap_to_json       2>/dev/null || true
    pkill -9 -f image_relay           2>/dev/null || true
    pkill -9 -f semantic_inference_node 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo "==> [2/3] Waiting for move_group (timeout ${MOVE_GROUP_WAIT_SEC}s)..."
MOVE_GROUP_READY=false
for ((i = 0; i < MOVE_GROUP_WAIT_SEC; i++)); do
    if ! kill -0 "${JAKA_PID}" 2>/dev/null; then
        echo "ERROR: jaka launch died. Last 40 lines of /tmp/launch.log:"
        tail -n 40 /tmp/launch.log
        exit 1
    fi
    if ros2 action list 2>/dev/null | grep -qx "/move_action"; then
        MOVE_GROUP_READY=true
        break
    fi
    sleep 1
done
if [ "${MOVE_GROUP_READY}" != "true" ]; then
    echo "ERROR: /move_action was not ready after ${MOVE_GROUP_WAIT_SEC}s."
    tail -n 40 /tmp/launch.log
    exit 1
fi
echo "==> [2/4] move_group is ready; full OctoMap collision checking is enabled."

if [ "${WAIT_INITIAL_SCAN}" = "true" ]; then
    echo "==> [3/4] Waiting for initial_scan to finish all 6 directions (timeout ${INITIAL_SCAN_WAIT_SEC}s)..."
    INITIAL_SCAN_DONE=false
    for ((i = 0; i < INITIAL_SCAN_WAIT_SEC; i++)); do
        if ! kill -0 "${JAKA_PID}" 2>/dev/null; then
            echo "ERROR: jaka launch died while waiting for initial_scan. Last 40 lines of /tmp/launch.log:"
            tail -n 40 /tmp/launch.log
            exit 1
        fi
        # initial_scan_node prints this exact line when done (see initial_scan_node.py:316-318)
        if grep -q "Initial scan complete:" /tmp/launch.log 2>/dev/null; then
            INITIAL_SCAN_DONE=true
            break
        fi
        sleep 2
    done
    if [ "${INITIAL_SCAN_DONE}" != "true" ]; then
        echo "WARN: initial_scan did not report completion within ${INITIAL_SCAN_WAIT_SEC}s; starting TSP anyway."
    else
        echo "==> [3/4] initial_scan completed. Extract from log:"
        grep "Initial scan complete:" /tmp/launch.log | tail -1
    fi
else
    echo "==> [3/4] WAIT_INITIAL_SCAN=false → skipping wait (TSP will run in parallel with initial_scan)."
fi

echo ""
echo "==> [4/4] Starting TSP node in foreground. Ctrl+C to stop everything."
echo "    (jaka launch output at /tmp/launch.log; 'tail -f /tmp/launch.log' in another shell)"
echo ""
ros2 run hppt_boundary_ros hppt_boundary_ros "$@"

# TSP 正常退出后也要清理 jaka
cleanup
