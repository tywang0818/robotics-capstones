#!/usr/bin/env bash
#
# container_start_all_realsense.sh — 一个终端启动真实 RealSense 建图 + HPP-PT。
#
# 数据链路：
#   physical D435 point cloud
#     -> octomap_server (/octomap_binary, frame=world)
#     -> octomap_to_json_bridge (/team/optimized_environment_json)
#     -> environment_bridge / decision_bridge
#     -> motion_planner / MoveIt PlanningScene
#     -> hppt_boundary_ros 根据同一份 /octomap_binary 选 frontier viewpoint
#
# 用法（容器内）：
#   bash /root/ros2_ws/scripts/container_start_all_realsense.sh
#
# 环境变量：
#   RESOLUTION=<m>             OctoMap 分辨率，默认 0.03
#   MAX_RANGE=<m>              RealSense raycast 最大距离，默认 2.0
#   ENVIRONMENT_RATE=<hz>      OctoMap 转 JSON 环境发布频率，默认 1.0
#   RVIZ=true|false            是否启动观察用 RViz，默认 true
#   MOVE_GROUP_WAIT_SEC=<sec>  等 MoveIt /move_action 的超时，默认 60
#   CAMERA_WAIT_SEC=<sec>      等 RealSense point cloud topic 的超时，默认 30
#   ENV_WAIT_SEC=<sec>         等非空环境被 bridge 接收的超时，默认 45
#   SAVE_OCTOMAP=true|false    是否周期性保存 OctoMap，默认 false
#   SAVE_INTERVAL=<sec>        保存间隔，默认 60.0
#   SAVE_DIRECTORY=<path>      保存目录，默认 /root/ros2_ws/maps
#
# 注意：这个模式使用真实 D435 的点云，但 motion planning 仍跑 mock JAKA。
# 如果真实相机没有和 mock tool0 做同样运动，world-frame 累积地图会几何失真。

set -euo pipefail

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
WS="$(readlink -f "${SCRIPT_DIR}/..")"

RESOLUTION="${RESOLUTION:-0.03}"
MAX_RANGE="${MAX_RANGE:-2.0}"
ENVIRONMENT_RATE="${ENVIRONMENT_RATE:-1.0}"
RVIZ="${RVIZ:-true}"
MOVE_GROUP_WAIT_SEC="${MOVE_GROUP_WAIT_SEC:-60}"
CAMERA_WAIT_SEC="${CAMERA_WAIT_SEC:-30}"
ENV_WAIT_SEC="${ENV_WAIT_SEC:-45}"
SAVE_OCTOMAP="${SAVE_OCTOMAP:-false}"
SAVE_INTERVAL="${SAVE_INTERVAL:-60.0}"
SAVE_DIRECTORY="${SAVE_DIRECTORY:-/root/ros2_ws/maps}"

echo "==> Killing residual ROS / RealSense processes..."
pkill -9 -f gz                         || true
pkill -9 -f rviz                       || true
pkill -9 -f robot_state_publisher      || true
pkill -9 -f parameter_bridge           || true
pkill -9 -f move_group                 || true
pkill -9 -f spawner                    || true
pkill -9 -f octomap                    || true
pkill -9 -f motion_planner             || true
pkill -9 -f environment_bridge         || true
pkill -9 -f decision_bridge            || true
pkill -9 -f octomap_to_json            || true
pkill -9 -f image_relay                || true
pkill -9 -f self_filter                || true
pkill -9 -f hppt_boundary_ros          || true
pkill -9 -f realsense2_camera          || true
pkill -9 -f static_transform_publisher || true
sleep 3

echo "==> Sourcing workspace..."
set +u
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
set -u

echo "==> [1/4] Launching RealSense + mock JAKA + MoveIt + OctoMap (log -> /tmp/realsense_launch.log)"
rm -f /tmp/realsense_launch.log
ros2 launch jaka_exploration_core realsense_virtual_arm_moveit_octomap.launch.py \
    start_hppt:=false \
    rviz:="${RVIZ}" \
    resolution:="${RESOLUTION}" \
    max_range:="${MAX_RANGE}" \
    environment_rate:="${ENVIRONMENT_RATE}" \
    save_octomap:="${SAVE_OCTOMAP}" \
    save_interval:="${SAVE_INTERVAL}" \
    save_directory:="${SAVE_DIRECTORY}" \
    > /tmp/realsense_launch.log 2>&1 &
LAUNCH_PID=$!
echo "    launch pid=${LAUNCH_PID}"

cleanup() {
    echo ""
    echo "==> Stopping everything..."
    kill "${LAUNCH_PID}" 2>/dev/null || true
    pkill -9 -f hppt_boundary_ros          2>/dev/null || true
    pkill -9 -f realsense2_camera          2>/dev/null || true
    pkill -9 -f static_transform_publisher 2>/dev/null || true
    pkill -9 -f octomap                    2>/dev/null || true
    pkill -9 -f octomap_to_json            2>/dev/null || true
    pkill -9 -f motion_planner             2>/dev/null || true
    pkill -9 -f environment_bridge         2>/dev/null || true
    pkill -9 -f decision_bridge            2>/dev/null || true
    pkill -9 -f move_group                 2>/dev/null || true
    pkill -9 -f rviz                       2>/dev/null || true
    pkill -9 -f robot_state_publisher      2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

echo "==> [2/4] Waiting for move_group (timeout ${MOVE_GROUP_WAIT_SEC}s)..."
MOVE_GROUP_READY=false
for ((i = 0; i < MOVE_GROUP_WAIT_SEC; i++)); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "ERROR: launch died. Last 40 lines of /tmp/realsense_launch.log:"
        tail -n 40 /tmp/realsense_launch.log
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
    tail -n 40 /tmp/realsense_launch.log
    exit 1
fi
echo "==> [2/4] move_group is ready."

echo "==> [3/4] Waiting for RealSense point cloud topic (timeout ${CAMERA_WAIT_SEC}s)..."
CAMERA_READY=false
for ((i = 0; i < CAMERA_WAIT_SEC; i++)); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "ERROR: launch died while waiting for RealSense. Last 40 lines:"
        tail -n 40 /tmp/realsense_launch.log
        exit 1
    fi
    if ros2 topic list 2>/dev/null | grep -qx "/camera/camera/depth/color/points"; then
        CAMERA_READY=true
        break
    fi
    sleep 1
done
if [ "${CAMERA_READY}" != "true" ]; then
    echo "WARN: RealSense point cloud topic was not visible after ${CAMERA_WAIT_SEC}s."
    echo "      HPP-PT will start, but OctoMap may stay empty until the camera publishes."
else
    echo "==> [3/4] RealSense point cloud topic is available."
fi

echo "==> [4/4] Waiting for environment bridge to accept a non-empty OctoMap (timeout ${ENV_WAIT_SEC}s)..."
ENV_READY=false
for ((i = 0; i < ENV_WAIT_SEC; i++)); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "ERROR: launch died while waiting for environment. Last 40 lines:"
        tail -n 40 /tmp/realsense_launch.log
        exit 1
    fi
    if timeout 2s ros2 topic echo /team/environment_status_json --once 2>/dev/null | grep -Eq '"accepted"[[:space:]]*:[[:space:]]*true'; then
        ENV_READY=true
        break
    fi
    sleep 1
done
if [ "${ENV_READY}" != "true" ]; then
    echo "WARN: no accepted environment observed within ${ENV_WAIT_SEC}s; starting HPP-PT anyway."
    echo "      Check /tmp/realsense_launch.log and /camera/camera/depth/color/points if no tasks publish."
else
    echo "==> [4/4] RealSense OctoMap is flowing into the planning environment."
fi

echo ""
echo "==> Starting HPP-PT in foreground. Ctrl+C to stop everything."
echo "    Launch output: /tmp/realsense_launch.log"
echo ""
ros2 run hppt_boundary_ros hppt_boundary_ros "$@"

cleanup
