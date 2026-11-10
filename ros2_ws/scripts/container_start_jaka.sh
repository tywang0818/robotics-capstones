#!/usr/bin/env bash
#
# container_start_jaka.sh — 在容器内一键启动 JAKA 仿真栈。
#
# 时序：
#   1. pkill 干净
#   2. 前台 launch，把日志同时写 /tmp/launch.log
#   3. Ctrl+C 结束整个 launch
#
# 用法（容器内）：
#   bash /root/ros2_ws/scripts/container_start_jaka.sh
#
# 需要单独启动 TSP 节点（另一个终端）：
#   ros2 run hppt_boundary_ros hppt_boundary_ros

set -euo pipefail

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
WS="$(readlink -f "${SCRIPT_DIR}/..")"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
WORLD_FILE="${WORLD_FILE:-exploration_scene.sdf}"

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
sleep 3

echo "==> Sourcing workspace..."
set +u
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
set -u

rm -f /tmp/launch.log
echo "==> Starting jaka launch (auto_bootstrap:=false, gazebo_gui=${GAZEBO_GUI}, world_file=${WORLD_FILE}, output → /tmp/launch.log)"
echo "    Ctrl+C to stop everything."
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
    auto_bootstrap:=false gazebo_gui:="${GAZEBO_GUI}" \
    world_file:="${WORLD_FILE}" 2>&1 | tee /tmp/launch.log
