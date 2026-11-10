#!/usr/bin/env bash
#
# container_deploy.sh — 在容器内为当前 workspace 安装依赖并执行 colcon build。
#
# 用法（容器内执行）：
#   bash /root/ros2_ws/scripts/container_deploy.sh

set -euo pipefail

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
WS="$(readlink -f "${SCRIPT_DIR}/..")"

echo "==> Workspace: ${WS}"

if [ ! -d "${WS}/src/hppt_boundary_ros" ]; then
    echo "ERROR: ${WS}/src/hppt_boundary_ros not found."
    exit 1
fi
if [ ! -d "${WS}/source" ] || [ ! -d "${WS}/include" ]; then
    echo "ERROR: cppraplab source/ or include/ missing in ${WS}."
    exit 1
fi
if [ ! -d "${WS}/src/jaka_exploration_core" ]; then
    echo "ERROR: ${WS}/src/jaka_exploration_core not found."
    exit 1
fi

echo "==> Installing apt dependencies (idempotent)..."
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-octomap-msgs ros-jazzy-moveit-msgs \
    liboctomap-dev libeigen3-dev

echo "==> Building the complete ROS 2 workspace..."
cd "${WS}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build

echo ""
echo "==> Deploy done."
echo "    Next: source ${WS}/install/setup.bash"
echo "          bash ${SCRIPT_DIR}/container_start_all.sh"
