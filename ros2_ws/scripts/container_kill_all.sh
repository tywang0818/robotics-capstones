#!/usr/bin/env bash
#
# container_kill_all.sh — 在容器内跑，杀干净整套栈的所有节点。
#
# 用法（容器内）：
#   bash /root/ros2_ws/scripts/container_kill_all.sh

echo "==> Killing all ROS/Gazebo processes..."
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
pkill -9 -f hppt_boundary_ros    || true
sleep 2
echo "==> Done."
