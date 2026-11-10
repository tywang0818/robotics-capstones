#!/usr/bin/env bash
#
# test_shelf_coverage.sh — 3-minute shelf exploration coverage test.
#
# Launches exploration_stage2 with the RealShelf world, runs the HPP-PT
# planner, and streams shelf-bbox coverage samples from
# multi_coverage_monitor. After DURATION_SEC (default 180s) the monitor
# prints a FINAL line and exits; this script then tears down the stack.
#
# Usage (inside container):
#   bash /root/ros2_ws/scripts/test_shelf_coverage.sh
#
# Env vars:
#   MODE                     hppt|random, default hppt
#                              hppt   -> ros2 run hppt_boundary_ros hppt_boundary_ros
#                              random -> ros2 run jaka_exploration_core random_frontier_selector
#   DURATION_SEC             test window seconds, default 180 (3 min)
#   SAMPLE_PERIOD_SEC        coverage log period, default 10
#   MOVE_GROUP_WAIT_SEC      wait for /move_action, default 60
#   INITIAL_SCAN_WAIT_SEC    wait for initial scan, default 600
#   WAIT_INITIAL_SCAN        true|false, default true
#   GAZEBO_GUI               true|false, default false
#   RANDOM_SEED              int; -1 (default) = system entropy (random mode only)
#   LOG_DIR                  where to place logs, default /tmp
#
# Bbox is fixed to the strict shelf AABB in world frame:
#   x=[0.80, 1.19], y=[-0.42, 0.42], z=[0, 1.11]
# derived from exploration_scene_multi.sdf. Note: shelf collision is a
# solid box, so max achievable coverage is a thin surface shell.

set -euo pipefail

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
WS="$(readlink -f "${SCRIPT_DIR}/..")"

MODE="${MODE:-hppt}"
DURATION_SEC="${DURATION_SEC:-180}"
SAMPLE_PERIOD_SEC="${SAMPLE_PERIOD_SEC:-10}"
MOVE_GROUP_WAIT_SEC="${MOVE_GROUP_WAIT_SEC:-60}"
INITIAL_SCAN_WAIT_SEC="${INITIAL_SCAN_WAIT_SEC:-600}"
WAIT_INITIAL_SCAN="${WAIT_INITIAL_SCAN:-true}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
RANDOM_SEED="${RANDOM_SEED:--1}"
LOG_DIR="${LOG_DIR:-/tmp}"

case "${MODE}" in
    hppt|random) ;;
    *) echo "ERROR: MODE must be 'hppt' or 'random' (got '${MODE}')"; exit 2;;
esac

STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="${LOG_DIR}/multi_test_${MODE}_launch_${STAMP}.log"
SELECTOR_LOG="${LOG_DIR}/multi_test_${MODE}_selector_${STAMP}.log"
MONITOR_LOG="${LOG_DIR}/multi_test_${MODE}_monitor_${STAMP}.log"

kill_all() {
    pkill -9 -f gz                    2>/dev/null || true
    pkill -9 -f rviz                  2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f parameter_bridge      2>/dev/null || true
    pkill -9 -f move_group            2>/dev/null || true
    pkill -9 -f spawner               2>/dev/null || true
    pkill -9 -f octomap               2>/dev/null || true
    pkill -9 -f motion_planner        2>/dev/null || true
    pkill -9 -f environment_bridge    2>/dev/null || true
    pkill -9 -f decision_bridge       2>/dev/null || true
    pkill -9 -f bootstrap             2>/dev/null || true
    pkill -9 -f octomap_to_json       2>/dev/null || true
    pkill -9 -f image_relay           2>/dev/null || true
    pkill -9 -f self_filter           2>/dev/null || true
    pkill -9 -f hppt_boundary_ros        2>/dev/null || true
    pkill -9 -f random_frontier_selector 2>/dev/null || true
    pkill -9 -f multi_coverage_monitor   2>/dev/null || true
    # ★ MEM-LEAK FIX: 之前缺 frontier_viz/initial_scan/periodic_octomap_saver,
    #   每次跑完这些进程活着不被杀,PID 1 (sleep infinity) 不 reap 子进程,
    #   多轮测试累积到 9 个 frontier_viz(每个 ~2GB VSZ)吃满容器内存。
    pkill -9 -f frontier_viz             2>/dev/null || true
    pkill -9 -f initial_scan             2>/dev/null || true
    pkill -9 -f bootstrap_scan           2>/dev/null || true
    pkill -9 -f periodic_octomap_saver   2>/dev/null || true
}

cleanup() {
    echo ""
    echo "==> Cleaning up..."
    if [ -n "${SELECTOR_PID:-}" ]; then kill "${SELECTOR_PID}" 2>/dev/null || true; fi
    if [ -n "${JAKA_PID:-}"     ]; then kill "${JAKA_PID}"     2>/dev/null || true; fi
    kill_all
    echo "==> Logs:"
    echo "    launch:    ${LAUNCH_LOG}"
    echo "    selector:  ${SELECTOR_LOG}"
    echo "    monitor:   ${MONITOR_LOG}"
}
trap cleanup INT TERM EXIT

echo "==> Killing residual ROS processes..."
kill_all
sleep 3

echo "==> Sourcing workspace..."
set +u
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
set -u

echo "==> [1/4] Launching jaka stack with multi world (log -> ${LAUNCH_LOG})"
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
    auto_bootstrap:=false gazebo_gui:="${GAZEBO_GUI}" \
    world_file:=exploration_scene_multi.sdf \
    > "${LAUNCH_LOG}" 2>&1 &
JAKA_PID=$!
echo "    jaka launch pid=${JAKA_PID}"

echo "==> [2/4] Waiting for move_group (timeout ${MOVE_GROUP_WAIT_SEC}s)..."
MOVE_GROUP_READY=false
for ((i = 0; i < MOVE_GROUP_WAIT_SEC; i++)); do
    if ! kill -0 "${JAKA_PID}" 2>/dev/null; then
        echo "ERROR: jaka launch died. Last 40 lines of ${LAUNCH_LOG}:"
        tail -n 40 "${LAUNCH_LOG}"
        exit 1
    fi
    if ros2 action list 2>/dev/null | grep -qx "/move_action"; then
        MOVE_GROUP_READY=true
        break
    fi
    sleep 1
done
if [ "${MOVE_GROUP_READY}" != "true" ]; then
    echo "ERROR: /move_action not ready after ${MOVE_GROUP_WAIT_SEC}s."
    tail -n 40 "${LAUNCH_LOG}"
    exit 1
fi
echo "    move_group ready."

if [ "${WAIT_INITIAL_SCAN}" = "true" ]; then
    echo "==> [3/4] Waiting for initial_scan (timeout ${INITIAL_SCAN_WAIT_SEC}s)..."
    INITIAL_SCAN_DONE=false
    for ((i = 0; i < INITIAL_SCAN_WAIT_SEC; i++)); do
        if ! kill -0 "${JAKA_PID}" 2>/dev/null; then
            echo "ERROR: jaka launch died during initial_scan wait."
            tail -n 40 "${LAUNCH_LOG}"
            exit 1
        fi
        if grep -q "Initial scan complete:" "${LAUNCH_LOG}" 2>/dev/null; then
            INITIAL_SCAN_DONE=true
            break
        fi
        sleep 2
    done
    if [ "${INITIAL_SCAN_DONE}" = "true" ]; then
        echo "    initial_scan finished: $(grep 'Initial scan complete:' "${LAUNCH_LOG}" | tail -1)"
    else
        echo "    WARN: initial_scan did not complete within ${INITIAL_SCAN_WAIT_SEC}s; continuing anyway."
    fi
else
    echo "==> [3/4] Skipping initial_scan wait."
fi

echo "==> [4/4] Starting ${MODE} selector in background (log -> ${SELECTOR_LOG})"
if [ "${MODE}" = "hppt" ]; then
    # HPPT_EXTRA_ARGS: extra ros2 args (e.g. workspace sampling params)
    # Example: HPPT_EXTRA_ARGS="--ros-args -p enable_workspace_sampling:=true"
    # shellcheck disable=SC2086
    ros2 run hppt_boundary_ros hppt_boundary_ros ${HPPT_EXTRA_ARGS:-} \
        > "${SELECTOR_LOG}" 2>&1 &
else
    ros2 run jaka_exploration_core random_frontier_selector --ros-args \
        -p random_seed:="${RANDOM_SEED}" \
        > "${SELECTOR_LOG}" 2>&1 &
fi
SELECTOR_PID=$!
echo "    ${MODE} selector pid=${SELECTOR_PID}"
# Give the selector a moment to publish its first goal before we start
# the timer, so 't=0' in coverage samples aligns with real exploration.
sleep 3

echo ""
echo "==> MODE=${MODE}: Running shelf coverage monitor for ${DURATION_SEC}s"
echo "    (samples every ${SAMPLE_PERIOD_SEC}s; also written to ${MONITOR_LOG})"
echo "----------------------------------------------------------------"
# Force decimal notation so ROS parses these as DOUBLE (the node
# declares them as double; integer literals otherwise get rejected
# with InvalidParameterTypeException).
DURATION_FLOAT=$(awk "BEGIN{printf \"%.6f\", ${DURATION_SEC}}")
SAMPLE_FLOAT=$(awk "BEGIN{printf \"%.6f\", ${SAMPLE_PERIOD_SEC}}")

# Foreground; the monitor self-terminates after duration_sec.
ros2 run jaka_exploration_core multi_coverage_monitor --ros-args \
    -p duration_sec:="${DURATION_FLOAT}" \
    -p sample_period_sec:="${SAMPLE_FLOAT}" \
    2>&1 | tee "${MONITOR_LOG}"
MONITOR_EXIT=${PIPESTATUS[0]}

echo "----------------------------------------------------------------"
echo "==> Monitor exited with code ${MONITOR_EXIT}"
echo "==> FINAL summary from monitor log:"
grep -E "\[FINAL " "${MONITOR_LOG}" || echo "    (no FINAL line found)"

exit "${MONITOR_EXIT}"
