#!/usr/bin/env bash
#
# run_world_test.sh  —  4 分钟 world 测试（放在 hppt_boundary_ros/tests/ 下）
#
# 特性：
#   1. 通过 WORLD_FILE 环境变量或第一个位置参数切换场景
#   2. 固定跑 240 秒
#   3. 结束时打印 success=true/false 的数量、失败原因分类
#   4. 完整报告写到 /tmp/world_test_<world>_<timestamp>.log
#
# 用法（容器内）：
#   bash /root/ros2_ws/src/hppt_boundary_ros/tests/run_world_test.sh
#   bash /root/ros2_ws/src/hppt_boundary_ros/tests/run_world_test.sh exploration_scene_shelf.sdf
#   WORLD_FILE=exploration_scene_multi.sdf \
#     bash /root/ros2_ws/src/hppt_boundary_ros/tests/run_world_test.sh
#
# 可选环境变量：
#   WORLD_FILE     场景 sdf（默认 exploration_scene.sdf）
#   DURATION_SEC   测试时长（默认 240）
#   GAZEBO_GUI     是否开 Gazebo GUI（默认 false）

set -uo pipefail

# 位置参数优先
WORLD_FILE="${1:-${WORLD_FILE:-exploration_scene.sdf}}"
DURATION_SEC="${DURATION_SEC:-240}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"

# 定位路径：脚本位于 ros2_ws/src/hppt_boundary_ros/tests/
#   SCRIPT_DIR  = .../ros2_ws/src/hppt_boundary_ros/tests
#   WS_DIR      = .../ros2_ws                  (3 层上：tests → hppt_boundary_ros → src → ros2_ws)
#   SCRIPTS_DIR = .../ros2_ws/scripts          (拿 container_kill_all.sh 用)
SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
# 路径层级：tests/ → hppt_boundary_ros/ → src/ → ros2_ws/  (3 层)
WS_DIR="$(readlink -f "${SCRIPT_DIR}/../../..")"
SCRIPTS_DIR="${WS_DIR}/scripts"

# 兜底：如果算错了，报错退出
if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
    echo "[ERROR] 路径解析失败：${WS_DIR}/install/setup.bash 不存在"
    echo "  SCRIPT_DIR=${SCRIPT_DIR}"
    echo "  WS_DIR=${WS_DIR}"
    echo "  提示：跑之前先在 /root/ros2_ws 里 colcon build --packages-select hppt_boundary_ros"
    exit 1
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
WORLD_TAG="$(basename "${WORLD_FILE}" .sdf)"
LAUNCH_LOG="/tmp/launch_${WORLD_TAG}_${TIMESTAMP}.log"
TSP_LOG="/tmp/tsp_${WORLD_TAG}_${TIMESTAMP}.log"
REPORT="/tmp/world_test_${WORLD_TAG}_${TIMESTAMP}.log"

# ---- 输出既到屏幕又到 REPORT 文件 ----
exec > >(tee -a "${REPORT}") 2>&1

# ---- Banner ----
cat <<EOF
════════════════════════════════════════════════════════════════════
  run_world_test.sh
   world       : ${WORLD_FILE}
   duration    : ${DURATION_SEC}s
   gazebo_gui  : ${GAZEBO_GUI}
   launch log  : ${LAUNCH_LOG}
   tsp log     : ${TSP_LOG}
   final report: ${REPORT}
════════════════════════════════════════════════════════════════════
EOF

# ---- Sourcing ----
set +u
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

# ---- 清残留 ----
echo ""
echo "==> Killing residual ROS processes (self-safe)..."
# ★ 不直接调 container_kill_all.sh：它有 `pkill -9 -f hppt_boundary_ros`，
# 而本脚本的路径里就含有 hppt_boundary_ros，会子串匹配自杀。
# 用**带 self-PID 排除**的 pgrep + 数组循环代替（避免 while 循环起子 shell 引来同名匹配）。
self_safe_kill() {
    local self_pid=$$
    local pat pid pids
    for pat in gz rviz robot_state_publisher parameter_bridge move_group \
               spawner octomap motion_planner environment_bridge decision_bridge \
               bootstrap octomap_to_json image_relay self_filter \
               hppt_boundary_ros frontier_viz initial_scan; do
        # 一次性抓完 pid 列表（在主 shell 里），避免管道创建的子 shell 也被 pgrep 命中
        pids=$(pgrep -f "${pat}" 2>/dev/null || true)
        for pid in ${pids}; do
            # 排除自己 + 排除 pgrep 本身的 PPID (=self)
            if [ "${pid}" != "${self_pid}" ]; then
                kill -9 "${pid}" 2>/dev/null || true
            fi
        done
    done
}
self_safe_kill
sleep 3

# ---- 后台启动 jaka ----
echo "==> Launching jaka stack in background..."
rm -f "${LAUNCH_LOG}" "${TSP_LOG}"
ros2 launch jaka_exploration_core exploration_stage2.launch.py \
    auto_bootstrap:=false \
    gazebo_gui:="${GAZEBO_GUI}" \
    world_file:="${WORLD_FILE}" \
    > "${LAUNCH_LOG}" 2>&1 &
JAKA_PID=$!

# ---- 等 move_group ready ----
echo "==> Waiting up to 60s for move_group..."
MOVE_GROUP_READY=false
for ((i = 0; i < 60; i++)); do
    if ! kill -0 "${JAKA_PID}" 2>/dev/null; then
        echo "[ERROR] jaka launch died. Last 30 lines of launch log:"
        tail -n 30 "${LAUNCH_LOG}"
        exit 2
    fi
    if ros2 action list 2>/dev/null | grep -qx "/move_action"; then
        MOVE_GROUP_READY=true
        break
    fi
    sleep 1
done
if [ "${MOVE_GROUP_READY}" != "true" ]; then
    echo "[ERROR] move_group not ready within 60s. Aborting."
    kill "${JAKA_PID}" 2>/dev/null || true
    exit 3
fi
echo "==> move_group ready."

# ---- 启动 TSP ----
echo "==> Starting TSP node..."
ros2 run hppt_boundary_ros hppt_boundary_ros > "${TSP_LOG}" 2>&1 &
TSP_PID=$!

# ---- 清理 hook（强化版：SIGTERM → SIGKILL → 广谱 pkill → killall）----
kill_hard() {
    local pid="$1"
    # 先给 SIGTERM，1 秒后仍活就 SIGKILL；同时 kill 整个进程组
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

# Gazebo Harmonic 的进程比较顽固：ruby 起 gz sim，gz sim 又 fork
# 出 gz-rendering-server / gz-sensors-server 等 daughters。
# 用 killall 按可执行文件名一网打尽（比 pkill -f 匹配子串更精确）。
nuke_gazebo() {
    local self_pid=$$
    # 排除自己：pkill / killall 时都跳过当前 shell PID
    # 按可执行名 kill（gz 系列）
    killall -9 gz                       2>/dev/null || true
    killall -9 gz-rendering-server      2>/dev/null || true
    killall -9 gz-sensors-server        2>/dev/null || true
    killall -9 gzserver                 2>/dev/null || true
    killall -9 gzclient                 2>/dev/null || true
    # ★ 不 killall ruby——太宽泛，容易误伤系统里其他 ruby 进程
    # 只 kill 明确跟 gz sim 相关的 ruby wrapper
    pkill -9 -f 'ruby.*gz sim'          2>/dev/null || true
    pkill -9 -f 'ruby.*gz_tools_vendor' 2>/dev/null || true
    # ★ 不 pkill -f 'sh -c.*gz sim' —— 会匹配到当前脚本自己的 shell
    # 通过 pgrep 手动排除 self_pid
    pgrep -f 'sh -c.*gz sim' 2>/dev/null | while read pid; do
        [ "${pid}" != "${self_pid}" ] && kill -9 "${pid}" 2>/dev/null || true
    done
    # 严格版：以 "gz " 开头的进程
    pgrep -f '^gz ' 2>/dev/null | while read pid; do
        [ "${pid}" != "${self_pid}" ] && kill -9 "${pid}" 2>/dev/null || true
    done
    # ros_gz_bridge 的 parameter_bridge
    pkill -9 -f 'ros_gz_bridge'         2>/dev/null || true
    pkill -9 -f 'parameter_bridge.*gz'  2>/dev/null || true
}

cleanup() {
    kill_hard "${TSP_PID}"  2>/dev/null || true
    kill_hard "${JAKA_PID}" 2>/dev/null || true
    # 广谱 pkill：明确列出 rviz / gazebo 变体
    pkill -9 -f rviz                 2>/dev/null || true
    pkill -9 -f 'gz sim'             2>/dev/null || true
    nuke_gazebo
    # ★ 不调 container_kill_all.sh：它的 pkill -f hppt_boundary_ros 会子串匹配自杀。
    # 用 self_safe_kill 代替（跳过 self_pid）。
    self_safe_kill
    # 二次 Gazebo 补刀
    nuke_gazebo
}
trap cleanup EXIT INT TERM

# ---- 运行到时长 ----
echo "==> Running for ${DURATION_SEC}s..."
elapsed=0
while [ ${elapsed} -lt ${DURATION_SEC} ]; do
    sleep 10
    elapsed=$((elapsed + 10))
    if [ $((elapsed % 60)) -eq 0 ]; then
        echo "    [${elapsed}s / ${DURATION_SEC}s elapsed]"
    fi
done

echo "==> Duration reached. Stopping..."
kill_hard "${TSP_PID}"
kill_hard "${JAKA_PID}"
sleep 2
self_safe_kill        # ★ 用 self-safe，跳过自己 PID
sleep 1

# 二次确认——若仍有 rviz/gz/move_group 存活，再狠一刀
# 注意：survivors 计数要排除自己 PID，否则本脚本自身进程会被计入
survivors=$(pgrep -af "rviz|gz|move_group|self_filter|octomap|hppt_boundary|parameter_bridge|ruby.*gz" 2>/dev/null \
              | awk -v self=$$ '$1 != self' | wc -l || true)
if [ "${survivors:-0}" -gt 0 ]; then
    echo "==> ${survivors} process(es) survived first pass; escalating..."
    self_safe_kill        # ★ 再用 self-safe
    nuke_gazebo
    sleep 1
fi

# 最后一次确认，把幸存者名单打出来（方便你手动补刀）；同样排除自己
final_survivors=$(pgrep -af "rviz|gz|move_group|self_filter|octomap|hppt_boundary|parameter_bridge" 2>/dev/null \
                    | awk -v self=$$ '$1 != self' || true)
if [ -n "${final_survivors}" ]; then
    echo "==> WARN: 以下进程仍未死："
    echo "${final_survivors}" | sed 's/^/    /'
    echo "==> 手动补刀： pkill -9 -f <关键字>"
fi

# ================================================================
# 报告生成
# ================================================================
echo ""
echo "════════════════════════════════════════════════════════════════════"
echo "  报告 — world=${WORLD_FILE}"
echo "════════════════════════════════════════════════════════════════════"

# ---- helper: safe grep count (grep -c 无匹配返回 exit 1 + "0"，会造成双 0) ----
gcnt() {
    local pat="$1" file="$2"
    local n
    n=$(grep -c "$pat" "$file" 2>/dev/null || true)
    echo "${n:-0}"
}

pub=$(gcnt "Published task" "${TSP_LOG}")
sr=$( gcnt "success=true"   "${TSP_LOG}")
fl=$( gcnt "success=false"  "${TSP_LOG}")
sur=$(gcnt "SURROGATE"      "${TSP_LOG}")
# ★ 排除 cost_probe_* 打入的行（它们会淹没真实任务的统计）
rej=$(grep -v "cost_probe" "${LAUNCH_LOG}" 2>/dev/null | grep -c "Rejected task" 2>/dev/null || true); rej=${rej:-0}
scol=$(gcnt "START_STATE_IN_COLLISION" "${LAUNCH_LOG}")
env_ap=$(gcnt "Applied OctoMap environment version" "${LAUNCH_LOG}")

# ★★★ cost_probe 单独统计（若节点没开 use_planning_cost 则全 0）
probe_pub=$(grep -c "cost_probe_" "${LAUNCH_LOG}" 2>/dev/null || true); probe_pub=${probe_pub:-0}
probe_completed=$(grep "Task 'cost_probe" "${LAUNCH_LOG}" 2>/dev/null | grep -c "completed" 2>/dev/null || true); probe_completed=${probe_completed:-0}
probe_ok=$(grep "Task 'cost_probe" "${LAUNCH_LOG}" 2>/dev/null | grep -c "success=True" 2>/dev/null || true); probe_ok=${probe_ok:-0}
probe_fail=$(grep "Task 'cost_probe" "${LAUNCH_LOG}" 2>/dev/null | grep -c "success=False" 2>/dev/null || true); probe_fail=${probe_fail:-0}
probe_rej=$(grep "cost_probe" "${LAUNCH_LOG}" 2>/dev/null | grep -c "Rejected" 2>/dev/null || true); probe_rej=${probe_rej:-0}

cat <<EOF

  ── 概况 ──
  已发布任务          published_tasks       = ${pub}
  成功 (TSP)          success=true          = ${sr}
  失败 (TSP)          success=false         = ${fl}
  SURROGATE           surrogate             = ${sur}
  Rejected (bridge)   rejected              = ${rej}
  START_STATE_IN_COLLISION (MoveIt)         = ${scol}
  env apply 到 PlanningScene                = ${env_ap}
EOF

if [ "${pub}" -gt 0 ]; then
    rate=$(awk -v s="${sr}" -v p="${pub}" 'BEGIN{ printf "%.1f", (s/p)*100 }')
    echo "  成功率                                    = ${rate} %"
fi

# ---- ★★★ Cost probe 段（plan-only 代价查询）----
if [ "${probe_pub}" -gt 0 ]; then
    cat <<EOF

  ── Cost probe 统计 (plan-only cost query) ──
    发出 probe 任务数                       = ${probe_completed}
    plan-only 成功 (success=True)           = ${probe_ok}
    plan-only 失败 (success=False)          = ${probe_fail}
    Rejected                                = ${probe_rej}
EOF
    if [ "${probe_completed}" -gt 0 ]; then
        probe_rate=$(awk -v s="${probe_ok}" -v p="${probe_completed}" \
            'BEGIN{ printf "%.1f", (s/p)*100 }')
        echo "    probe 成功率                            = ${probe_rate} %"
    fi
    # 从 TSP log 的 "cost probes: X/K succeeded" 汇总
    echo ""
    echo "    TSP 每帧的 probe 结果分布 (最后 5 帧)："
    grep "cost probes:" "${TSP_LOG}" 2>/dev/null | tail -5 | sed 's/^/      /' \
        || echo "      (无 TSP 端汇总数据)"
    # cost probe 的 planning_time+execution_time 分布
    echo ""
    echo "    Cost probe 的 (planning_time, execution_time) 前 10 条："
    grep "Task 'cost_probe" "${LAUNCH_LOG}" 2>/dev/null \
        | grep -oE "planning_time=[0-9.]+s, estimated_execution_time=[0-9.]+s" \
        | head -10 | sed 's/^/      /' || echo "      (无 timing 数据)"
fi

# ---- MoveIt error code 分布（★ 排除 cost_probe）----
echo ""
echo "  ── MoveIt error code 分布 (排除 cost_probe) ──"
grep -v "cost_probe" "${LAUNCH_LOG}" 2>/dev/null \
    | grep -oE "success=(True|False), MoveIt error=-?[0-9]+" \
    | sort | uniq -c | sort -rn | sed 's/^/    /' || echo "    (无数据)"

# ---- 失败原因分类：先解析 error_type 字段 ----
echo ""
echo "  ── 失败原因 (error_type from motion_result_json) ──"
grep -oE 'error_type["]*: *"[^"]*"' "${LAUNCH_LOG}" "${TSP_LOG}" 2>/dev/null \
    | sed 's/.*error_type[^:]*: *//' | sed 's/^"//' | sed 's/"$//' \
    | grep -v '^$' | sort | uniq -c | sort -rn | sed 's/^/    /' \
    || echo "    (未提取到 error_type)"

# ---- 失败任务的具体 launch 行（★ 排除 cost_probe）----
echo ""
echo "  ── 失败任务 (success=False 完整行，最多 10 条，排除 cost_probe) ──"
grep "success=False" "${LAUNCH_LOG}" 2>/dev/null | grep -v "cost_probe" \
    | head -10 | sed 's/^/    /' || echo "    (无 success=False 行)"

# ---- Rejected 摘要（★ 排除 cost_probe，前 5 条即可）----
if [ "${rej}" -gt 0 ]; then
    echo ""
    echo "  ── Rejected 任务 (前 5 条，排除 cost_probe) ──"
    grep "Rejected task" "${LAUNCH_LOG}" 2>/dev/null | grep -v "cost_probe" \
        | head -5 | sed 's/^/    /'
fi

# ---- Filter 计数（最后 3 帧）----
echo ""
echo "  ── Boundary filter (最后 3 帧) ──"
grep "Boundary filter:" "${TSP_LOG}" 2>/dev/null | tail -3 | sed 's/^/    /' \
    || echo "    (无 Filter 输出)"

# ---- self_filter 状态 ----
echo ""
echo "  ── self_filter 状态 ──"
grep -E "self_filter|self-points dropped" "${LAUNCH_LOG}" 2>/dev/null \
    | tail -5 | sed 's/^/    /'

# ---- 总结 ----
echo ""
echo "  ── 判读提示 ──"
if [ "${pub}" -eq 0 ]; then
    echo "    ⚠️  一个任务都没发——检查 TSP 有没有起、env_version 有没有 tracking"
elif [ "${sr}" -eq 0 ]; then
    echo "    ⚠️  没有一个成功——检查 MoveIt error 分布，可能是全 -10 (self_filter leak) 或全 Rejected"
elif [ "${rej}" -gt "${pub}" ]; then
    echo "    ⚠️  Rejected 比 published 还多——可能 environment_version race"
elif [ "${scol}" -gt "$((pub / 2))" ]; then
    echo "    ⚠️  超半数 START_STATE_IN_COLLISION——self_filter 有 leak，或视点参数让 arm 撞基座"
fi

echo ""
echo "  详细日志:"
echo "    launch:  ${LAUNCH_LOG}"
echo "    tsp:     ${TSP_LOG}"
echo "    report:  ${REPORT}"
echo "════════════════════════════════════════════════════════════════════"
