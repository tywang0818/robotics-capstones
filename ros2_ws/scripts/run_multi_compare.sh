#!/usr/bin/env bash
# Multi world comparison: HPP-PT (v11) vs random baseline
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
mkdir -p /root/ros2_ws/test_logs

BATCH_STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_LOG="/root/ros2_ws/test_logs/multi_compare_${BATCH_STAMP}.log"
echo "===== multi world compare (HPP-PT v11 vs random) at $(date) =====" | tee "$BATCH_LOG"

for mode in hppt random; do
    echo "" | tee -a "$BATCH_LOG"
    echo "=====================================================" | tee -a "$BATCH_LOG"
    echo "  MODE=${mode} starting at $(date)" | tee -a "$BATCH_LOG"
    echo "=====================================================" | tee -a "$BATCH_LOG"
    MODE=${mode} DURATION_SEC=180 SAMPLE_PERIOD_SEC=15 \
    WAIT_INITIAL_SCAN=true INITIAL_SCAN_WAIT_SEC=200 \
    GAZEBO_GUI=false LOG_DIR=/root/ros2_ws/test_logs \
    RANDOM_SEED=42 \
        bash /root/ros2_ws/scripts/test_multi_coverage.sh 2>&1 | tee -a "$BATCH_LOG"
    echo "==> ${mode} done at $(date)" | tee -a "$BATCH_LOG"
    sleep 5
done

echo "" | tee -a "$BATCH_LOG"
echo "===== compare finished $(date) =====" | tee -a "$BATCH_LOG"
echo "batch log: $BATCH_LOG"
