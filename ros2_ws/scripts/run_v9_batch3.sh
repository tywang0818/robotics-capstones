#!/usr/bin/env bash
# 3-run batch: greedy + sizepen hard-cap 5000 + visited_radius 0.30
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
mkdir -p /root/ros2_ws/test_logs

BATCH_STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_LOG="/root/ros2_ws/test_logs/v9_batch3_${BATCH_STAMP}.log"
echo "===== v9 (sizepen=5000, visited_radius=0.30) 3-run batch $(date) =====" | tee "$BATCH_LOG"

for run in 1 2 3; do
    echo "" | tee -a "$BATCH_LOG"
    echo "=====================================================" | tee -a "$BATCH_LOG"
    echo "  RUN ${run}/3 starting at $(date)" | tee -a "$BATCH_LOG"
    echo "=====================================================" | tee -a "$BATCH_LOG"
    MODE=hppt DURATION_SEC=180 SAMPLE_PERIOD_SEC=15 \
    WAIT_INITIAL_SCAN=true INITIAL_SCAN_WAIT_SEC=200 \
    GAZEBO_GUI=false LOG_DIR=/root/ros2_ws/test_logs \
        bash /root/ros2_ws/scripts/test_shelf_coverage.sh 2>&1 | tee -a "$BATCH_LOG"
    echo "==> run ${run} done at $(date)" | tee -a "$BATCH_LOG"
    sleep 5
done

echo "" | tee -a "$BATCH_LOG"
echo "===== batch finished $(date) =====" | tee -a "$BATCH_LOG"
echo "batch log: $BATCH_LOG"
