#!/usr/bin/env bash
# 2-run batch for greedy-argmin validation.
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
mkdir -p /root/ros2_ws/test_logs

BATCH_STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_LOG="/root/ros2_ws/test_logs/greedy_batch2_${BATCH_STAMP}.log"
echo "===== 2-run greedy batch started $(date) =====" | tee "$BATCH_LOG"

for run in 1 2; do
    echo "" | tee -a "$BATCH_LOG"
    echo "=====================================================" | tee -a "$BATCH_LOG"
    echo "  RUN ${run}/2 starting at $(date)" | tee -a "$BATCH_LOG"
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
