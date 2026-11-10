#!/usr/bin/env bash
#
# test_shelf_coverage_compare.sh — HPP-PT vs random-frontier baseline.
#
# Runs test_shelf_coverage.sh twice back-to-back (once with MODE=hppt,
# once with MODE=random), then prints a side-by-side comparison of the
# FINAL coverage samples from each run.
#
# Both runs use identical world (shelf), identical duration, identical
# initial_scan, identical frontier extraction / workspace filter /
# standoff / look-at formula, identical MoveIt params. The ONLY
# difference is the selection policy: HPP-PT graph search vs uniform
# random choice.
#
# Usage (inside container):
#   bash /root/ros2_ws/scripts/test_shelf_coverage_compare.sh
#
# Env vars (forwarded to test_shelf_coverage.sh):
#   DURATION_SEC          per-run duration, default 180
#   SAMPLE_PERIOD_SEC     coverage log period, default 10
#   GAZEBO_GUI            true|false, default false
#   RANDOM_SEED           int, default -1 (system entropy)
#   INITIAL_SCAN_WAIT_SEC default 600
#   RUN_ORDER             "hppt,random" or "random,hppt", default "hppt,random"
#   INTER_RUN_SLEEP_SEC   grace period between runs, default 10
#   LOG_DIR               where to place logs, default /tmp

set -euo pipefail

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
DURATION_SEC="${DURATION_SEC:-180}"
SAMPLE_PERIOD_SEC="${SAMPLE_PERIOD_SEC:-10}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
RANDOM_SEED="${RANDOM_SEED:--1}"
INITIAL_SCAN_WAIT_SEC="${INITIAL_SCAN_WAIT_SEC:-600}"
RUN_ORDER="${RUN_ORDER:-hppt,random}"
INTER_RUN_SLEEP_SEC="${INTER_RUN_SLEEP_SEC:-10}"
LOG_DIR="${LOG_DIR:-/tmp}"

STAMP="$(date +%Y%m%d_%H%M%S)"
SUMMARY_LOG="${LOG_DIR}/shelf_test_compare_${STAMP}.log"

# Where each run's monitor log went (filled in as we go).
declare -A MONITOR_LOG_OF

run_one() {
    local mode="$1"
    echo ""
    echo "###############################################################"
    echo "##  RUN: MODE=${mode}   duration=${DURATION_SEC}s"
    echo "###############################################################"
    # test_shelf_coverage.sh names its monitor log
    # ${LOG_DIR}/shelf_test_${MODE}_monitor_<stamp>.log
    # so record the log dir listing BEFORE the run and diff after, to
    # capture the exact filename (stamp differs from ours).
    local before after new_log
    before=$(ls "${LOG_DIR}" 2>/dev/null | grep -c "^shelf_test_${mode}_monitor_" || true)
    MODE="${mode}" \
    DURATION_SEC="${DURATION_SEC}" \
    SAMPLE_PERIOD_SEC="${SAMPLE_PERIOD_SEC}" \
    GAZEBO_GUI="${GAZEBO_GUI}" \
    RANDOM_SEED="${RANDOM_SEED}" \
    INITIAL_SCAN_WAIT_SEC="${INITIAL_SCAN_WAIT_SEC}" \
    LOG_DIR="${LOG_DIR}" \
        bash "${SCRIPT_DIR}/test_shelf_coverage.sh" || {
            echo "WARN: MODE=${mode} run exited non-zero; continuing to next."
        }
    after=$(ls "${LOG_DIR}" 2>/dev/null | grep -c "^shelf_test_${mode}_monitor_" || true)
    if [ "${after}" -le "${before}" ]; then
        echo "WARN: no new monitor log detected for MODE=${mode}"
        MONITOR_LOG_OF[${mode}]=""
        return
    fi
    new_log=$(ls -t "${LOG_DIR}"/shelf_test_${mode}_monitor_*.log 2>/dev/null | head -1)
    MONITOR_LOG_OF[${mode}]="${new_log}"
    echo "==> ${mode} monitor log: ${new_log}"
}

# ---- Parse RUN_ORDER (comma-separated) ----
IFS=',' read -ra ORDER <<< "${RUN_ORDER}"
for mode in "${ORDER[@]}"; do
    case "${mode}" in
        hppt|random) ;;
        *) echo "ERROR: RUN_ORDER contains invalid mode '${mode}'"; exit 2;;
    esac
done

# ---- Run each mode ----
for mode in "${ORDER[@]}"; do
    run_one "${mode}"
    if [ "${mode}" != "${ORDER[-1]}" ]; then
        echo "==> Cooling down for ${INTER_RUN_SLEEP_SEC}s before next run..."
        sleep "${INTER_RUN_SLEEP_SEC}"
    fi
done

# ---- Extract FINAL lines and produce comparison ----
extract_final() {
    local log="$1"
    [ -z "${log}" ] && { echo "(no log)"; return; }
    [ -f "${log}" ] || { echo "(log missing)"; return; }
    grep -E "\[FINAL " "${log}" | tail -1
}

parse_field() {
    # $1 = FINAL line, $2 = regex capturing the value
    local line="$1"
    local re="$2"
    if [[ "${line}" =~ ${re} ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        echo "-"
    fi
}

FINAL_HPPT=$(extract_final "${MONITOR_LOG_OF[hppt]:-}")
FINAL_RAND=$(extract_final "${MONITOR_LOG_OF[random]:-}")

HPPT_COV=$(parse_field "${FINAL_HPPT}" 'coverage=([0-9.]+)%')
HPPT_OCC=$(parse_field "${FINAL_HPPT}" 'occ=([0-9.]+)%')
HPPT_FREE=$(parse_field "${FINAL_HPPT}" 'free=([0-9.]+)%')
HPPT_UNK=$(parse_field "${FINAL_HPPT}" 'unknown=([0-9.]+)%')
HPPT_KU=$(parse_field "${FINAL_HPPT}"  'known/unknown=([0-9.inf]+)')

RAND_COV=$(parse_field "${FINAL_RAND}" 'coverage=([0-9.]+)%')
RAND_OCC=$(parse_field "${FINAL_RAND}" 'occ=([0-9.]+)%')
RAND_FREE=$(parse_field "${FINAL_RAND}" 'free=([0-9.]+)%')
RAND_UNK=$(parse_field "${FINAL_RAND}" 'unknown=([0-9.]+)%')
RAND_KU=$(parse_field "${FINAL_RAND}"  'known/unknown=([0-9.inf]+)')

{
echo ""
echo "==============================================================="
echo " Shelf coverage comparison  (duration=${DURATION_SEC}s)"
echo "==============================================================="
echo ""
printf "  %-16s %-16s %-16s\n" "metric" "HPP-PT" "random"
printf "  %-16s %-16s %-16s\n" "----------------" "----------------" "----------------"
printf "  %-16s %-16s %-16s\n" "coverage %"       "${HPPT_COV}"      "${RAND_COV}"
printf "  %-16s %-16s %-16s\n" "occupied %"       "${HPPT_OCC}"      "${RAND_OCC}"
printf "  %-16s %-16s %-16s\n" "free %"           "${HPPT_FREE}"     "${RAND_FREE}"
printf "  %-16s %-16s %-16s\n" "unknown %"        "${HPPT_UNK}"      "${RAND_UNK}"
printf "  %-16s %-16s %-16s\n" "known/unknown"    "${HPPT_KU}"       "${RAND_KU}"
echo ""
echo "  FINAL lines:"
echo "    HPP-PT : ${FINAL_HPPT:-(missing)}"
echo "    random : ${FINAL_RAND:-(missing)}"
echo ""
echo "  Monitor logs:"
echo "    HPP-PT : ${MONITOR_LOG_OF[hppt]:-(none)}"
echo "    random : ${MONITOR_LOG_OF[random]:-(none)}"
echo "==============================================================="
} | tee "${SUMMARY_LOG}"

echo ""
echo "==> Summary saved to: ${SUMMARY_LOG}"
