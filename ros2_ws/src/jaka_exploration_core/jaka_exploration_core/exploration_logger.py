#!/usr/bin/env python3
"""Structured exploration event logger for post-hoc AI analysis.

Subscribes to /rosout (all node logs), the team JSON topics, and joint
state, parses the narrative log lines into structured events, and writes
one JSON object per event to a JSONL file. An LLM or grep-based analyst
can reconstruct the entire exploration timeline from this file without
looking at any visualization.

Design goals
------------
* One line per event, always machine-parseable JSON.
* Every event carries: wall-clock ts, ROS stamp, seq, cycle_num, type.
* Per-cycle events are grouped by `cycle_num`, incremented at every
  "HPP-PT cycle START". Downstream can `WHERE cycle_num=N` to reassemble
  a cycle.
* Original raw log line is embedded as `raw` field so unknown-format
  edges are still recoverable.
* Every 30 seconds (configurable), a periodic snapshot of arm joint
  state + latest env_version + sticky-blacklist counts is emitted.
* Task-lifecycle stitching: publish/queued/dispatched/completed events
  for the same task_id are all annotated with `task_id`.

Output schema (all events share ts, ros_stamp, seq, cycle_num, type)
--------------------------------------------------------------------
type=cycle_start         env_version, robot_xyz? (later from STAGE 0)
type=stage0_tf           robot_xyz
type=stage1_frontier     total, per_axis
type=stage2_filter       before, after, reasons_dict
type=stage2_empty        (with reasons_dict)
type=stage3_cluster      raw, filtered, picked_K
type=stage3c_empty       (with drop reasons)
type=prob_candidate      cand_i, frontier_xyz, obs/unk/occl/ig/peek/
                         attract/sem/los/igate_killed/raw
type=stage5_greedy_pick  picked_i, raw, cost, score, goal_xyz
type=stage5_greedy_ranks ranks_list
type=stage5_empty        skip_reasons_dict
type=confirm_probe       goal, cost, result ("OK"|"FAIL"), planning_time
type=publish             task_id, goal, surrogate, message_snippet
type=task_result         task_id, success, moveit_error, planning_time,
                         exec_time, age_sec, env_drift_sec
type=blacklist_add       goal, retryable, planning_time, radius, kind
type=sticky_release      count_cleared, reason
type=visited_mark        goal
type=env_version         version, payload_bytes
type=joint_snapshot      positions (list of 6)
type=scan_phase          phase_name, j4_deg, direction, j1_start/j1_end
type=initial_scan_done   succeeded, failed
type=cycle_summary       (emitted right before next cycle_start; sums
                         per-cycle counters into one line)
"""
import json
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# ------------------------- regex patterns -------------------------------
FLOAT = r"[\-0-9.eE+]+"

RE_CYCLE_START = re.compile(
    r"HPP-PT cycle START \(env_version=(\d+)\)")
RE_STAGE0_TF = re.compile(
    r"STAGE 0 \(TF\): robot at \[({F}), ({F}), ({F})\]".format(F=FLOAT))
RE_STAGE1_FRONTIER = re.compile(
    r"STAGE 1 \(frontier extract\): (\d+) free-unknown boundary voxels found")
RE_STAGE2_FILTER = re.compile(
    r"STAGE 2 \(per-voxel filter\): (\d+) -> (\d+) \(dropped (\d+)\)\. reasons: "
    r"too_far=(\d+), forbidden\(r<reach_min or z-oob\)=(\d+), "
    r"unreach_but_observed_flag_off=(\d+), blacklisted=(\d+), "
    r"perm_blocked=(\d+), visited=(\d+), inward_look_at=(\d+)")
RE_STAGE2_EMPTY = re.compile(r"STAGE 2 EMPTY: all (\d+) boundary voxels dropped")
RE_STAGE3_CLUSTER_A = re.compile(
    r"STAGE 3a \(clusterFrontier\): (\d+) voxels -> (\d+) raw clusters")
RE_STAGE3_CLUSTER_B = re.compile(
    r"STAGE 3b \(min_voxel filter\): (\d+) raw -> (\d+) after cluster_min_voxels")
RE_STAGE3_DONE = re.compile(
    r"STAGE 3 done: K=(\d+) candidates ready")
RE_STAGE3C_EMPTY = re.compile(
    r"STAGE 3c EMPTY: all (\d+) clusters dropped by centroid checks "
    r"\(workspace/forbid=(\d+), blacklist=(\d+), perm=(\d+), visited=(\d+), "
    r"inward=(\d+), LOS_blocked=(\d+)\)")
RE_PROB = re.compile(
    r"PROB \[p=\(({F}),({F}),({F})\)\]: obs=({F}) unk=({F}) occl=({F}) "
    r"ig=({F}) peek=({F}) attract=({F}) sem=({F}) los=(\d) "
    r"igate=(\d) raw=({F})".format(F=FLOAT))
RE_GREEDY_RANKS = re.compile(
    r"GREEDY-ARGMIN ranks \(score=raw/eff_cost\): (.+)")
RE_GREEDY_PICK = re.compile(
    r"GREEDY picked i=(\d+) cand=(\d+) score=({F}) raw=({F}) prob=({F}) "
    r"cost=({F}) goal=\[({F}),({F}),({F})\]".format(F=FLOAT))
RE_STAGE5_EMPTY = re.compile(
    r"STAGE 5 EMPTY: all (\d+) candidates rejected\. "
    r"\(isGoalBad=(\d+): out-of-workspace/blacklisted/inward; "
    r"probe_cost>=({F})=(\d+): MoveIt already rejected in probe; "
    r"duplicate_of_last_goal=(\d+)\)".format(F=FLOAT))
RE_CONFIRM_SEND = re.compile(
    r"STAGE 5\.5 \(confirm probe\): sending fresh probe '(\S+)' "
    r"for goal=\[({F}),({F}),({F})\] against env_v=(\d+) \(drift ({F})s".format(F=FLOAT))
RE_CONFIRM_RESULT = re.compile(
    r"STAGE 5\.5 \(confirm probe\): '(\S+)' returned cost=({F}) -> (OK|FAIL)".format(F=FLOAT))
RE_STAGE6_PUBLISH = re.compile(
    r"STAGE 6 \(publish\): task '(\S+)' -> goal \[({F}), ({F}), ({F})\] "
    r"(?:\((SURROGATE[^)]*)\) )?".format(F=FLOAT))
RE_TASK_RESULT_PENDING = re.compile(
    r"Result for pending task '(\S+)': success=(true|false), "
    r"retryable=(true|false), planning_time=({F})s, "
    r"estimated_execution_time=({F})s, task_age=({F})s, "
    r"probe_env_v=(\d+), current_env_v=(\d+), env_drift=({F})s".format(F=FLOAT))
RE_BLACKLIST_ADD = re.compile(
    r"Blacklisting failed goal \[({F}), ({F}), ({F})\] for ({F})s "
    r"\(retryable=(true|false), planning_time=({F})s(?: → FAST-REJECT)?, "
    r"radius=({F})m\)".format(F=FLOAT))
RE_STICKY_RELEASE = re.compile(
    r"STICKY-RELEASE: clearing (\d+) sticky-blacklist entries because (.+?)\.")
RE_VISITED_MARK = re.compile(
    r"Marking completed goal \[({F}), ({F}), ({F})\] as visited for ({F})s".format(F=FLOAT))
RE_COST_PROBE_IK_REJECT = re.compile(
    r"cost_probe '(\S+)' IK-rejected \(pt=({F})s, error_type='([^']*)'\) "
    r"-> blacklist \[({F}), ({F}), ({F})\]".format(F=FLOAT))

# initial_scan lifecycle
RE_SCAN_PHASE = re.compile(
    r"==> PHASE (\d+)/(\d+): (\S+) J1 ({F})° -> ({F})° \((CCW|CW)\), "
    r"J4=({F})°".format(F=FLOAT))
RE_SCAN_DONE = re.compile(
    r"Initial scan complete: (\d+)/(\d+) OK\. succeeded=(\[[^\]]*\]) "
    r"failed=(\[[^\]]*\])")

# env bridge & motion planner (payload_bytes signal)
RE_ENV_PUBLISHED = re.compile(
    r"Published environment version (\d+) with OctoMap payload size (\d+) byte")


class ExplorationLogger(Node):
    def __init__(self):
        super().__init__("exploration_logger")
        self.declare_parameter(
            "output_path", "/tmp/exploration_events.jsonl")
        self.declare_parameter("joint_snapshot_period_sec", 30.0)
        # rosout is BEST_EFFORT with a large depth; match it to catch bursts.
        rosout_qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._out_path = self.get_parameter("output_path").value
        os.makedirs(os.path.dirname(self._out_path) or ".", exist_ok=True)
        self._fh = open(self._out_path, "w")
        self._seq = 0
        self._current_cycle = 0
        self._latest_env_version = None
        self._latest_joints = None

        self.create_subscription(
            Log, "/rosout", self._rosout_cb, rosout_qos)
        self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10)

        period = float(
            self.get_parameter("joint_snapshot_period_sec").value)
        self.create_timer(period, self._joint_snapshot)

        self._emit({
            "type": "logger_start",
            "output": self._out_path,
            "joint_snapshot_period_sec": period,
        })
        self.get_logger().info(
            f"exploration_logger writing to {self._out_path}")

    # ---------------------- emit --------------------------------------
    def _emit(self, evt):
        self._seq += 1
        base = {
            "seq": self._seq,
            "ts": time.time(),
            "cycle": self._current_cycle,
        }
        base.update(evt)
        self._fh.write(json.dumps(base, separators=(",", ":")) + "\n")
        self._fh.flush()

    # ---------------------- callbacks ---------------------------------
    def _rosout_cb(self, msg: Log):
        name = msg.name
        text = msg.msg
        ros_stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1e9
        try:
            if name == "hppt_boundary_node":
                self._parse_hppt(text, ros_stamp)
            elif name == "decision_bridge":
                self._parse_decision(text, ros_stamp)
            elif name == "environment_bridge":
                self._parse_env_bridge(text, ros_stamp)
            elif name == "initial_scan":
                self._parse_initial_scan(text, ros_stamp)
        except Exception as exc:  # noqa: BLE001
            # Never crash the logger on a parse mismatch; emit the raw
            # line so nothing gets lost.
            self._emit({
                "type": "parse_error",
                "ros_stamp": ros_stamp,
                "node": name,
                "raw": text,
                "exc": str(exc),
            })

    def _joint_cb(self, msg: JointState):
        self._latest_joints = {
            "name": list(msg.name),
            "position": list(msg.position),
        }

    def _joint_snapshot(self):
        if self._latest_joints is None:
            return
        self._emit({
            "type": "joint_snapshot",
            "joints": self._latest_joints,
            "latest_env_version": self._latest_env_version,
        })

    # ---------------------- hppt parser --------------------------------
    def _parse_hppt(self, text, ros_stamp):
        m = RE_CYCLE_START.search(text)
        if m:
            self._current_cycle += 1
            self._latest_env_version = int(m.group(1))
            self._emit({
                "type": "cycle_start",
                "ros_stamp": ros_stamp,
                "env_version": int(m.group(1)),
            })
            return

        m = RE_STAGE0_TF.search(text)
        if m:
            self._emit({
                "type": "stage0_tf",
                "ros_stamp": ros_stamp,
                "robot_xyz": [float(m.group(1)), float(m.group(2)), float(m.group(3))],
            })
            return

        m = RE_STAGE1_FRONTIER.search(text)
        if m:
            self._emit({
                "type": "stage1_frontier",
                "ros_stamp": ros_stamp,
                "total_voxels": int(m.group(1)),
            })
            return

        m = RE_STAGE2_FILTER.search(text)
        if m:
            self._emit({
                "type": "stage2_filter",
                "ros_stamp": ros_stamp,
                "before": int(m.group(1)),
                "after": int(m.group(2)),
                "dropped": int(m.group(3)),
                "reasons": {
                    "too_far": int(m.group(4)),
                    "forbidden": int(m.group(5)),
                    "unreach_flag_off": int(m.group(6)),
                    "blacklisted": int(m.group(7)),
                    "perm_blocked": int(m.group(8)),
                    "visited": int(m.group(9)),
                    "inward": int(m.group(10)),
                },
            })
            return

        m = RE_STAGE2_EMPTY.search(text)
        if m:
            self._emit({
                "type": "stage2_empty",
                "ros_stamp": ros_stamp,
                "before": int(m.group(1)),
            })
            return

        m = RE_STAGE3_CLUSTER_A.search(text)
        if m:
            self._emit({
                "type": "stage3a",
                "ros_stamp": ros_stamp,
                "voxels": int(m.group(1)),
                "raw_clusters": int(m.group(2)),
            })
            return

        m = RE_STAGE3_CLUSTER_B.search(text)
        if m:
            self._emit({
                "type": "stage3b",
                "ros_stamp": ros_stamp,
                "raw": int(m.group(1)),
                "after_min_voxels": int(m.group(2)),
            })
            return

        m = RE_STAGE3_DONE.search(text)
        if m:
            self._emit({
                "type": "stage3_done",
                "ros_stamp": ros_stamp,
                "K": int(m.group(1)),
            })
            return

        m = RE_STAGE3C_EMPTY.search(text)
        if m:
            self._emit({
                "type": "stage3c_empty",
                "ros_stamp": ros_stamp,
                "before": int(m.group(1)),
                "reasons": {
                    "workspace_forbid": int(m.group(2)),
                    "blacklist": int(m.group(3)),
                    "perm": int(m.group(4)),
                    "visited": int(m.group(5)),
                    "inward": int(m.group(6)),
                    "los_blocked": int(m.group(7)),
                },
            })
            return

        m = RE_PROB.search(text)
        if m:
            self._emit({
                "type": "prob_candidate",
                "ros_stamp": ros_stamp,
                "frontier_xyz": [float(m.group(1)), float(m.group(2)), float(m.group(3))],
                "obs": float(m.group(4)),
                "unk": float(m.group(5)),
                "occl": float(m.group(6)),
                "ig": float(m.group(7)),
                "peek": float(m.group(8)),
                "attract": float(m.group(9)),
                "sem": float(m.group(10)),
                "los": int(m.group(11)) == 1,
                "igate_killed": int(m.group(12)) == 1,
                "raw": float(m.group(13)),
            })
            return

        m = RE_GREEDY_RANKS.search(text)
        if m:
            # Parse the "[i=0 raw=.. cost=.. s=..]" pattern individually
            ranks_str = m.group(1)
            item_re = re.compile(
                r"\[i=(\d+) raw=({F}) cost=({F}) s=({F})\]".format(F=FLOAT))
            ranks = [
                {
                    "i": int(a),
                    "raw": float(b),
                    "cost": float(c),
                    "score": float(d),
                }
                for (a, b, c, d) in item_re.findall(ranks_str)
            ]
            self._emit({
                "type": "stage5_greedy_ranks",
                "ros_stamp": ros_stamp,
                "ranks": ranks,
            })
            return

        m = RE_GREEDY_PICK.search(text)
        if m:
            self._emit({
                "type": "stage5_greedy_pick",
                "ros_stamp": ros_stamp,
                "picked_i": int(m.group(1)),
                "cand": int(m.group(2)),
                "score": float(m.group(3)),
                "raw": float(m.group(4)),
                "prob": float(m.group(5)),
                "cost": float(m.group(6)),
                "goal": [float(m.group(7)), float(m.group(8)), float(m.group(9))],
            })
            return

        m = RE_STAGE5_EMPTY.search(text)
        if m:
            self._emit({
                "type": "stage5_empty",
                "ros_stamp": ros_stamp,
                "K": int(m.group(1)),
                "skip_bad": int(m.group(2)),
                "cost_threshold": float(m.group(3)),
                "skip_probe": int(m.group(4)),
                "skip_duplicate": int(m.group(5)),
            })
            return

        m = RE_CONFIRM_SEND.search(text)
        if m:
            self._emit({
                "type": "confirm_probe_send",
                "ros_stamp": ros_stamp,
                "probe_id": m.group(1),
                "goal": [float(m.group(2)), float(m.group(3)), float(m.group(4))],
                "env_version": int(m.group(5)),
                "drift_sec": float(m.group(6)),
            })
            return

        m = RE_CONFIRM_RESULT.search(text)
        if m:
            self._emit({
                "type": "confirm_probe_result",
                "ros_stamp": ros_stamp,
                "probe_id": m.group(1),
                "cost": float(m.group(2)),
                "result": m.group(3),
            })
            return

        m = RE_STAGE6_PUBLISH.search(text)
        if m:
            self._emit({
                "type": "publish",
                "ros_stamp": ros_stamp,
                "task_id": m.group(1),
                "goal": [float(m.group(2)), float(m.group(3)), float(m.group(4))],
                "surrogate": bool(m.group(5)),
                "surrogate_note": m.group(5) if m.group(5) else None,
            })
            return

        m = RE_TASK_RESULT_PENDING.search(text)
        if m:
            self._emit({
                "type": "task_result",
                "ros_stamp": ros_stamp,
                "task_id": m.group(1),
                "success": m.group(2) == "true",
                "retryable": m.group(3) == "true",
                "planning_time_sec": float(m.group(4)),
                "exec_time_sec": float(m.group(5)),
                "age_sec": float(m.group(6)),
                "probe_env_v": int(m.group(7)),
                "current_env_v": int(m.group(8)),
                "env_drift_sec": float(m.group(9)),
            })
            return

        m = RE_BLACKLIST_ADD.search(text)
        if m:
            self._emit({
                "type": "blacklist_add",
                "ros_stamp": ros_stamp,
                "goal": [float(m.group(1)), float(m.group(2)), float(m.group(3))],
                "duration_sec": float(m.group(4)),
                "retryable": m.group(5) == "true",
                "planning_time_sec": float(m.group(6)),
                "radius": float(m.group(7)),
                "kind": "regular+sticky",
            })
            return

        m = RE_COST_PROBE_IK_REJECT.search(text)
        if m:
            self._emit({
                "type": "blacklist_add",
                "ros_stamp": ros_stamp,
                "probe_id": m.group(1),
                "planning_time_sec": float(m.group(2)),
                "error_type": m.group(3),
                "goal": [float(m.group(4)), float(m.group(5)), float(m.group(6))],
                "kind": "ik_reject",
            })
            return

        m = RE_STICKY_RELEASE.search(text)
        if m:
            self._emit({
                "type": "sticky_release",
                "ros_stamp": ros_stamp,
                "count_cleared": int(m.group(1)),
                "reason": m.group(2),
            })
            return

        m = RE_VISITED_MARK.search(text)
        if m:
            self._emit({
                "type": "visited_mark",
                "ros_stamp": ros_stamp,
                "goal": [float(m.group(1)), float(m.group(2)), float(m.group(3))],
                "duration_sec": float(m.group(4)),
            })
            return

    # ---------------------- decision_bridge parser ---------------------
    def _parse_decision(self, text, ros_stamp):
        # Task 'X' completed: success=True, MoveIt error=1, planning_time=..., estimated_execution_time=...
        m = re.search(
            r"Task '(\S+)' completed: success=(True|False), MoveIt error=(-?\d+), "
            r"planning_time=({F})s, estimated_execution_time=({F})s".format(F=FLOAT),
            text)
        if m:
            self._emit({
                "type": "decision_task_completed",
                "ros_stamp": ros_stamp,
                "task_id": m.group(1),
                "success": m.group(2) == "True",
                "moveit_error": int(m.group(3)),
                "planning_time_sec": float(m.group(4)),
                "exec_time_sec": float(m.group(5)),
            })
            return
        # Queued task 'X' at position N.
        m = re.search(r"Queued task '(\S+)' at position (\d+)", text)
        if m:
            self._emit({
                "type": "decision_queued",
                "ros_stamp": ros_stamp,
                "task_id": m.group(1),
                "position": int(m.group(2)),
            })
            return
        # Task 'X' stamped env_version A; upgrading to current B before dispatch.
        m = re.search(
            r"Task '(\S+)' stamped env_version (\d+); upgrading to current (\d+)",
            text)
        if m:
            self._emit({
                "type": "decision_env_upgrade",
                "ros_stamp": ros_stamp,
                "task_id": m.group(1),
                "old_v": int(m.group(2)),
                "new_v": int(m.group(3)),
            })
            return

    def _parse_env_bridge(self, text, ros_stamp):
        m = RE_ENV_PUBLISHED.search(text)
        if m:
            v = int(m.group(1))
            if v != self._latest_env_version:
                self._latest_env_version = v
            self._emit({
                "type": "env_version",
                "ros_stamp": ros_stamp,
                "env_version": v,
                "payload_bytes": int(m.group(2)),
            })

    def _parse_initial_scan(self, text, ros_stamp):
        m = RE_SCAN_PHASE.search(text)
        if m:
            self._emit({
                "type": "scan_phase",
                "ros_stamp": ros_stamp,
                "phase_idx": int(m.group(1)),
                "phase_total": int(m.group(2)),
                "name": m.group(3),
                "j1_start_deg": float(m.group(4)),
                "j1_end_deg": float(m.group(5)),
                "direction": m.group(6),
                "j4_deg": float(m.group(7)),
            })
            return
        m = RE_SCAN_DONE.search(text)
        if m:
            self._emit({
                "type": "initial_scan_done",
                "ros_stamp": ros_stamp,
                "success_count": int(m.group(1)),
                "total_count": int(m.group(2)),
                "succeeded_raw": m.group(3),
                "failed_raw": m.group(4),
            })
            return


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._fh.flush()
            node._fh.close()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
