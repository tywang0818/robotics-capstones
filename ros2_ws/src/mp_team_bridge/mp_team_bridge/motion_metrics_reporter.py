#!/usr/bin/env python3

import json
import math
import os
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def timing_summary(records, field):
    values = [
        float(item[field])
        for item in records
        if item.get("success") and item.get(field) is not None
    ]
    if not values:
        return {"count": 0, "mean_sec": None, "median_sec": None, "p95_sec": None,
                "min_sec": None, "max_sec": None}
    return {
        "count": len(values),
        "mean_sec": mean(values),
        "median_sec": median(values),
        "p95_sec": percentile(values, 0.95),
        "min_sec": min(values),
        "max_sec": max(values),
    }


def summarize(records):
    successes = sum(bool(item.get("success")) for item in records)
    failures = len(records) - successes
    return {
        "tasks": len(records),
        "successes": successes,
        "failures": failures,
        "success_rate_percent": (100.0 * successes / len(records)) if records else None,
        "planning_time": timing_summary(records, "planning_time"),
        "estimated_execution_time": timing_summary(
            records, "estimated_execution_time"
        ),
        "error_types": dict(Counter(
            item.get("error_type") or "unspecified"
            for item in records if not item.get("success")
        )),
        "moveit_error_codes": dict(Counter(
            str(item.get("moveit_error_code", 0)) for item in records
        )),
        "environment_versions": sorted({
            item["environment_version"]
            for item in records if item.get("environment_version") is not None
        }),
    }


class MotionMetricsReporter(Node):
    def __init__(self):
        super().__init__("motion_metrics_reporter")
        self.declare_parameter("result_topic", "/team/motion_result_json")
        self.declare_parameter("report_interval_sec", 60.0)
        self.declare_parameter(
            "report_directory", "/root/ros2_ws/runtime_reports/motion_planning"
        )
        report_override = os.environ.get("MOTION_REPORT_DIR")
        self.report_directory = Path(
            report_override or self.get_parameter("report_directory").value
        )
        self.report_directory.mkdir(parents=True, exist_ok=True)
        self.started_at = datetime.now(timezone.utc)
        self.records = []
        self.pending_records = []
        self.seen_task_ids = set()
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            self.get_parameter("result_topic").value,
            self.result_callback,
            qos,
        )
        self.create_timer(
            float(self.get_parameter("report_interval_sec").value),
            self.write_report,
            clock=self.steady_clock,
        )
        self.get_logger().info(
            f"Writing motion-planning reports every "
            f"{self.get_parameter('report_interval_sec').value:.0f}s to "
            f"{self.report_directory}."
        )

    def result_callback(self, message):
        try:
            record = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().warning(f"Ignoring invalid result JSON: {error}")
            return
        task_id = record.get("task_id")
        if not task_id or task_id in self.seen_task_ids:
            return
        self.seen_task_ids.add(task_id)
        record["received_at"] = datetime.now(timezone.utc).isoformat()
        if task_id.startswith("cost_probe_"):
            record["category"] = "cost_probe"
        elif task_id.startswith("initial_scan_"):
            record["category"] = "initial_scan"
        else:
            record["category"] = "motion_task"
        self.records.append(record)
        self.pending_records.append(record)

    @staticmethod
    def markdown_section(title, summary):
        success_rate = summary["success_rate_percent"]
        rate_text = "n/a" if success_rate is None else f"{success_rate:.1f}%"
        planning = summary["planning_time"]
        execution = summary["estimated_execution_time"]

        def value(item, key):
            result = item[key]
            return "n/a" if result is None else f"{result:.3f} s"

        return "\n".join([
            f"## {title}",
            "",
            f"- Tasks: {summary['tasks']}",
            f"- Success / failure: {summary['successes']} / {summary['failures']}",
            f"- Success rate: {rate_text}",
            f"- Successful-task planning time mean / median / P95: "
            f"{value(planning, 'mean_sec')} / {value(planning, 'median_sec')} / "
            f"{value(planning, 'p95_sec')}",
            f"- Successful-task estimated execution time mean / median / P95: "
            f"{value(execution, 'mean_sec')} / {value(execution, 'median_sec')} / "
            f"{value(execution, 'p95_sec')}",
            f"- Failure types: {summary['error_types'] or {}}",
            f"- MoveIt error codes: {summary['moveit_error_codes'] or {}}",
            f"- Environment versions: {summary['environment_versions']}",
        ])

    @staticmethod
    def atomic_write(path, text):
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(text, encoding="utf-8")
        temporary.replace(path)

    def write_report(self):
        generated_at = datetime.now(timezone.utc)
        motion_all = [r for r in self.records if r["category"] == "motion_task"]
        probes_all = [r for r in self.records if r["category"] == "cost_probe"]
        scans_all = [r for r in self.records if r["category"] == "initial_scan"]
        motion_interval = [
            r for r in self.pending_records if r["category"] == "motion_task"
        ]
        probes_interval = [
            r for r in self.pending_records if r["category"] == "cost_probe"
        ]
        scans_interval = [
            r for r in self.pending_records if r["category"] == "initial_scan"
        ]
        report = {
            "run_started_at": self.started_at.isoformat(),
            "generated_at": generated_at.isoformat(),
            "report_interval_sec": float(
                self.get_parameter("report_interval_sec").value
            ),
            "cumulative": {
                "motion_tasks": summarize(motion_all),
                "initial_scan": summarize(scans_all),
                "cost_probes": summarize(probes_all),
            },
            "latest_interval": {
                "motion_tasks": summarize(motion_interval),
                "initial_scan": summarize(scans_interval),
                "cost_probes": summarize(probes_interval),
            },
            "latest_results": self.pending_records[-20:],
        }
        timestamp = generated_at.strftime("%Y%m%dT%H%M%SZ")
        json_text = json.dumps(report, indent=2, sort_keys=True) + "\n"
        markdown = "\n\n".join([
            "# Motion Planning Runtime Report",
            f"Generated: `{generated_at.isoformat()}`  \n"
            f"Run started: `{self.started_at.isoformat()}`",
            self.markdown_section(
                "Cumulative motion tasks", report["cumulative"]["motion_tasks"]
            ),
            self.markdown_section(
                "Latest interval motion tasks",
                report["latest_interval"]["motion_tasks"],
            ),
            self.markdown_section(
                "Cumulative initial scan (known-free bypass)",
                report["cumulative"]["initial_scan"],
            ),
            self.markdown_section(
                "Cumulative plan-only cost probes",
                report["cumulative"]["cost_probes"],
            ),
            "## Debug Notes\n\n"
            "- `trajectory_not_known_free`: planned path touched unknown or occupied voxels.\n"
            "- `validation_environment_mismatch`: validator map changed during a task.\n"
            "- `conservative_map_failed`: unknown-as-occupied map generation failed before planning.\n"
            "- `moveit_planning_failure`: MoveIt could not produce a trajectory.\n"
            "- `execution_failure`: a validated trajectory failed during execution.\n"
            "- Execution values are trajectory estimates, not measured wall-clock time.",
        ]) + "\n"

        self.atomic_write(self.report_directory / f"report_{timestamp}.json", json_text)
        self.atomic_write(self.report_directory / f"report_{timestamp}.md", markdown)
        self.atomic_write(self.report_directory / "latest.json", json_text)
        self.atomic_write(self.report_directory / "latest.md", markdown)
        self.get_logger().info(
            f"Wrote motion report {timestamp}: "
            f"motion_tasks={len(motion_all)}, initial_scan={len(scans_all)}, "
            f"cost_probes={len(probes_all)}."
        )
        self.pending_records.clear()


def main(args=None):
    rclpy.init(args=args)
    node = MotionMetricsReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
