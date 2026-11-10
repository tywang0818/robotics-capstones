#!/usr/bin/env python3
"""Periodically snapshot octomap_server maps through its official saver."""
from datetime import datetime
from pathlib import Path
import subprocess

import rclpy
from rclpy.node import Node


class PeriodicOctomapSaver(Node):
    def __init__(self):
        super().__init__("periodic_octomap_saver")
        self.declare_parameter("save_interval", 60.0)
        self.declare_parameter("save_directory", "/root/ros2_ws/maps")

        interval = float(self.get_parameter("save_interval").value)
        self.output_dir = Path(
            str(self.get_parameter("save_directory").value)
        ).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if interval <= 0.0:
            raise ValueError("save_interval must be greater than zero")

        self.get_logger().info(
            f"Saving .bt and .ot snapshots every {interval:.1f}s to "
            f"{self.output_dir}"
        )
        self.timer = self.create_timer(interval, self._save)

    def _save_one(self, path: Path) -> bool:
        command = [
            "ros2", "run", "octomap_server", "octomap_saver_node",
            "--ros-args", "-p", f"octomap_path:={path}",
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Timed out saving {path}")
            return False

        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            self.get_logger().error(
                f"Failed to save {path} (exit {result.returncode}): {detail}"
            )
            return False
        return True

    def _save(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bt_path = self.output_dir / f"octomap_{stamp}.bt"
        ot_path = self.output_dir / f"octomap_{stamp}.ot"

        bt_ok = self._save_one(bt_path)
        ot_ok = self._save_one(ot_path)
        if bt_ok and ot_ok:
            self.get_logger().info(f"Saved {bt_path.name} and {ot_path.name}")


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicOctomapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
