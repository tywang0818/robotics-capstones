#!/usr/bin/env python3
"""复位 = 机械臂 MoveIt 规划(避障)回初始 home 位姿。

home = initial-scan 起始竖直姿态 [J1..J6] = [-90, 90, 0, 0, 90, 0]°。
做法:发一个 joint-goal 的 task JSON 到 decision_bridge(/team/viewpoint_task_json),
decision_bridge 转成 PlanMotion(JOINT_GOAL)→ motion_planner MoveIt 规划避障 + 执行;
订阅 /team/motion_result_json 等真实结果。退出码 0=成功,1=失败/超时。

用途:
  1) 前端"复位"按钮(server.py 的 ControlAdapter.reset 调用本节点);
  2) hppt 收敛退出后,launch 自动先复位再关闭整个系统。
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HOME_DEG = [-90.0, 90.0, 0.0, 0.0, 90.0, 0.0]        # J1..J6,与 jaka_scan_and_publish 的 HOME 一致
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class GoHome(Node):
    def __init__(self):
        super().__init__("go_home")
        self.declare_parameter("task_topic", "/team/viewpoint_task_json")
        self.declare_parameter("result_topic", "/team/motion_result_json")
        self.declare_parameter("env_status_topic", "/team/environment_status_json")
        self.declare_parameter("velocity_scaling", 0.05)
        self.declare_parameter("timeout_sec", 200.0)     # MoveIt 规划+慢速执行回 home
        self.declare_parameter("env_wait_sec", 10.0)
        self.declare_parameter("max_attempts", 10)    # 复位规划失败时外部重试次数

        self.task_topic = self.get_parameter("task_topic").value
        self.result_topic = self.get_parameter("result_topic").value
        self.env_topic = self.get_parameter("env_status_topic").value
        self.vel = float(self.get_parameter("velocity_scaling").value)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        self.env_wait = float(self.get_parameter("env_wait_sec").value)
        self.max_attempts = int(self.get_parameter("max_attempts").value)

        self.env_version = None
        self.result = None
        self.task_id = f"go_home_{self.get_clock().now().nanoseconds}"

        self.pub = self.create_publisher(String, self.task_topic, 10)
        self.create_subscription(String, self.env_topic, self._env_cb, 10)
        self.create_subscription(String, self.result_topic, self._res_cb, 10)

    def _env_cb(self, msg):
        try:
            v = json.loads(msg.data).get("version")
            if isinstance(v, int) and not isinstance(v, bool):
                self.env_version = v
        except Exception:
            pass

    def _res_cb(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get("task_id") == self.task_id:
                self.result = d
        except Exception:
            pass

    def dispatch(self):
        # 每次(含重试)用新 task_id 并清空上次结果:_res_cb 按 task_id 匹配,
        # 避免把上一次旧结果误当本次;environment_version 取最新(持续订阅刷新)。
        self.task_id = f"go_home_{self.get_clock().now().nanoseconds}"
        self.result = None
        task = {
            "task_id": self.task_id,
            "environment_version": self.env_version if self.env_version is not None else 0,
            "goal": {
                "type": "joint",
                "names": JOINT_NAMES,
                "positions": [math.radians(x) for x in HOME_DEG],
            },
            "use_current_start_state": True,
            "plan_only": False,
            "planning_group": "jaka_a5",
            "velocity_scaling": self.vel,
            "acceleration_scaling": self.vel,
        }
        m = String()
        m.data = json.dumps(task, separators=(",", ":"))
        self.pub.publish(m)
        self.get_logger().info(
            f"go_home: dispatched joint goal (env_v={self.env_version}) "
            f"target={HOME_DEG} deg, task_id={self.task_id}")


def main(args=None):
    rclpy.init(args=args)
    node = GoHome()
    t0 = time.time()

    # 1) 等拿到 env version(motion_planner 会校验;拿不到就用 0,可能被拒)
    while rclpy.ok() and node.env_version is None and time.time() - t0 < node.env_wait:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.env_version is None:
        node.get_logger().warn(
            "go_home: 未在 %ds 内拿到 env version,用 version=0 尝试(可能被 motion_planner 拒)。"
            % int(node.env_wait))

    # 2/3) 派发 + 等结果,失败则重试(最多 max_attempts 次)。
    #   trajectory_not_known_free 常是轨迹擦到极少数 occupied voxel(如 occupied=1),
    #   OMPL 随机采样重规划一次多半能绕开;每次都走完整 free-validation,不放宽安全。
    ok = False
    for attempt in range(1, node.max_attempts + 1):
        node.dispatch()   # 新 task_id + 清空 result + 最新 env_version
        node.get_logger().info(
            f"go_home: 第 {attempt}/{node.max_attempts} 次规划复位(env_v={node.env_version})...")
        deadline = time.time() + node.timeout
        while rclpy.ok() and node.result is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        ok = bool(node.result and node.result.get("success"))
        if node.result is None:
            node.get_logger().error(
                f"go_home: 第 {attempt} 次 {int(node.timeout)}s 内未收到结果,超时。")
        else:
            node.get_logger().info(
                f"go_home: 第 {attempt} 次结果 success={ok} "
                f"error_type='{node.result.get('error_type','')}' "
                f"msg='{node.result.get('message','')}'")
        if ok:
            break
        if attempt < node.max_attempts:
            node.get_logger().warn(
                f"go_home: 复位第 {attempt} 次失败,重新规划重试...")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
