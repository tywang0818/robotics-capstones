# ros2_ws/scripts/

TSP + JAKA workspace 的容器内部署和启动工具。脚本自动将其父目录识别为
ROS 2 workspace，因此默认位置是 `/root/ros2_ws/scripts/`。

## 用法一览

```bash
# 1. 首次安装依赖 + colcon build（一次即可）
bash /root/ros2_ws/scripts/container_deploy.sh

# 2. 每次运行（推荐：一个终端启动 JAKA + TSP）
bash /root/ros2_ws/scripts/container_start_all.sh
# 传参：
bash /root/ros2_ws/scripts/container_start_all.sh \
  --ros-args -p reach_min:=0.55

# 或者两个终端调试用：
# 终端 A（只启动 JAKA 仿真和 motion planning）
bash /root/ros2_ws/scripts/container_start_jaka.sh
# 终端 B
ros2 run hppt_boundary_ros hppt_boundary_ros

# 使用真实 RealSense 建 OctoMap，再让 HPP-PT / motion planning 使用该地图
bash /root/ros2_ws/scripts/container_start_all_realsense.sh

# 可选：在 Gazebo RGB-D 流上启动 SegFormer 语义分割节点
START_SEMANTIC=true bash /root/ros2_ws/scripts/container_start_all.sh

# 停止 / 清理
bash /root/ros2_ws/scripts/container_kill_all.sh
```

完整架构、参数和故障排查见仓库根目录 [README.md](../../README.md)。
