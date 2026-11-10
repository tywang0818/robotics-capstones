# RViz Web Viewer（第一版 / Linux）

局域网只读观看 RViz + 本机管理员控制（开始 / 暂停 / 复位）。视频用 `mss` 采集
X11 桌面（可选按窗口标题用 `wmctrl` 定位 RViz 客户区），控制按钮通过控制适配器
发 ROS 2 `/exploration_pause`（接入本项目 hppt + self_filter 的暂停/继续）。

> 环境：纯 Linux / X11（不涉及 WSL）。Web 服务与 RViz 同一 `DISPLAY`。

## 依赖

```bash
pip install -r requirements.txt      # fastapi uvicorn mss pillow
sudo apt-get install -y wmctrl       # 仅 x11 抓屏源需要：按窗口标题定位 RViz
```

## 视频源(避免卡顿的关键)——`RVIZ_WEB_SOURCE`

| 源 | 显示内容 | 性能 | 说明 |
|---|---|---|---|
| **`topic`(默认)** | 该 image topic 的画面(默认 realsense 相机) | **不卡** | 订阅 `CompressedImage`(JPEG)直接转发,**不抓屏、不软件编码**,完全避开和 RViz 抢 X11 显示管线 / 抢 CPU |
| `x11` | RViz 3D 可视化 | 可能卡 | `mss` 抓 X11 桌面,会和 RViz 渲染争显示管线 |

- 默认 `topic` 源根治了「加前端后机械臂/画面一卡一卡」——采集不再碰显示管线。
- 但 `topic` 显示的是相机画面;**RViz 没有 image topic 输出,要看 RViz 只能用 `x11` 抓屏**(那就回到竞争,靠下面的实时隔离 + 降负载缓解)。
- 切换:`RVIZ_WEB_SOURCE=x11`;换 topic:`RVIZ_WEB_IMAGE_TOPIC=/your/image/compressed`。

## 实时性隔离(launch 自动)

真机 launch 默认 `isolate_realtime:=true`:
- 采集进程 `nice -n 15`(降优先级);
- `jaka_scan_and_publish`(连臂、下发轨迹的实时节点)起来后自动 `chrt -f 80`(SCHED_FIFO)+ `taskset -c 0-3`(绑独立核),保证轨迹周期不被抢占 → 机械臂平滑。失败无害(不影响连臂节点)。关掉:`isolate_realtime:=false`。

## 随真机 launch 自动启动（推荐）

`real_arm_exploration.launch.py` 已集成，默认 `web_viewer:=true`：

```bash
ros2 launch jaka_exploration_core real_arm_exploration.launch.py
```

- 启动后本机自动 `xdg-open http://127.0.0.1:8765`（关掉：`web_viewer_open:=false`）。
- 局域网访客用 `http://<主机局域网IP>:8765` 观看（只读）。
- 完全关掉前端：`web_viewer:=false`。

## 单独启动（调试）

```bash
# 需先 source ROS（控制适配器要调 ros2 topic pub）
python3 rviz_web_viewer/server.py
```

## 管理员登录

- **仅本机**（`127.0.0.1` / `::1`）可登录和调用控制接口；局域网访客直接调用控制接口返回 `403`。
- 凭据来源（优先级从高到低）：
  1. `RVIZ_WEB_ADMIN_HASH`（`scrypt$<salt_hex>$<hash_hex>`，生产推荐，不落明文）
  2. `RVIZ_WEB_ADMIN_PASSWORD`（明文，仅 dev，会打警告）
  3. 都不设 → 启动时生成一次性随机密码并打印到日志（仅本进程有效）
- 生成 hash：
  ```bash
  python3 -c "import sys; sys.path.insert(0,'rviz_web_viewer'); import server; print(server.make_hash('你的密码'))"
  export RVIZ_WEB_ADMIN_HASH='scrypt$....'
  ```

## 控制映射（第一版）

| 按钮 | 动作 | 说明 |
|---|---|---|
| 开始 / 继续 | `ros2 topic pub /exploration_pause {data: false}` | 取消暂停，恢复探索 + octomap 更新 |
| 暂停 | `ros2 topic pub /exploration_pause {data: true}` | 当前任务跑完后停，octomap 停更新 |
| 复位 | 预留适配器（模拟） | 等项目 reset service/脚本接入后替换 `ControlAdapter.reset()` |

## 主要环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `RVIZ_WEB_PORT` | 8765 | 监听端口 |
| `RVIZ_WEB_HOST` | 0.0.0.0 | 监听地址 |
| `RVIZ_WEB_RVIZ_WINDOW_TITLE` | RViz | wmctrl 按标题定位窗口；找不到则采 `CAPTURE_MONITOR` 整屏 |
| `RVIZ_WEB_CAPTURE_MONITOR` | 1 | mss 显示器序号（1=主屏）|
| `RVIZ_WEB_WIDTH` / `_HEIGHT` | 1280 / 720 | 输出分辨率 |
| `RVIZ_WEB_FPS` | 12 | 采集帧率 |
| `RVIZ_WEB_JPEG_QUALITY` | 75 | JPEG 质量 |
| `RVIZ_WEB_PAUSE_TOPIC` | /exploration_pause | 暂停/继续 topic |
| `RVIZ_WEB_ADMIN_USER` | admin | 管理员用户名 |

## 接口（对应方案 §5）

`GET /`、`GET /stream/rviz.mjpg`、`GET /api/status`、`GET /api/auth/session` 公开；
`POST /api/auth/login`（仅本机）、`/api/auth/logout`、
`POST /api/control/{start,pause,reset}`（管理员且仅本机）。控制返回
`{ok, state, message}`，`state ∈ {idle,running,paused,resetting,unknown}`。

## 已知限制 / 待验证

- **视频采集需在目标机验证**：`mss` 采集依赖 X11 `DISPLAY`；`wmctrl` 定位需窗口可见、
  未最小化。窗口最小化时进入不可采集态，页面显示“RViz 未连接”并自动重试。
- 所有访客共享**单个**采集/编码任务（`CaptureWorker` 单例）。
- 会话存内存，服务重启即失效。
- 第一版不含：网页端视角控制、多视窗、用户注册、公网/HTTPS。公网需另加 HTTPS +
  反向代理 + 更完整认证（见方案 §6）。
