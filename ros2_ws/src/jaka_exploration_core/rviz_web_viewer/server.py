#!/usr/bin/env python3
"""RViz Web Viewer — 第一版(Linux / X11)。

面向局域网的只读 RViz 展示页 + 本机管理员控制(开始/暂停/复位)。
- 访客:打开 http://<主机局域网IP>:8765 直接观看 RViz(MJPEG),页面只读。
- 管理员:在运行 RViz 的主机上用 http://127.0.0.1:8765 登录后使用控制按钮。

视频:mss 采集 X11 桌面(可选按窗口标题用 wmctrl 定位 RViz 客户区),所有访客
共享同一个采集/编码任务。控制:通过控制适配器发 ROS 2 /exploration_pause(接入
本项目 hppt + self_filter 的暂停/继续),复位为预留适配器。

依赖:fastapi uvicorn mss pillow  (wmctrl 可选,用于按标题定位窗口)
配置见文件末尾 CONFIG 说明 / 环境变量。
"""
from __future__ import annotations

import hashlib
import hmac
import io
import os
import secrets
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from fastapi import FastAPI, Request, Response, Form, HTTPException
from fastapi.responses import (
    HTMLResponse,
    JSONResponse,
    StreamingResponse,
    FileResponse,
)
import uvicorn

try:
    import mss  # type: ignore
    from PIL import Image  # type: ignore
    _HAVE_CAPTURE = True
except Exception:  # pragma: no cover - 采集依赖缺失时降级为占位图
    _HAVE_CAPTURE = False

try:
    import rclpy  # type: ignore
    from rclpy.qos import (  # type: ignore
        qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
        ReliabilityPolicy, HistoryPolicy,
    )
    from sensor_msgs.msg import CompressedImage, Image as RosImage  # type: ignore
    from std_msgs.msg import Bool as RosBool  # type: ignore
    _HAVE_ROS = True
except Exception:  # 未 source ROS 时 topic 源 / pause publisher 不可用
    _HAVE_ROS = False

# 相机原镜头是 raw Image,需 cv2 编码成 JPEG 才能推流(YOLO 输出已是 JPEG,直转)。
try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False


# ============================ 配置 ============================
def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


HOST = _env("RVIZ_WEB_HOST", "0.0.0.0")
PORT = int(_env("RVIZ_WEB_PORT", "8765"))
OUT_W = int(_env("RVIZ_WEB_WIDTH", "1280"))
OUT_H = int(_env("RVIZ_WEB_HEIGHT", "720"))
FPS = float(_env("RVIZ_WEB_FPS", "12"))
JPEG_Q = int(_env("RVIZ_WEB_JPEG_QUALITY", "75"))
# 采集来源:窗口标题(wmctrl 定位)优先;否则用 monitor 序号(mss)。
RVIZ_WINDOW_TITLE = _env("RVIZ_WEB_RVIZ_WINDOW_TITLE", "RViz")
CAPTURE_MONITOR = int(_env("RVIZ_WEB_CAPTURE_MONITOR", "1"))  # mss: 1=主屏
# ★ 视频源:
#   topic = 订阅 ROS CompressedImage(JPEG 直转,不抓屏不编码,不和 RViz 抢显示,
#           推荐;但显示的是该 topic 的画面,如相机,不是 RViz 3D 可视化)。
#   x11   = mss 抓 X11 桌面(能显示 RViz,但会和 RViz 渲染抢显示管线,可能卡)。
SOURCE = _env("RVIZ_WEB_SOURCE", "topic")
IMAGE_TOPIC = _env("RVIZ_WEB_IMAGE_TOPIC",
                   "/camera/camera/color/image_raw/compressed")
# 相机原镜头(compressed,image_transport 编码好的 JPEG ~83KB,直推零编码,和最初
# 单源前端一样的处理——DDS 传得动、不卡)。前端可在 IMAGE_TOPIC(真机=YOLO 标注图)
# 与它之间切换。
CAMERA_TOPIC = _env("RVIZ_WEB_CAMERA_TOPIC",
                    "/camera/camera/color/image_raw/compressed")
# 控制:ROS 暂停 topic(接入 hppt + self_filter)
PAUSE_TOPIC = _env("RVIZ_WEB_PAUSE_TOPIC", "/exploration_pause")
# 管理员凭据:优先 hash(格式 "scrypt$<salt_hex>$<hash_hex>");dev 可用明文 PASSWORD。
ADMIN_USER = _env("RVIZ_WEB_ADMIN_USER", "admin")
ADMIN_HASH = os.environ.get("RVIZ_WEB_ADMIN_HASH", "")
ADMIN_PASSWORD = os.environ.get("RVIZ_WEB_ADMIN_PASSWORD", "")

_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
_LOCAL_HOSTS = {"127.0.0.1", "::1", "localhost"}


# ============================ 密码 (scrypt) ============================
def _scrypt(password: str, salt: bytes) -> bytes:
    return hashlib.scrypt(password.encode("utf-8"), salt=salt, n=2**14, r=8, p=1, dklen=32)


def make_hash(password: str) -> str:
    salt = secrets.token_bytes(16)
    return f"scrypt${salt.hex()}${_scrypt(password, salt).hex()}"


def verify_password(password: str, stored: str) -> bool:
    try:
        algo, salt_hex, hash_hex = stored.split("$", 2)
        if algo != "scrypt":
            return False
        expected = bytes.fromhex(hash_hex)
        actual = _scrypt(password, bytes.fromhex(salt_hex))
        return hmac.compare_digest(expected, actual)  # 恒定时间比较
    except Exception:
        return False


def _resolve_admin_hash() -> Optional[str]:
    if ADMIN_HASH:
        return ADMIN_HASH
    if ADMIN_PASSWORD:
        print("[rviz_web] WARN: 使用明文 RVIZ_WEB_ADMIN_PASSWORD(仅供 dev,生产请用 RVIZ_WEB_ADMIN_HASH)。")
        return make_hash(ADMIN_PASSWORD)
    # 都没配:生成一次性随机密码(dev 便利),打印到日志,不落盘。
    pw = secrets.token_urlsafe(9)
    print(f"[rviz_web] 未配置管理员凭据,已生成临时密码 -> 用户 '{ADMIN_USER}' 密码 '{pw}'(仅本次进程有效)。")
    return make_hash(pw)


ADMIN_HASH_RESOLVED = _resolve_admin_hash()


# ============================ 视频采集(单例共享)============================
def _placeholder_jpeg(text: str) -> bytes:
    """采集不可用时的占位帧。无 PIL 时返回极小的静态 JPEG。"""
    if not _HAVE_CAPTURE:
        # 1x1 黑色 JPEG(最小可用),前端会显示"RViz 未连接"文案。
        return bytes.fromhex(
            "ffd8ffe000104a46494600010100000100010000ffdb004300"
            "08060607060508070707090909080a0c140d0c0b0b0c1912130f"
            "141d1a1f1e1d1a1c1c20242e2720222c231c1c2837292c303134"
            "34341f27393d38323c2e333432ffc0000b080001000101011100"
            "ffc4001f0000010501010101010100000000000000000102030405"
            "060708090a0bffc400b5100002010303020403050504040000017d"
            "01020300041105122131410613516107227114328191a1082342b1"
            "c11552d1f02433627282090a161718191a25262728292a34353637"
            "38393a434445464748494a535455565758595a636465666768696a"
            "737475767778797a838485868788898a92939495969798999aa2a3"
            "a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3"
            "d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa"
            "ffda0008010100003f00fbfeffd9"
        )
    img = Image.new("RGB", (OUT_W, OUT_H), (24, 26, 28))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=JPEG_Q)
    return buf.getvalue()


def _find_window_bbox(title: str) -> Optional[dict]:
    """用 wmctrl -lG 找标题含 title 的窗口客户区 bbox(X11)。找不到返回 None。"""
    if not shutil.which("wmctrl"):
        return None
    try:
        out = subprocess.check_output(["wmctrl", "-lG"], text=True, timeout=1.0)
    except Exception:
        return None
    for line in out.splitlines():
        parts = line.split(None, 7)
        if len(parts) < 8:
            continue
        _id, _desk, x, y, w, h, _host, wtitle = parts
        if title.lower() in wtitle.lower():
            try:
                return {"left": int(x), "top": int(y), "width": int(w), "height": int(h)}
            except ValueError:
                return None
    return None


@dataclass
class CaptureState:
    connected: bool = False
    last_ok_ts: float = 0.0
    frame: bytes = field(default_factory=lambda: _placeholder_jpeg("init"))
    cond: threading.Condition = field(default_factory=threading.Condition)


class CaptureWorker(threading.Thread):
    """单例采集线程:定位 RViz 窗口 → 截屏 → 缩放 → JPEG。所有访客共享 latest frame。"""

    def __init__(self, state: CaptureState):
        super().__init__(daemon=True)
        self.state = state
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        period = 1.0 / max(1.0, FPS)
        if not _HAVE_CAPTURE:
            print("[rviz_web] WARN: 未安装 mss/pillow,视频降级为占位帧(pip install mss pillow)。")
        placeholder = _placeholder_jpeg("no-rviz")
        with (mss.mss() if _HAVE_CAPTURE else _NullCtx()) as sct:
            while not self._stop.is_set():
                t0 = time.time()
                jpeg = None
                if _HAVE_CAPTURE:
                    try:
                        bbox = _find_window_bbox(RVIZ_WINDOW_TITLE)
                        region = bbox if bbox else sct.monitors[CAPTURE_MONITOR]
                        raw = sct.grab(region)
                        img = Image.frombytes("RGB", raw.size, raw.bgra, "raw", "BGRX")
                        img = img.resize((OUT_W, OUT_H))
                        buf = io.BytesIO()
                        img.save(buf, format="JPEG", quality=JPEG_Q)
                        jpeg = buf.getvalue()
                    except Exception:
                        jpeg = None
                if jpeg is not None:
                    with self.state.cond:
                        self.state.frame = jpeg
                        self.state.connected = True
                        self.state.last_ok_ts = time.time()
                        self.state.cond.notify_all()
                else:
                    # 采集失败 → 标记断开,推占位帧(保持流不断)。
                    if time.time() - self.state.last_ok_ts > 2.0:
                        with self.state.cond:
                            self.state.frame = placeholder
                            self.state.connected = False
                            self.state.cond.notify_all()
                dt = time.time() - t0
                self._stop.wait(max(0.0, period - dt))


class _NullCtx:
    def __enter__(self):
        return None

    def __exit__(self, *a):
        return False


class RosImageSource:
    """★ 换采集方式:订阅 ROS CompressedImage(JPEG),直接作为 MJPEG 帧转发 ——
    不抓 X11 屏、不做软件编码,完全避开和 RViz 抢显示管线 / 抢 CPU。
    显示的是该 topic 的画面(默认 realsense 相机压缩图)。"""

    def __init__(self, state: CaptureState, topic: str):
        self.state = state
        self.topic = topic
        self._node = None
        self._thread = None
        self._own_rclpy = False

    def start(self):
        if not _HAVE_ROS:
            print("[rviz_web] WARN: 无 rclpy/sensor_msgs,image-topic 源不可用 —— "
                  "请在 source 了 ROS 的环境启动,或改用 RVIZ_WEB_SOURCE=x11。")
            return
        try:
            if not rclpy.ok():
                rclpy.init()
                self._own_rclpy = True
            self._node = rclpy.create_node("rviz_web_image_sub")
            self._node.create_subscription(
                CompressedImage, self.topic, self._cb, qos_profile_sensor_data)
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
            print(f"[rviz_web] 视频源 = ROS topic '{self.topic}'(零抓屏零编码)")
        except Exception as e:  # noqa
            print(f"[rviz_web] WARN: 订阅 {self.topic} 失败:{e}")

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def _cb(self, msg):
        # CompressedImage.data 已是 JPEG 字节流,直接推流。
        with self.state.cond:
            self.state.frame = bytes(msg.data)
            self.state.connected = True
            self.state.last_ok_ts = time.time()
            self.state.cond.notify_all()

    def stop(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
            if self._own_rclpy and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


class DualTopicSource:
    """一个 ROS node 同时订阅两路视频源,各自维护 latest JPEG,供前端切换:
      - YOLO 输出(yolo_topic, CompressedImage):已是 JPEG,直接转发;
      - 相机原镜头(cam_topic, raw Image):用 cv2 编码成 JPEG。
    两路都用 sensor_data(best_effort)QoS,与相机/YOLO 发布端匹配。"""

    def __init__(self, yolo_state: CaptureState, cam_state: CaptureState,
                 yolo_topic: str, cam_topic: str):
        self.yolo_state = yolo_state
        self.cam_state = cam_state
        self.yolo_topic = yolo_topic
        self.cam_topic = cam_topic
        self._node = None
        self._thread = None
        self._own_rclpy = False

    def start(self):
        if not _HAVE_ROS:
            print("[rviz_web] WARN: 无 rclpy/sensor_msgs,视频 topic 源不可用 —— "
                  "请在 source 了 ROS 的环境启动,或改用 RVIZ_WEB_SOURCE=x11。")
            return
        try:
            if not rclpy.ok():
                rclpy.init()
                self._own_rclpy = True
            self._node = rclpy.create_node("rviz_web_image_sub")
            self._node.create_subscription(
                CompressedImage, self.yolo_topic, self._yolo_cb, qos_profile_sensor_data)
            self._node.create_subscription(
                CompressedImage, self.cam_topic, self._cam_cb, qos_profile_sensor_data)
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
            print(f"[rviz_web] 双视频源就绪(均 compressed 直推): "
                  f"YOLO='{self.yolo_topic}'  相机原图='{self.cam_topic}'")
        except Exception as e:  # noqa
            print(f"[rviz_web] WARN: 视频源订阅失败:{e}")

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def _yolo_cb(self, msg):
        # CompressedImage.data 已是 JPEG,直接推。
        self._update(self.yolo_state, bytes(msg.data))

    def _cam_cb(self, msg):
        # 相机 compressed 已是 JPEG,直接推(零软件编码,与 YOLO 源、最初单源前端一样)。
        self._update(self.cam_state, bytes(msg.data))

    @staticmethod
    def _update(state: CaptureState, jpeg: bytes):
        with state.cond:
            state.frame = jpeg
            state.connected = True
            state.last_ok_ts = time.time()
            state.cond.notify_all()

    def stop(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
            if self._own_rclpy and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


# ============================ 会话 / 限流 ============================
_sessions: dict[str, float] = {}          # token -> created_ts(内存,重启即失效)
_SESSION_TTL = float(_env("RVIZ_WEB_SESSION_TTL_SEC", "3600"))
_login_fail: dict[str, list] = {}         # ip -> [ts,...] 最近失败,短时限流
_LOGIN_WINDOW = 60.0
_LOGIN_MAX = 5


def _is_local(request: Request) -> bool:
    host = request.client.host if request.client else ""
    return host in _LOCAL_HOSTS


def _new_session() -> str:
    tok = secrets.token_urlsafe(24)
    _sessions[tok] = time.time()
    return tok


def _valid_session(request: Request) -> bool:
    tok = request.cookies.get("rviz_admin")
    if not tok or tok not in _sessions:
        return False
    if time.time() - _sessions[tok] > _SESSION_TTL:
        _sessions.pop(tok, None)
        return False
    return True


def _rate_limited(ip: str) -> bool:
    now = time.time()
    hits = [t for t in _login_fail.get(ip, []) if now - t < _LOGIN_WINDOW]
    _login_fail[ip] = hits
    return len(hits) >= _LOGIN_MAX


# ============================ 控制适配器 ============================
class ControlAdapter:
    """把 开始/暂停/复位 映射到实际后端。第一版:暂停/继续 → ROS /exploration_pause;
    复位 → 预留(等项目提供 reset service/脚本再替换)。串行执行,返回 (ok,state,message)。"""

    def __init__(self):
        self.state = "unknown"
        self._lock = threading.Lock()
        self._pause_node = None
        self._pause_pub = None

    def start_ros(self):
        # ★ 持久 latched pause publisher(transient_local):替换不可靠的
        #   `ros2 topic pub -1`(临时 publisher + volatile,DDS discovery 竞态会
        #   让部分订阅者丢消息 —— self_filter 丢了 pause=false 就永远卡在 paused)。
        #   transient_local latch → self_filter/hppt 无论何时 discovery 都拿到最新值。
        if not _HAVE_ROS:
            print("[rviz_web] WARN: 无 rclpy,pause 控制不可用(需 source ROS)。")
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            self._pause_node = rclpy.create_node("rviz_web_control")
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
            self._pause_pub = self._pause_node.create_publisher(RosBool, PAUSE_TOPIC, qos)
            print(f"[rviz_web] pause publisher ready (transient_local latched) on {PAUSE_TOPIC}")
        except Exception as e:  # noqa
            print(f"[rviz_web] WARN: pause publisher 初始化失败:{e}")

    def _pub_pause(self, value: bool) -> tuple[bool, str]:
        if self._pause_pub is None:
            return False, "ROS pause publisher 未就绪(需在 source 了 ROS 的环境启动本服务)。"
        m = RosBool()
        m.data = bool(value)
        self._pause_pub.publish(m)
        return True, ""

    def start(self) -> tuple[bool, str, str]:
        with self._lock:
            ok, err = self._pub_pause(False)  # 继续 = 取消暂停
            if ok:
                self.state = "running"
                return True, self.state, "系统已启动/继续"
            return False, self.state, err or "启动失败"

    def pause(self) -> tuple[bool, str, str]:
        with self._lock:
            ok, err = self._pub_pause(True)
            if ok:
                self.state = "paused"
                return True, self.state, "系统已暂停(当前任务完成后停止)"
            return False, self.state, err or "暂停失败"

    def reset(self) -> tuple[bool, str, str]:
        # 复位 = 机械臂 MoveIt 规划(避障)回初始 home 位姿。调 go_home 节点执行,
        #   阻塞等它完成(回 home 慢速,可能几十秒)。
        with self._lock:
            self.state = "resetting"
            if not shutil.which("ros2"):
                self.state = "unknown"
                return False, self.state, "未找到 ros2(请在 source 了 ROS 的环境启动本服务)。"
            # ★ 复位前先暂停探索(/exploration_pause true):hppt 不再派发新观察点,
            #   避免"回 home"和探索选点互相抢;self_filter 同步停 octomap 更新。
            #   复位后系统保持暂停,想继续探索点"开始/继续"即可。
            self._pub_pause(True)
            try:
                r = subprocess.run(
                    ["ros2", "run", "jaka_exploration_core", "go_home"],
                    capture_output=True, text=True, timeout=240.0)
                ok = (r.returncode == 0)
                self.state = "idle" if ok else "unknown"
                if ok:
                    return True, self.state, "机械臂已复位回初始位姿"
                tail = (r.stderr or r.stdout or "").strip()[-200:]
                return False, self.state, f"复位失败:{tail}"
            except subprocess.TimeoutExpired:
                self.state = "unknown"
                return False, self.state, "复位超时(>240s)"


# ============================ FastAPI 应用 ============================
app = FastAPI(title="RViz Web Viewer")
_yolo_capture = CaptureState()   # IMAGE_TOPIC 源(真机 launch=YOLO 标注图)
_cam_capture = CaptureState()    # 相机原镜头(raw Image → JPEG)
_worker = None   # startup 时按 SOURCE 选择:DualTopicSource(topic) 或 CaptureWorker(x11)
_control = ControlAdapter()


def _capture_for(source: str) -> CaptureState:
    """前端 ?source= 参数 → 对应视频 buffer。camera=相机原图,其余=YOLO/IMAGE_TOPIC。"""
    return _cam_capture if source == "camera" else _yolo_capture


@app.on_event("startup")
def _startup():
    global _worker
    _control.start_ros()   # 先建 latched pause publisher(会 rclpy.init)
    if SOURCE == "topic":
        _worker = DualTopicSource(_yolo_capture, _cam_capture, IMAGE_TOPIC, CAMERA_TOPIC)
    else:
        _worker = CaptureWorker(_yolo_capture)   # x11 抓屏填 YOLO buffer(作默认单源)
    _worker.start()
    print(f"[rviz_web] serving on http://{HOST}:{PORT}  source={SOURCE}  "
          f"(admin login only from 127.0.0.1)")


@app.on_event("shutdown")
def _shutdown():
    if _worker is not None:
        _worker.stop()


@app.get("/", response_class=HTMLResponse)
def index():
    path = os.path.join(_STATIC_DIR, "index.html")
    return FileResponse(path)


def _mjpeg_gen(capture: CaptureState):
    boundary = b"--frame"
    while True:
        with capture.cond:
            capture.cond.wait(timeout=1.0)
            frame = capture.frame
        yield (boundary + b"\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
               + frame + b"\r\n")


@app.get("/stream/rviz.mjpg")
def stream(source: str = "yolo"):
    # source=camera → 相机原镜头;其余(yolo)→ IMAGE_TOPIC(真机=YOLO 标注图)。
    return StreamingResponse(
        _mjpeg_gen(_capture_for(source)),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-cache, no-store", "Pragma": "no-cache"},
    )


@app.get("/api/status")
def status(source: str = "yolo"):
    cap = _capture_for(source)
    connected = cap.connected and (time.time() - cap.last_ok_ts < 3.0)
    return {
        "video": "connected" if connected else "disconnected",
        "state": _control.state,
        "source": source,
    }


@app.get("/api/auth/session")
def auth_session(request: Request):
    return {"admin": _valid_session(request), "is_local": _is_local(request)}


@app.post("/api/auth/login")
def auth_login(request: Request, username: str = Form(...), password: str = Form(...)):
    if not _is_local(request):
        raise HTTPException(status_code=403, detail="管理员登录仅限本机(127.0.0.1)。")
    ip = request.client.host if request.client else "?"
    if _rate_limited(ip):
        raise HTTPException(status_code=429, detail="登录尝试过多,请稍后再试。")
    ok = (username == ADMIN_USER) and ADMIN_HASH_RESOLVED and verify_password(password, ADMIN_HASH_RESOLVED)
    if not ok:
        _login_fail.setdefault(ip, []).append(time.time())
        raise HTTPException(status_code=401, detail="用户名或密码错误。")
    tok = _new_session()
    resp = JSONResponse({"ok": True})
    resp.set_cookie("rviz_admin", tok, httponly=True, samesite="strict", path="/")
    return resp


@app.post("/api/auth/logout")
def auth_logout(request: Request):
    tok = request.cookies.get("rviz_admin")
    if tok:
        _sessions.pop(tok, None)
    resp = JSONResponse({"ok": True})
    resp.delete_cookie("rviz_admin", path="/")
    return resp


def _require_admin_local(request: Request):
    if not _is_local(request):
        raise HTTPException(status_code=403, detail="控制接口仅限本机。")
    if not _valid_session(request):
        raise HTTPException(status_code=403, detail="需要管理员会话。")


@app.post("/api/control/start")
def control_start(request: Request):
    _require_admin_local(request)
    ok, state, msg = _control.start()
    return {"ok": ok, "state": state, "message": msg}


@app.post("/api/control/pause")
def control_pause(request: Request):
    _require_admin_local(request)
    ok, state, msg = _control.pause()
    return {"ok": ok, "state": state, "message": msg}


@app.post("/api/control/reset")
def control_reset(request: Request):
    _require_admin_local(request)
    ok, state, msg = _control.reset()
    return {"ok": ok, "state": state, "message": msg}


def main():
    uvicorn.run(app, host=HOST, port=PORT, log_level="warning")


if __name__ == "__main__":
    main()
