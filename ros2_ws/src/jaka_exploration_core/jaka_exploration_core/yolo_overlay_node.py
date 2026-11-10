#!/usr/bin/env python3
"""旁路 YOLO 叠加节点(bypass overlay)。

唯一与主系统的关联:订阅主系统相机的 RGB 图像。它做的事:
  1) 用 YOLOv8(COCO 80 类,默认 GPU)对每帧 RGB 做目标检测;
  2) 在图上用矩形框框出物品并标注「类别 + 置信度」;
  3) 把画好框的图 JPEG 编码后发布成一个 CompressedImage topic。

前端(RViz Web Viewer)的视频源指向本节点的输出 topic,就能在视频流里
看到框选效果。本节点 **完全旁路**:除了订阅相机 RGB,只发布自己的标注
图像 topic,不发布任何主系统(hppt/octomap/motion)会消费的消息,因此
不会影响探索决策与机械臂控制。

YOLO 推理跑在 GPU(CUDA),而主系统的 ROS 决策/控制跑在 CPU,两者不抢
同一资源;订阅 QoS 用 best_effort+depth1,推理跟不上时自动丢旧帧,不积压。
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

import cv2


def _image_to_bgr(msg: Image) -> np.ndarray:
    """把 ROS Image 手动解成 OpenCV 的 BGR ndarray(不依赖 cv_bridge)。"""
    encoding = msg.encoding.lower()
    if encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # OpenCV 与 ultralytics 都按 BGR 处理,rgb8 需翻通道。
        return arr[..., ::-1].copy() if encoding == "rgb8" else arr.copy()
    if encoding in ("rgba8", "bgra8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        bgr = arr[..., [2, 1, 0]] if encoding == "rgba8" else arr[..., :3]
        return bgr.copy()
    if encoding in ("mono8", "8uc1"):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported image encoding for YOLO overlay: {msg.encoding}")


def _color_for_class(class_id: int) -> tuple:
    """按 class_id 生成稳定、区分度高的 BGR 颜色(同一类每帧同色)。"""
    hue = int((class_id * 47) % 180)  # 47 与 180 互质,类别间色相分散
    hsv = np.uint8([[[hue, 200, 255]]])
    b, g, r = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(b), int(g), int(r)


class YoloOverlay(Node):
    def __init__(self) -> None:
        super().__init__("yolo_overlay")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("output_topic", "/yolo/annotated/compressed")
        self.declare_parameter("model_path", "/root/ros2_ws/yolov8n.pt")
        self.declare_parameter("device", "auto")           # auto/cuda:0/cpu
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("jpeg_quality", 80)         # 1-100
        self.declare_parameter("class_filter", [""])       # 空=全部 COCO;否则只画这些类名
        self.declare_parameter("max_rate_hz", 15.0)        # 推理限流,0=不限
        self.declare_parameter("cpu_threads", 4)           # CPU 推理线程上限(0=不限),防吃满核抢主系统

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        model_path = str(self.get_parameter("model_path").value)
        device_param = str(self.get_parameter("device").value)
        self.conf = float(self.get_parameter("confidence_threshold").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        raw_filter = [str(x).strip().lower() for x in self.get_parameter("class_filter").value]
        self.class_filter = {x for x in raw_filter if x}   # 空集合 = 不过滤
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self._min_interval = (1.0 / self.max_rate_hz) if self.max_rate_hz > 0 else 0.0
        self._last_infer_t = 0.0
        self._first = True
        self._last_warn_t = 0.0

        # 选设备:auto → 试 GPU;若 GPU 的 CUDA kernel 与当前 torch 不兼容
        #   (新架构 + 旧 torch,如 RTX 50 系 sm_120 + torch cu121),预热会崩,
        #   自动回退 CPU。显式 cpu/cuda:0 则不回退。
        allow_fallback = (device_param == "auto")
        if device_param == "auto":
            want = "cpu"
            try:
                import torch
                if torch.cuda.is_available():
                    # 真实 kernel 探测:.item() 会同步并立即暴露「GPU 架构与 torch
                    #   不匹配」这类异步 CUDA 错误。is_available() 只看驱动,探测
                    #   不到 kernel 不兼容(如 RTX 50 系 sm_120 + torch cu121)。
                    probe = torch.zeros(8, device="cuda:0")
                    float((probe + 1).sum().item())
                    want = "cuda:0"
            except Exception:
                want = "cpu"
        else:
            want = device_param

        # 限制 torch 线程数:CPU 推理默认会吃满所有核,会抢主系统的实时控制
        #   回路。限到少数几个核,旁路可视化够用又不干扰探索/机械臂。
        cpu_threads = int(self.get_parameter("cpu_threads").value)
        if cpu_threads > 0:
            try:
                import torch
                torch.set_num_threads(cpu_threads)
            except Exception:
                pass

        from ultralytics import YOLO  # 运行时导入,避免主系统无谓依赖
        self.model = YOLO(model_path)
        self.names = self.model.names   # {class_id: name}
        # 预热一帧(顺带探测设备可用性),避免首帧初始化延迟拖慢在线流。
        self.device = self._warmup_or_fallback(want, allow_fallback)

        self.pub = self.create_publisher(
            CompressedImage, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            CompressedImage, self.input_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"yolo_overlay: {self.input_topic} --YOLO({self.device})--> "
            f"{self.output_topic} | model={model_path} conf={self.conf} "
            f"classes={'ALL' if not self.class_filter else sorted(self.class_filter)}")

    def _warmup_or_fallback(self, device: str, allow_fallback: bool) -> str:
        """预热一帧;若在 device 上推理失败且允许回退,则退回 CPU。"""
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            self.model.predict(dummy, device=device, verbose=False)
            if str(device).startswith("cuda"):
                import torch
                torch.cuda.synchronize()   # 强制同步,让异步 kernel 错误在此浮现
            return device
        except Exception as exc:  # noqa
            if allow_fallback and device != "cpu":
                self.get_logger().warn(
                    f"YOLO 在 {device} 上不可用({type(exc).__name__});回退 CPU。"
                    " 常见于新 GPU 架构 + 旧 torch(CUDA kernel 不匹配)。")
                try:
                    self.model.predict(dummy, device="cpu", verbose=False)
                except Exception as exc2:  # noqa
                    self.get_logger().error(f"CPU 预热也失败:{exc2}")
                return "cpu"
            self.get_logger().warn(f"YOLO 预热失败(继续):{exc}")
            return device

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warn_t >= 2.0:
            self.get_logger().warning(message)
            self._last_warn_t = now

    def _cb(self, msg) -> None:
        # 限流:GPU 跟不上时按目标帧率丢帧(best_effort 订阅本就会丢旧帧)。
        now = time.monotonic()
        if self._min_interval and (now - self._last_infer_t) < self._min_interval:
            return
        self._last_infer_t = now

        try:
            # 输入是 compressed(JPEG),cv2.imdecode 解成 BGR。订阅 compressed 而非
            #   raw:DDS 本机传 raw(900KB)会丢帧卡顿,compressed(~83KB)小、传得动。
            bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError("cv2.imdecode 返回 None(输入非 JPEG?)")
        except Exception as exc:  # noqa
            self._warn_throttled(f"解码相机图失败:{exc}")
            return

        try:
            results = self.model.predict(
                bgr, device=self.device, conf=self.conf, verbose=False)
        except Exception as exc:  # noqa
            self._warn_throttled(f"YOLO 推理失败:{exc}")
            return

        annotated = self._draw(bgr, results[0] if results else None)

        ok, buf = cv2.imencode(
            ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            self._warn_throttled("JPEG 编码失败,丢弃该帧。")
            return

        out = CompressedImage()
        out.header = msg.header       # 沿用相机 header(frame_id/stamp)
        out.format = "jpeg"
        out.data = buf.tobytes()
        self.pub.publish(out)

        if self._first:
            self._first = False
            self.get_logger().info(
                f"yolo_overlay: 首帧已标注并发布到 {self.output_topic}。")

    def _draw(self, bgr: np.ndarray, result) -> np.ndarray:
        """在 BGR 图上画所有检测框 + 「类别 conf」标签。"""
        if result is None or getattr(result, "boxes", None) is None:
            return bgr
        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
        cls = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else np.asarray(boxes.cls)
        conf = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)

        for i in range(len(xyxy)):
            class_id = int(cls[i])
            name = str(self.names.get(class_id, class_id))
            if self.class_filter and name.lower() not in self.class_filter:
                continue
            x1, y1, x2, y2 = (int(round(v)) for v in xyxy[i])
            color = _color_for_class(class_id)
            cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)

            label = f"{name} {conf[i]:.2f}"
            (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ytop = max(0, y1 - th - base - 2)
            cv2.rectangle(bgr, (x1, ytop), (x1 + tw + 2, y1), color, -1)
            cv2.putText(bgr, label, (x1 + 1, y1 - base - 1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
        return bgr


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
