# ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2 development container
#
# Designed for the autonomous manipulator environment-exploration project:
#   - JAKA A5 in Gazebo Harmonic
#   - MoveIt 2 motion planning
#   - OctoMap support
#   - Python deps for the exploration / perception algorithms
#
# Image:  jaka_exploration:jazzy
# Build:  bash docker/build.sh
# Run:    bash docker/run.sh

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Build-time proxy support. build.sh passes these through when a host proxy
# at 127.0.0.1:7897 is detected. The values are cleared at the end of this
# Dockerfile so the runtime image does not inherit them.
ARG HTTP_PROXY=
ARG HTTPS_PROXY=
ARG NO_PROXY=
ENV HTTP_PROXY=${HTTP_PROXY} \
    HTTPS_PROXY=${HTTPS_PROXY} \
    NO_PROXY=${NO_PROXY} \
    http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY} \
    no_proxy=${NO_PROXY}

SHELL ["/bin/bash", "-c"]

# ★ 换清华源避免 archive.ubuntu.com 慢/挂
RUN sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
      -e 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
      /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list 2>/dev/null || true

# ---------- 1. Basic tools ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim nano wget curl ca-certificates \
    build-essential cmake pkg-config \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    python3-vcstool python3-venv \
    mesa-utils \
    libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libglu1-mesa \
    mesa-vulkan-drivers \
    x11-apps \
    locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---------- 1.5 Realsense ----------
RUN apt-get update && apt-get install -y \
    ros-jazzy-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# ---------- 2. MoveIt 2 ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-setup-assistant \
    && rm -rf /var/lib/apt/lists/*

# ---------- 3. Gazebo Harmonic + ROS 2 bridge + gz_ros2_control ----------
# Jazzy's default Gazebo pair is Harmonic; ros-gz pulls it in automatically.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# ---------- 4. ros2_control + controllers ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-controller-manager \
    && rm -rf /var/lib/apt/lists/*

# ---------- 5. URDF / xacro / TF tools ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-xacro \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-tf2-tools \
    ros-jazzy-rqt-tf-tree \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rviz-visual-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------- 6. OctoMap / point-cloud / image transport ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-server \
    ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-image-pipeline \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-sensor-msgs-py \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Docker can otherwise reuse ROS package layers produced on different dates.
# The cache-buster supplied by build.sh forces one final repository refresh,
# then full-upgrade makes all ROS binaries ABI-consistent. Keep the packages
# involved in the previous controller_manager/diagnostic_updater mismatch
# explicit so a new image cannot omit them.
ARG ROS_APT_REFRESH=manual
RUN echo "ROS apt refresh: ${ROS_APT_REFRESH}" \
    && apt-get update \
    && apt-get full-upgrade -y \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-octomap-server \
        ros-jazzy-controller-manager \
        ros-jazzy-diagnostic-updater \
        ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# ---------- 7. Python algorithm dependencies ----------
# Split into small + large layers; large stuff has a local-wheel fallback.
# ★ 走清华镜像 + 短超时；官方源作为 fallback，避免 pypi 慢包被卡 10 分钟。
ARG PIP_OPTS="--no-cache-dir --default-timeout=120 --retries 5 --break-system-packages \
              --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
              --extra-index-url https://pypi.org/simple"

RUN pip3 install ${PIP_OPTS} \
    numpy \
    scipy \
    networkx \
    python-tsp \
    matplotlib \
    pyyaml \
    jsonschema \
    addict \
    configargparse \
    pyquaternion \
    tqdm \
    fastapi \
    uvicorn \
    python-multipart \
    mss \
    pillow

# ---------- 7.5 Semantic segmentation / YOLO 的 torch ----------
# ★ 固化为已验证通过的 torch 2.5.1+cu121(YOLO 走 CPU)。RTX 5080 是 Blackwell
#   (sm_120),cu121 的 kernel 只到 sm_90,GPU 上推理会崩;YOLO 节点用 device=auto
#   会自动探测并回退 CPU(~7fps,够用)。要 GPU 时改用 cu128/cu130 的 wheel
#   (torch 2.13,自带 sm_120 kernel),但需另行验证整套依赖。
#   注意:torch 2.5.1 这个构建需要 numpy>=2(下面 YOLO 段会强制把 numpy 升到 2.x)。
RUN pip3 install --no-cache-dir --default-timeout=180 --retries 5 \
        --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu121 \
        torch==2.5.1 torchvision==0.20.1 \
 && pip3 install ${PIP_OPTS} \
        transformers \
        accelerate \
        pillow \
        safetensors

# Target detection / bypass YOLO overlay 用 Ultralytics YOLO。版本对齐(均已验证):
#   - ultralytics --no-deps,避免它把上面的 CUDA torch/torchvision 换掉;再补它
#     运行时需要的轻量依赖(thop / py-cpuinfo)。
#   - torch 2.5.1 需要 numpy>=2,但基础层的 python-tsp/scipy 会把 numpy 拉到 <2。
#     这两个库主系统实际并未 import,所以在此强制把 numpy 升回 2.x(冲突警告无害),
#     否则 torch.from_numpy 会报 "Numpy is not available"。
#   - apt 的 cv2(4.6.0)是用 numpy1 编译的,在 numpy2 下 import 会崩,改用 numpy2
#     兼容的 opencv-python。
#   - filelock 需 >=3.15(提供 AsyncFileLock),distro 自带 3.13 太旧会让 import 失败。
RUN pip3 install ${PIP_OPTS} --no-deps \
        ultralytics==8.4.112 \
        ultralytics-thop \
        py-cpuinfo \
 && pip3 install ${PIP_OPTS} --ignore-installed "numpy>=2" "filelock>=3.15" \
 && pip3 install ${PIP_OPTS} opencv-python

ENV YOLO_CONFIG_DIR=/tmp

# 预取 YOLOv8n COCO 权重到 /root/ros2_ws/,避免真机无网时在线下载失败。
RUN mkdir -p /root/ros2_ws \
 && cd /root/ros2_ws \
 && python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# Open3D in its own layer. If a local wheel is provided under docker/wheels/,
# install from it; otherwise pull the wheel from PyPI. Use --no-deps so pip
# does not re-resolve Open3D's large optional visualization dependency chain
# or upgrade NumPy/SciPy/Setuptools behind ROS/colcon's back.
COPY wheels/ /tmp/wheels/
RUN if ls /tmp/wheels/open3d-*.whl > /dev/null 2>&1; then \
        echo "Installing open3d from local wheel" && \
        pip3 install ${PIP_OPTS} --no-deps /tmp/wheels/open3d-*.whl; \
    else \
        echo "Installing open3d from PyPI" && \
        pip3 install ${PIP_OPTS} --no-deps open3d; \
    fi && \
    rm -rf /tmp/wheels

# Keep numeric/build tooling in the versions expected by python-tsp and colcon.
RUN pip3 install ${PIP_OPTS} --ignore-installed --force-reinstall \
    "numpy>=1.26.3,<2.0" \
    "scipy>=1.11,<1.15" \
    "setuptools<80"

# ---------- 8. rosdep ----------
RUN rosdep update || true

# ---------- 9. Workspace ----------
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# ---------- 10. Source ROS automatically in every shell ----------
# Entrypoint covers `docker run`; .bashrc covers `docker exec`.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
 && echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc \
 && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc

# Clear the proxy env vars so the runtime container does not inherit them.
# (Build-time proxy was only needed for apt/pip during image construction.)
ENV HTTP_PROXY="" \
    HTTPS_PROXY="" \
    NO_PROXY="" \
    http_proxy="" \
    https_proxy="" \
    no_proxy=""

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
