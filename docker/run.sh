#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT_DIR="$(cd "$PROJECT_DIR/.." && pwd)"


IMAGE="jaka_exploration:jazzy"
CONTAINER="jaka_exploration_ros2"

# Allow X11 forwarding from the container.
xhost +local:root > /dev/null 2>&1 || true

XAUTH_HOST="${XAUTHORITY:-$HOME/.Xauthority}"

# Common Docker arguments.
DOCKER_ARGS=(
    --name "${CONTAINER}"
    --gpus all
    --net=host
    --ipc=host
    --privileged
    -e "DISPLAY=${DISPLAY:-:0}"
    -e QT_X11_NO_MITSHM=1
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${PROJECT_DIR}/ros2_ws":/root/ros2_ws:rw
    -v "${ROOT_DIR}":/root/host_project:rw
)

# WSLg renders OpenGL through Mesa's D3D12 driver and the Windows GPU.
# Docker Desktop injects /dev/dxg through --gpus all; these mounts expose
# the WSL graphics libraries and the Wayland/X11/audio sockets.
if [[ -d /usr/lib/wsl/lib && -d /mnt/wslg ]]; then
    echo "[run.sh] WSLg detected; enabling D3D12 GPU rendering."
    DOCKER_ARGS+=(
        -e GALLIUM_DRIVER=d3d12
        -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
        -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}"
        -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
        -e PULSE_SERVER=unix:/mnt/wslg/PulseServer
        -e LD_LIBRARY_PATH=/usr/lib/wsl/lib
        -v /usr/lib/wsl:/usr/lib/wsl:ro
        -v /mnt/wslg:/mnt/wslg:rw
    )
fi

# RealSense access is optional for the Gazebo-only demo.
if [[ -e /dev/bus/usb ]]; then
    DOCKER_ARGS+=(--device=/dev/bus/usb:/dev/bus/usb)
fi

# JAKA vendor SDK — required for jaka_state_publisher / real-arm launches.
# The folder name contains spaces upstream ("JAKA SDK v2.2.2"), so we mount it
# under a stable spaceless path at /opt/jaka_sdk.  Override the host path with
# the JAKA_SDK_HOST env var if the SDK lives elsewhere.
JAKA_SDK_HOST_CANDIDATES=(
    "${JAKA_SDK_HOST:-}"
    "${ROOT_DIR}/../JAKA-A5+UR5e/JAKA_A5_Docs/JAKA SDK v2.2.2"
    "$HOME/project/450_g12/JAKA-A5+UR5e/JAKA_A5_Docs/JAKA SDK v2.2.2"
)
for candidate in "${JAKA_SDK_HOST_CANDIDATES[@]}"; do
    if [[ -n "$candidate" && -f "$candidate/04 Linux/python3/x86_64-linux-gnu/libjakaAPI.so" ]]; then
        echo "[run.sh] Mounting JAKA SDK from: $candidate"
        DOCKER_ARGS+=(-v "$candidate:/opt/jaka_sdk:ro")
        DOCKER_ARGS+=(-e "JAKA_SDK_ROOT=/opt/jaka_sdk")
        break
    fi
done

# Xauthority may not exist under WSLg, where the socket is already trusted.
if [[ -f "$XAUTH_HOST" ]]; then
    DOCKER_ARGS+=(
        -e "XAUTHORITY=${XAUTH_HOST}"
        -v "${XAUTH_HOST}:${XAUTH_HOST}:rw"
    )
fi

# If a container with this name is already running, attach to it.
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "[run.sh] Container '${CONTAINER}' is already running. Attaching ..."
    docker exec -it "${CONTAINER}" bash
    exit 0
fi

# If exists but stopped, remove it first.
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    docker rm -f "${CONTAINER}" > /dev/null
fi

echo "[run.sh] Starting container '${CONTAINER}' ..."
docker run -it --rm "${DOCKER_ARGS[@]}" "${IMAGE}" bash
