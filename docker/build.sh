#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Ensure wheels/ exists so the Dockerfile COPY does not fail.
mkdir -p wheels
touch wheels/.gitkeep

# Detect host proxy. If a local clash/v2ray-style proxy is running on
# 127.0.0.1:7897, pass it through to the build container so that apt and
# pip fetches go through the proxy (much faster than direct in some regions).
PROXY_URL="http://127.0.0.1:7897"
EXTRA_ARGS=()
EXTRA_ARGS+=(
    --build-arg "ROS_APT_REFRESH=$(date -u +%Y%m%dT%H%M%SZ)"
)
if curl -s --max-time 3 --proxy "$PROXY_URL" https://www.google.com > /dev/null 2>&1; then
    echo "[build.sh] Host proxy detected at $PROXY_URL — passing through to build."
    EXTRA_ARGS+=(
        --network=host
        --build-arg "HTTP_PROXY=$PROXY_URL"
        --build-arg "HTTPS_PROXY=$PROXY_URL"
        --build-arg "NO_PROXY=localhost,127.0.0.1"
    )
else
    echo "[build.sh] No host proxy detected at $PROXY_URL — using direct network."
fi

echo "[build.sh] Building jaka_exploration:jazzy ..."
docker build "${EXTRA_ARGS[@]}" -t jaka_exploration:jazzy .
echo "[build.sh] Done. Image: jaka_exploration:jazzy"
