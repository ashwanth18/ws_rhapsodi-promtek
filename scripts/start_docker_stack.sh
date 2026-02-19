#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

DO_BUILD=1
if [[ "${1:-}" == "--no-build" ]]; then
  DO_BUILD=0
fi

if [[ "$DO_BUILD" -eq 1 ]]; then
  echo "Building images..."
  docker compose build --no-cache runtime
  docker compose build --no-cache dashboard
else
  echo "Skipping builds (--no-build)."
fi

echo "Starting rosbridge..."
docker compose up -d rosbridge

echo "Starting sensor launchers..."
docker compose up -d realsense_launcher scale_launcher
# scale_launcher micro_ros_launcher

echo "Starting dashboard..."
VITE_ROSBRIDGE_URL=ws://localhost:9090 \
VITE_REALSENSE_CAMERA_INFO_TOPIC=/camera/camera/color/camera_info \
DASHBOARD_PORT=8080 \
docker compose up -d --build dashboard

echo "Done. Dashboard: http://localhost:8080"

