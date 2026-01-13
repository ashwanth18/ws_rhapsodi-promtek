#!/usr/bin/env bash
set -eEo pipefail

# Prefer the image's venv for runtime Python deps (roslibpy, backend deps, etc.)
if [ -d "/opt/venv" ]; then
  export VIRTUAL_ENV="/opt/venv"
  export PATH="/opt/venv/bin:${PATH}"
fi

# ROS setup files sometimes read optional env vars (e.g., AMENT_TRACE_SETUP_FILES).
# Don't fail the container just because those aren't set.
set +u

# Source ROS 2 Jazzy underlay
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/jazzy/setup.bash"
fi

# Source workspace overlay (built into the image at /ws)
if [ -f "/ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/ws/install/setup.bash"
fi

set -u

exec "$@"


