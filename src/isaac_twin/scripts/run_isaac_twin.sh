#!/usr/bin/env bash
# Isaac Sim side of the twin: Niryo, D455 and powder on the twin ROS domain.
#   src/isaac_twin/scripts/run_isaac_twin.sh [--headless] [--layout-id ID] [--fill-depth 0.04]
# Then, in another terminal: ros2 launch isaac_twin scooping_isaac.launch.py
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS="$(cd "${HERE}/../../.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64}"
CACHE="${XDG_CACHE_HOME:-${HOME}/.cache}/isaac_twin"
URDF="${CACHE}/niryo_ned3pro.urdf"

if [[ ! -x "${ISAAC_SIM_ROOT}/python.sh" ]]; then
  echo "Isaac Sim not found at ${ISAAC_SIM_ROOT} (set ISAAC_SIM_ROOT)" >&2
  exit 1
fi

# xacro needs the native ROS install; Isaac's python does not have it.
mkdir -p "${CACHE}"
(
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${WS}/install/setup.bash"
  xacro "$(ros2 pkg prefix niryo_robot_description)/share/niryo_robot_description/urdf/ned3pro/niryo_ned3pro.urdf.xacro" \
    > "${URDF}"
)
# The importer names prims after mesh files, so names like
# niryo_scoop_v4-ros.STL (invalid USD path) get a sanitised symlink.
python3 - "${URDF}" "${CACHE}/meshes" <<'PY'
import os, re, sys
urdf, mesh_dir = sys.argv[1], sys.argv[2]
os.makedirs(mesh_dir, exist_ok=True)
text = open(urdf, encoding="utf-8").read()

def fix(match):
    path = match.group(2).removeprefix("file://")
    stem, ext = os.path.splitext(os.path.basename(path))
    clean = re.sub(r"[^A-Za-z0-9_]", "_", stem)
    if clean != stem:
        link = os.path.join(mesh_dir, clean + ext)
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(path, link)
        path = link
    return f'{match.group(1)}"{path}"'

text = re.sub(r'(filename=)"([^"]+)"', fix, text)
open(urdf, "w", encoding="utf-8").write(text)
PY

source "${HERE}/twin_env.sh"
echo "isaac_twin: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} discovery=${ROS_AUTOMATIC_DISCOVERY_RANGE} urdf=${URDF}"

# Isaac's bundled Jazzy (Python 3.11) instead of the system ROS (Python 3.12).
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  ROS_DISTRO=jazzy \
  LD_LIBRARY_PATH="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/jazzy/lib" \
  ISAAC_TWIN_WS="${WS}" \
  "${ISAAC_SIM_ROOT}/python.sh" "${WS}/src/isaac_twin/isaac_twin/scene/build_cell.py" --urdf "${URDF}" "$@"
