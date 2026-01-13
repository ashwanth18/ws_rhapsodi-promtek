#
# ROS 2 Jazzy workspace container (multi-arch: works on Raspberry Pi 5 / arm64)
#
# Build:
#   docker build -t rhapsodi-promtek:jazzy .
#
# Run:
#   docker run --rm -it --net=host --ipc=host rhapsodi-promtek:jazzy bash
#

FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1

# Tools needed to resolve deps + build a typical ROS2 colcon workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      ninja-build \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-venv \
      python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep setup (idempotent)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

WORKDIR /ws

# Copy only the source tree into the image (keeps builds smaller)
COPY src/ /ws/src/

# If this repo uses vcstool, fetch external dependencies into /ws/src.
# This keeps the GitHub repo clean while still making the Docker build self-contained.
RUN if [ -f /ws/src/ros2.repos ] && ! grep -qE '^repositories:\\s*\\{\\}\\s*$' /ws/src/ros2.repos; then \
      vcs import /ws/src < /ws/src/ros2.repos; \
    fi

# Non-ROS Python deps:
# Install into a venv so we don't fight system/apt Python packages (PEP 668 / RECORD issues).
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/python -m pip install --upgrade pip setuptools wheel
RUN if [ -f /ws/src/requirements.txt ]; then /opt/venv/bin/pip install -r /ws/src/requirements.txt; fi \
    && if [ -f /ws/src/backend/requirements.txt ]; then /opt/venv/bin/pip install -r /ws/src/backend/requirements.txt; fi

# Install ROS/system deps declared in package.xml files
RUN apt-get update \
    && rosdep install \
        --from-paths /ws/src \
        --ignore-src \
        -r -y \
        --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

# Build the whole workspace into /ws/install (overlay)
#
# Colcon build can be memory-hungry on constrained devices (and Docker Desktop defaults).
# Tune with build args if you have more/less RAM available.
ARG COLCON_PARALLEL_WORKERS=1
ARG CMAKE_BUILD_TYPE=Release
ARG CMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG"

ENV MAKEFLAGS="-j${COLCON_PARALLEL_WORKERS}" \
    CMAKE_BUILD_PARALLEL_LEVEL="${COLCON_PARALLEL_WORKERS}"

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --merge-install \
        --executor sequential \
        --parallel-workers ${COLCON_PARALLEL_WORKERS} \
        --cmake-args \
          -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
          "-DCMAKE_CXX_FLAGS_RELEASE=${CMAKE_CXX_FLAGS_RELEASE}"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]


