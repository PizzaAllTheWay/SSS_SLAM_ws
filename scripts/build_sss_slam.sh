#!/usr/bin/env bash
set -e

############################################################
# WORKSPACE SETUP
############################################################
# Go to script dir, then workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

ROS_DISTRO="jazzy"
ROS_DISTRO_PATH="/opt/ros/${ROS_DISTRO}/setup.bash"

# Clean previous build artifacts (avoids stale CMake trash builds)
rm -rf build install log

# Source ROS 2
source "${ROS_DISTRO_PATH}"

# Build: suppress warnings, fail only on errors
# ? Note: For data replay we don't need to build all the packages  
echo "[INFO] Building workspace..."
colcon build \
  --packages-select \
  state_estimator \
  --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Source workspace if build succeeded
source install/setup.bash
echo "[INFO] Workspace ready"