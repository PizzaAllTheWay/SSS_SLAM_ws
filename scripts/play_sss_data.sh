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

# Source ROS 2
source "${ROS_DISTRO_PATH}"

# Source workspace if build succeeded
source install/setup.bash
echo "[INFO] Workspace ready"

############################################################
# DATA REPLAY
############################################################
DATA_DIR="$WS_DIR/data"

# Only consider sss_data_* directories
mapfile -t DATASETS < <(ls -td "$DATA_DIR"/sss_data_* 2>/dev/null || true)

if [ ${#DATASETS[@]} -eq 0 ]; then
  echo "[ERROR] No sss_data_* datasets found in $DATA_DIR"
  exit 1
fi

# If dataset specified, use it
if [ $# -eq 1 ]; then
  BAG_PATH="$DATA_DIR/$1"
  if [[ ! -d "$BAG_PATH" || "$BAG_PATH" != "$DATA_DIR"/sss_data_* ]]; then
    echo "[ERROR] Specified dataset not found or invalid: $BAG_PATH"
    exit 1
  fi
else
  # Pick latest matching dataset
  BAG_PATH="${DATASETS[0]}"
fi

echo "[INFO] Playing back dataset: $(basename "$BAG_PATH")"

ros2 bag play "$BAG_PATH"