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

echo "[INFO] Workspace ready"

############################################################
# ROS DATATYPE DEPENDENCIES
############################################################
echo "[INFO] Installing ROS 2 Data type dependencies..."

echo "[INFO] Making sure ROS 2 Jazzy is up to date..."
sudo apt update

echo "[INFO] Installing marine acoustics ROS 2 messages from https://docs.ros.org/en/jazzy/p/marine_acoustic_msgs/"
sudo apt install ros-jazzy-marine-acoustic-msgs

############################################################
# STATE ESTIMATOR DEPENDENCIES
############################################################
echo "[INFO] Installing State Estimator dependencies..."

echo "[INFO] Installing UKF-M python lib"
cd "$WS_DIR/libs/ukfm/python"
python3 -m pip install -e . --break-system-packages
cd "$WS_DIR"

