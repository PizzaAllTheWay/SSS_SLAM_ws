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
  sss_data \
  --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Source workspace if build succeeded
source install/setup.bash
echo "[INFO] Workspace ready"

############################################################
# ROS TOPICS TO RECORD
############################################################
ROS_TOPICS_TO_RECORD=(
  /chatter
)

############################################################
# MONO TMUX EXECUTOR
############################################################
mono_execute() {
  # Custom TMUX terminal name passe down
  local TMUX_NAME="$1"
  shift
  # Custom command passed down
  local CMD="$*"

  # Configuring ROS environment
  local CMD_ROS="source ${ROS_DISTRO_PATH}
                 export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
                 export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"

  # Configuring workspace environment
  local CMD_WS="source '$WS_DIR/install/setup.bash'"

  # Combine all
  local CMD_ALL="${CMD_ROS}
                 ${CMD_WS}
                 ${CMD}
                 exec bash"

  # Execute
  tmux new-session -d -s "${TMUX_NAME}" "$CMD_ALL"
}

############################################################
# GLOBAL KILL
############################################################
cleanup() {
  echo "[CLEANUP] Stopping rosbag recording"

  # Gracefully stop rosbag so metadata.yaml is written
  bash -c "tmux send-keys -t data_recording C-c 2>/dev/null || true" || true
  sleep 2

  echo "[CLEANUP] Killing all ROS 2 + tmux"

  CMD_CLEAN_ALL="pkill tmux || true
                 pkill -f /opt/ros || true"

  # Kill
  bash -c "$CMD_CLEAN_ALL" || true
}

############################################################
# START FUNCTIONS
############################################################
start_sss_data() {
  mono_execute sss_data "ros2 launch sss_data sss_data.launch.py"
}

start_data_recording() {
  local ROS_DATA_BAG_DIR="$WS_DIR/data/sss_data_$(date +%Y%m%d_%H%M%S)"

  mono_execute data_recording "ros2 bag record --disable-keyboard-controls --topics ${ROS_TOPICS_TO_RECORD[*]} -o ${ROS_DATA_BAG_DIR}"
}

############################################################
# LAUNCH SEQUENCE
############################################################
echo "[INFO] Starting data recording..."
start_data_recording

echo "[INFO] Starting SSS node..."
start_sss_data

# ? NOTE: This will only run when you press CTRL+C or an error occurs, otherwise it will not run
trap cleanup EXIT SIGINT SIGTERM

echo "[INFO] All running. Ctrl+C to stop."
while true
do
    sleep 1
done

