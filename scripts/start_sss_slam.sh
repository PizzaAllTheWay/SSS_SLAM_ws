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
# CHECK FOR FLAGS
############################################################
LOG=false

for arg in "$@"
do
  case $arg in
    --LOG=*)
      LOG="${arg#*=}"
      ;;
  esac
done

echo "[INFO] LOG = ${LOG}"

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
  echo "[CLEANUP] Stopping Side Scan Sonar SLAM pipeline"
  echo "[CLEANUP] Killing all ROS 2 + tmux"

  CMD_CLEAN_ALL="pkill tmux || true
                 pkill -f /opt/ros || true"

  # Kill
  bash -c "$CMD_CLEAN_ALL" || true
}

############################################################
# START FUNCTIONS
############################################################
start_state_estimator() {
  mono_execute state_estimator "ros2 launch state_estimator state_estimator.launch.py log:=${LOG}"
}

############################################################
# LAUNCH SEQUENCE
############################################################
echo "[INFO] Starting state estimator..."
start_state_estimator

# ? NOTE: This will only run when you press CTRL+C or an error occurs, otherwise it will not run
trap cleanup EXIT SIGINT SIGTERM

echo "[INFO] All running. Ctrl+C to stop."
while true
do
    sleep 1
done

