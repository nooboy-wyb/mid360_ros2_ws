#!/usr/bin/env bash
set -eo pipefail
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export GAZEBO_MODEL_PATH="$ws_dir/vendor_gazebo_models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ws_dir/install/ros2_livox_simulation/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export ROS_LOG_DIR="$ws_dir/sim/output/gazebo_gui_ros_logs"
export GAZEBO_LOG_PATH="$ws_dir/sim/output/gazebo_gui_logs"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"
world_path="$ws_dir/sim/output/mesh_lab_classic_gui.sdf"
python3 sim/create_classic_world.py "$world_path" "${MID360_SCENE:-house}"
gzserver --verbose -s libgazebo_ros_init.so "$world_path" &
server_pid=$!
gzclient --verbose &
client_pid=$!
cleanup() {
  kill -TERM "$client_pid" "$server_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$client_pid" "$server_pid" 2>/dev/null || true
  wait "$client_pid" "$server_pid" 2>/dev/null || true
}
trap cleanup EXIT
wait "$client_pid"
