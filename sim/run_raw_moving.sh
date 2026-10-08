#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export GAZEBO_MODEL_PATH="$ws_dir/vendor_gazebo_models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ws_dir/install/ros2_livox_simulation/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MASTER_URI="http://127.0.0.1:$((15000 + RANDOM % 20000))"

scene=lab
gui=true
while (($#)); do
  case "$1" in
    --scene)
      (($# >= 2)) || { echo "--scene requires house, lab, or box" >&2; exit 2; }
      scene="$2"; shift 2 ;;
    --scene=*) scene="${1#*=}"; shift ;;
    --headless) gui=false; shift ;;
    --help|-h)
      echo "Usage: $0 [--scene house|lab|box] [--headless]"
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$scene" in house|lab|box) ;; *) echo "Unknown scene: $scene" >&2; exit 2 ;; esac

mkdir -p sim/output
run_dir="$(mktemp -d sim/output/raw_move_XXXXXXXX)"
export ROS_LOG_DIR="$ws_dir/$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$ws_dir/$run_dir/gazebo_logs"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"
python3 sim/create_classic_world.py "$run_dir/raw_moving.sdf" "$scene"
max_range=12
[[ "$scene" == box ]] && max_range=30

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/raw_moving.sdf" >"$run_dir/gazebo.log" 2>&1 &
sim_pid=$!
python3 sim/raw_moving_capture.py "$run_dir" --max-range "$max_range" >"$run_dir/capture.log" 2>&1 &
capture_pid=$!
gui_pid=""
rviz_pid=""
if "$gui"; then
  gzclient --verbose >"$run_dir/gzclient.log" 2>&1 &
  gui_pid=$!
  rviz2 -d sim/raw_lidar_moving.rviz >"$run_dir/rviz.log" 2>&1 &
  rviz_pid=$!
fi
(
  sleep 3
  timeout 8s ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.12}, angular: {z: 0.15}}' || test "$?" -eq 124
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}'
) >"$run_dir/drive.log" 2>&1 &
drive_pid=$!

cleanup() {
  for pid in "$rviz_pid" "$gui_pid" "$drive_pid"; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done
  if [[ -n "$capture_pid" ]]; then
    kill -INT "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
  fi
  kill -TERM "$sim_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$sim_pid" 2>/dev/null || true
  for pid in "$rviz_pid" "$gui_pid" "$drive_pid" "$sim_pid"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
  echo "Saved raw odometry-aligned scans (no FAST-LIO2): $ws_dir/$run_dir/raw_accum_valid_odom.pcd"
  echo "View later with: bash sim/view_raw_map.sh $ws_dir/$run_dir/raw_accum_valid_odom.pcd"
}
trap cleanup EXIT

if "$gui"; then
  echo "Scene: $scene. The robot drives for 8 seconds; close RViz to finish and save the raw point cloud."
  wait "$rviz_pid" || true
else
  echo "Scene: $scene. Driving for 8 seconds and capturing raw scans without a GUI."
  wait "$drive_pid" || true
  sleep 3
  kill -INT "$capture_pid" 2>/dev/null || true
  wait "$capture_pid" 2>/dev/null || true
  capture_pid=""
fi
