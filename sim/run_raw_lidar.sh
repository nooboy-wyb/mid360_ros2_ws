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
gui=false
accumulate=false
scene="${MID360_SCENE:-house}"
while (($#)); do
  case "$1" in
    --gui) gui=true ;;
    --accumulate) accumulate=true ;;
    --scene=*) scene="${1#*=}" ;;
    --scene)
      if (($# < 2)); then echo "--scene requires house, lab, or box" >&2; exit 2; fi
      scene="$2"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--gui] [--accumulate] [--scene house|lab|box]"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
case "$scene" in
  house|lab|box) ;;
  *) echo "Unknown scene '$scene'; choose house, lab, or box" >&2; exit 2 ;;
esac
export MID360_SCENE="$scene"
mkdir -p sim/output
run_dir="$(mktemp -d sim/output/raw_XXXXXXXX)"
echo "Raw-only run: $ws_dir/$run_dir"
export ROS_LOG_DIR="$ws_dir/$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$ws_dir/$run_dir/gazebo_logs"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"
echo "Scene: $scene"
python3 sim/create_classic_world.py "$run_dir/house_raw.sdf" "$scene"
save_frames=3
display_max_range=12
rviz_config=sim/raw_lidar.rviz
if [[ "$scene" == "box" ]]; then
  display_max_range=30
fi
if "$accumulate"; then
  save_frames=100
  rviz_config=sim/raw_lidar_accum.rviz
fi

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/house_raw.sdf" >"$run_dir/gazebo.log" 2>&1 &
sim_pid=$!
python3 sim/raw_lidar_capture.py "$run_dir" --save-frames "$save_frames" --max-range "$display_max_range" >"$run_dir/capture.log" 2>&1 &
capture_pid=$!
gui_pid=""
rviz_pid=""
cleanup() {
  kill -TERM "$rviz_pid" "$gui_pid" "$capture_pid" "$sim_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$rviz_pid" "$gui_pid" "$capture_pid" "$sim_pid" 2>/dev/null || true
  wait "$rviz_pid" "$gui_pid" "$capture_pid" "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT

if "$gui"; then
  gzclient --verbose >"$run_dir/gzclient.log" 2>&1 &
  gui_pid=$!
  rviz2 -d "$rviz_config" >"$run_dir/rviz.log" 2>&1 &
  rviz_pid=$!
  echo "RViz shows /raw_livox/valid_points; close RViz to end capture."
  wait "$rviz_pid" || true
else
  if "$accumulate"; then
    timeout 180s bash -c 'until [[ -s "$1" ]]; do sleep 0.5; done' _ "$run_dir/raw_100.json"
  else
    sleep 12
  fi
  timeout 10s ros2 topic echo /mid360_plugin/points --once --field width >"$run_dir/plugin_width.txt"
  timeout 10s ros2 topic echo /raw_livox/valid_points --once --field width >"$run_dir/valid_width.txt"
  test -s "$run_dir/raw_full_001.pcd"
  test -s "$run_dir/raw_valid_001.pcd"
  python3 - "$run_dir/raw_001.json" <<'PY'
import json, sys
data = json.load(open(sys.argv[1]))
assert data['full_points'] > 0
print(f"Raw plugin frame: {data['full_points']} points; valid returns: {data['valid_returns']}")
PY
  if "$accumulate"; then
    python3 sim/compare_raw_coverage.py "$run_dir"
  fi
fi
echo "Saved raw scan data in: $ws_dir/$run_dir"
