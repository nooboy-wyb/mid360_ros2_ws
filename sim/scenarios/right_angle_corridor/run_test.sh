#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

scenario_dir="$ws_dir/sim/scenarios/right_angle_corridor"
out_root="$ws_dir/sim/output/right_angle_corridor"
run_dir="$out_root/run_$(date +%Y%m%d_%H%M%S)"
gui=false
lidar_shift_ms=0
while (($#)); do
  case "$1" in
    --gui) gui=true; shift ;;
    --lidar-shift-ms)
      (($# >= 2)) || { echo "--lidar-shift-ms requires a number" >&2; exit 2; }
      lidar_shift_ms="$2"; shift 2 ;;
    --help|-h)
      echo "Usage: $0 [--gui] [--lidar-shift-ms 0|5|10]"
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
mkdir -p "$run_dir/ros_logs" "$run_dir/gazebo_logs"
# Isolate each run from adapters left by earlier runs in the user's domain.
export ROS_DOMAIN_ID="${RIGHT_ANGLE_ROS_DOMAIN_ID:-$((20 + RANDOM % 210))}"
export ROS_LOG_DIR="$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$run_dir/gazebo_logs"
export GAZEBO_MODEL_PATH="$ws_dir/vendor_gazebo_models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ws_dir/install/ros2_livox_simulation/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MASTER_URI="http://127.0.0.1:$((17000 + RANDOM % 12000))"
export MID360_DROP_FIRST_SCAN=1
export MID360_LIDAR_STAMP_SHIFT_NS="$(python3 - "$lidar_shift_ms" <<'PY'
import sys
print(round(float(sys.argv[1]) * 1_000_000))
PY
)"

python3 "$scenario_dir/create_world.py" "$run_dir/world.sdf" >"$run_dir/world_generation.log"

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/world.sdf" >"$run_dir/gazebo.log" 2>&1 &
gazebo_pid=$!
setsid python3 sim/plugin_mid360_adapter.py >"$run_dir/adapter.log" 2>&1 & adapter_pid=$!
publisher_count=""
imu_publisher_count=""
for attempt in {1..10}; do
  lidar_info="$(timeout 4s ros2 topic info /livox/lidar --no-daemon --spin-time 1 --verbose 2>&1 || true)"
  imu_info="$(timeout 4s ros2 topic info /livox/imu --no-daemon --spin-time 1 --verbose 2>&1 || true)"
  publisher_count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$lidar_info")"
  imu_publisher_count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$imu_info")"
  if [[ "$publisher_count" == 1 && "$imu_publisher_count" == 1 ]]; then break; fi
  sleep 1
done
if [[ "$publisher_count" != 1 || "$imu_publisher_count" != 1 ]]; then
  printf '%s\n' '--- /livox/lidar graph ---' "$lidar_info" '--- /livox/imu graph ---' "$imu_info" >"$run_dir/publisher_check.log"
  echo "Could not confirm one publisher per sensor topic (lidar=${publisher_count:-none}, imu=${imu_publisher_count:-none}); startup stopped. See $run_dir/publisher_check.log and adapter.log." >&2
  kill -TERM -- "-$adapter_pid" 2>/dev/null || true
  kill -TERM "$adapter_pid" 2>/dev/null || true
  kill -TERM "$gazebo_pid" 2>/dev/null || true
  wait "$adapter_pid" "$gazebo_pid" 2>/dev/null || true
  exit 1
fi
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$ws_dir/sim/fastlio_sim.yaml" \
  -p "map_file_path:=$run_dir/fastlio_map.pcd" \
  >"$run_dir/fastlio.log" 2>&1 & fastlio_pid=$!
setsid ros2 bag record -o "$run_dir/trajectory_bag" \
  /clock /cmd_vel /odom /joint_states \
  /ground_truth/model_states /ground_truth/link_states \
  /mid360_plugin/points /mid360_sim/imu /livox/lidar /livox/imu \
  /Odometry \
  >"$run_dir/rosbag.log" 2>&1 & bag_pid=$!
gui_pid=""
if "$gui"; then
  gzclient --verbose >"$run_dir/gzclient.log" 2>&1 & gui_pid=$!
fi

cleanup() {
  trap - EXIT INT TERM
  for pid in "$bag_pid" "$fastlio_pid" "$adapter_pid"; do
    [[ -n "$pid" ]] && kill -INT -- "-$pid" 2>/dev/null || true
  done
  [[ -n "$gui_pid" ]] && kill -TERM "$gui_pid" 2>/dev/null || true
  sleep 2
  for pid in "$bag_pid" "$fastlio_pid" "$adapter_pid"; do
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
  done
  kill -TERM "$gazebo_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$gazebo_pid" 2>/dev/null || true
  for pid in "$bag_pid" "$fastlio_pid" "$adapter_pid" "$gui_pid" "$gazebo_pid"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

sleep 5
if ! timeout 10s ros2 topic echo /ground_truth/model_states --once --field name >"$run_dir/ground_truth_ready.log"; then
  echo "Gazebo ground-truth topic did not become ready; inspect $run_dir/gazebo.log" >&2
  exit 1
fi
python3 "$scenario_dir/route_controller.py" "$run_dir" >"$run_dir/route_controller.log" 2>&1
if ! rg -q '"status": "complete"' "$run_dir/route_status.json"; then
  cat "$run_dir/route_status.json" >&2
  exit 1
fi
# Close the sensor recording and freeze simulation time as soon as the route
# ends. This prevents idle scans and growing map clouds from bloating the bag.
kill -INT -- "-$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
kill -TERM "$gazebo_pid" 2>/dev/null || true
wait "$gazebo_pid" 2>/dev/null || true
# FAST-LIO stays alive to serialize its final map after the simulation stops.
timeout 30s ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/map_save.log" 2>&1 || true
kill -INT -- "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
sleep 2
kill -TERM -- "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
wait "$fastlio_pid" "$adapter_pid" 2>/dev/null || true
if [[ -n "$gui_pid" ]]; then kill -TERM "$gui_pid" 2>/dev/null || true; wait "$gui_pid" 2>/dev/null || true; fi
trap - EXIT INT TERM

python3 "$scenario_dir/evaluate_run.py" "$run_dir"
echo "Scenario complete: $run_dir"
echo "Metrics: $run_dir/metrics.json"
echo "Plot: $run_dir/localization_plot.png"
echo "FAST-LIO2 map: $run_dir/fastlio_map.pcd"
