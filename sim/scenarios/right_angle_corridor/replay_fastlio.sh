#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if (($# != 1)); then
  echo "Usage: $0 <manual-run-directory-or-trajectory_bag>" >&2
  exit 2
fi
source_path="$(realpath "$1")"
if [[ -d "$source_path/trajectory_bag" ]]; then
  source_bag="$source_path/trajectory_bag"
elif [[ -f "$source_path/metadata.yaml" ]]; then
  source_bag="$source_path"
  source_path="$(dirname "$source_path")"
else
  echo "No trajectory_bag/metadata.yaml found under: $1" >&2
  exit 2
fi

scenario_dir="$ws_dir/sim/scenarios/right_angle_corridor"
replay_dir="$source_path/replay_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$replay_dir/ros_logs"
export ROS_DOMAIN_ID="${RIGHT_ANGLE_ROS_DOMAIN_ID:-$((20 + RANDOM % 210))}"
export ROS_LOG_DIR="$replay_dir/ros_logs"
export MID360_DROP_FIRST_SCAN=1
echo "Starting same-bag FAST-LIO2 replay in ROS_DOMAIN_ID=$ROS_DOMAIN_ID; output: $replay_dir"

setsid python3 sim/plugin_mid360_adapter.py >"$replay_dir/adapter.log" 2>&1 & adapter_pid=$!
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$ws_dir/sim/fastlio_sim.yaml" \
  -p "map_file_path:=$replay_dir/fastlio_map.pcd" \
  >"$replay_dir/fastlio.log" 2>&1 & fastlio_pid=$!
setsid ros2 bag record -o "$replay_dir/trajectory_bag" \
  /clock /ground_truth/model_states /ground_truth/link_states /Odometry \
  >"$replay_dir/rosbag.log" 2>&1 & bag_pid=$!

cleanup() {
  trap - EXIT INT TERM
  kill -INT -- "-$bag_pid" "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$bag_pid" "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
  for pid in "$bag_pid" "$fastlio_pid" "$adapter_pid"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

publisher_count=""
imu_publisher_count=""
for attempt in {1..10}; do
  lidar_info="$(timeout 4s ros2 topic info /livox/lidar --no-daemon --spin-time 1 --verbose 2>&1 || true)"
  imu_info="$(timeout 4s ros2 topic info /livox/imu --no-daemon --spin-time 1 --verbose 2>&1 || true)"
  publisher_count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$lidar_info")"
  imu_publisher_count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$imu_info")"
  if [[ "$lidar_info$imu_info" == *"Operation not permitted"* ||
        "$lidar_info$imu_info" == *"Permission denied"* ]]; then
    printf '%s\n' '--- /livox/lidar graph ---' "$lidar_info" '--- /livox/imu graph ---' "$imu_info" >"$replay_dir/publisher_check.log"
    echo "ROS middleware cannot open its DDS network socket in this execution environment. Replay stopped; run this script from the host ROS terminal. Details: $replay_dir/publisher_check.log" >&2
    exit 1
  fi
  if [[ "$publisher_count" == 1 && "$imu_publisher_count" == 1 ]]; then break; fi
  sleep 1
done
if [[ "$publisher_count" != 1 || "$imu_publisher_count" != 1 ]]; then
  printf '%s\n' '--- /livox/lidar graph ---' "$lidar_info" '--- /livox/imu graph ---' "$imu_info" >"$replay_dir/publisher_check.log"
  echo "Could not confirm one publisher per sensor topic (lidar=${publisher_count:-none}, imu=${imu_publisher_count:-none}); replay stopped. See $replay_dir/publisher_check.log." >&2
  exit 1
fi
echo "Replaying recorded sensor inputs in ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
echo "Recorded /Odometry is excluded; FAST-LIO2 will publish a fresh estimate."
ros2 bag play "$source_bag" --rate 1.0 --disable-keyboard-controls --topics \
  /clock /mid360_plugin/points /mid360_sim/imu \
  /ground_truth/model_states /ground_truth/link_states \
  >"$replay_dir/playback.log" 2>&1

timeout 30s ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$replay_dir/map_save.log" 2>&1 || true
cleanup
python3 "$scenario_dir/evaluate_run.py" "$replay_dir"
echo "Replay evaluation: $replay_dir/metrics.json"
echo "Replay trajectory bag: $replay_dir/trajectory_bag"
