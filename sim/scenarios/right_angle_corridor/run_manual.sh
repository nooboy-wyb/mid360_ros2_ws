#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# Isolate each run from adapters left by earlier runs in the user's domain.
# Use a scenario-specific override so an exported, fixed ROS_DOMAIN_ID does not
# silently put repeated manual runs back into the same DDS graph.
export ROS_DOMAIN_ID="${RIGHT_ANGLE_ROS_DOMAIN_ID:-$((20 + RANDOM % 210))}"
export GAZEBO_MODEL_PATH="$ws_dir/vendor_gazebo_models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ws_dir/install/ros2_livox_simulation/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MASTER_URI="http://127.0.0.1:$((17000 + RANDOM % 12000))"
export MID360_DROP_FIRST_SCAN=1

run_dir="$(mktemp -d "$ws_dir/sim/output/right_angle_corridor/manual_XXXXXXXX")"
mkdir -p "$run_dir/ros_logs" "$run_dir/gazebo_logs"
export ROS_LOG_DIR="$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$run_dir/gazebo_logs"
echo "Starting corridor demo in ROS_DOMAIN_ID=$ROS_DOMAIN_ID; logs: $run_dir"
python3 sim/scenarios/right_angle_corridor/create_world.py "$run_dir/world.sdf" >"$run_dir/world_generation.log"

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/world.sdf" >"$run_dir/gazebo.log" 2>&1 & gazebo_pid=$!
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
echo "Verified exactly one publisher each on /livox/lidar and /livox/imu. Starting FAST-LIO2, RViz, and recording."
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$ws_dir/sim/fastlio_sim.yaml" \
  -p "map_file_path:=$run_dir/fastlio_map.pcd" >"$run_dir/fastlio.log" 2>&1 & fastlio_pid=$!
setsid ros2 bag record -o "$run_dir/trajectory_bag" \
  /clock /cmd_vel /odom /joint_states \
  /ground_truth/model_states /ground_truth/link_states \
  /mid360_plugin/points /mid360_sim/imu /livox/lidar /livox/imu /Odometry \
  >"$run_dir/rosbag.log" 2>&1 & bag_pid=$!
gzclient --verbose >"$run_dir/gzclient.log" 2>&1 & gui_pid=$!
rviz2 -d sim/scenarios/right_angle_corridor/manual.rviz >"$run_dir/rviz.log" 2>&1 & rviz_pid=$!

cleanup() {
  trap - EXIT INT TERM
  timeout 2s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  kill -TERM "$rviz_pid" "$gui_pid" 2>/dev/null || true
  kill -INT -- "-$bag_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  kill -TERM "$gazebo_pid" 2>/dev/null || true
  wait "$gazebo_pid" 2>/dev/null || true
  timeout 30s ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/map_save.log" 2>&1 || true
  kill -INT -- "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
  for pid in "$rviz_pid" "$gui_pid" "$fastlio_pid" "$adapter_pid" "$bag_pid" "$gazebo_pid"; do wait "$pid" 2>/dev/null || true; done
  if [[ -f "$run_dir/trajectory_bag/metadata.yaml" ]]; then
    python3 "$ws_dir/sim/scenarios/right_angle_corridor/evaluate_run.py" "$run_dir" >"$run_dir/evaluation_stdout.json" 2>"$run_dir/evaluation_stderr.log" || true
  fi
  echo
  echo "Manual run saved: $run_dir"
  echo "Bag: $run_dir/trajectory_bag (includes /clock; replay without --clock)"
  echo "FAST-LIO2 map: $run_dir/fastlio_map.pcd"
  echo "Metrics/plot, when enough data was recorded: $run_dir/metrics.json and $run_dir/localization_plot.png"
}
trap cleanup EXIT INT TERM

echo "Right-angle corridor is open in Gazebo and RViz. Start from the horizontal corridor arm."
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID; verified one publisher each for /livox/lidar and /livox/imu."
echo "Keyboard control: w forward, x reverse, a/d turn, s or space stop, q quit. Ctrl-C also stops and saves."
sleep 4
python3 sim/scenarios/right_angle_corridor/teleop_wasd.py
