#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${RIGHT_ANGLE_ROS_DOMAIN_ID:-$((20 + RANDOM % 210))}"
export GAZEBO_MODEL_PATH="$ws_dir/vendor_gazebo_models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ws_dir/install/ros2_livox_simulation/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MASTER_URI="http://127.0.0.1:$((17000 + RANDOM % 12000))"
export MID360_DROP_FIRST_SCAN=1

run_dir="$(mktemp -d "$ws_dir/sim/output/right_angle_corridor/ground_mid360_XXXXXXXX")"
mkdir -p "$run_dir/ros_logs" "$run_dir/gazebo_logs"
export ROS_LOG_DIR="$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$run_dir/gazebo_logs"
echo "Starting static ground MID360 test in ROS_DOMAIN_ID=$ROS_DOMAIN_ID; output: $run_dir"
python3 sim/scenarios/right_angle_corridor/create_ground_mid360_world.py "$run_dir/world.sdf" >"$run_dir/world_generation.log"
if ! python3 - "$run_dir/world.sdf" <<'PY'
import sys
import xml.etree.ElementTree as ET

world = ET.parse(sys.argv[1]).getroot().find('world')
names = {model.get('name') for model in world.findall('model')}
allowed = {
    'corridor_floor', 'wall_south', 'wall_west_end',
    'wall_horizontal_north', 'wall_vertical_west', 'wall_east',
    'wall_north_end', 'landmark_cylinder_1', 'landmark_cylinder_2',
    'landmark_cylinder_3', 'landmark_cylinder_4', 'landmark_cylinder_5',
    'mid360_standalone',
}
if names != allowed:
    print(f'Unexpected world models: {sorted(names)}', file=sys.stderr)
    raise SystemExit(1)
PY
then
  echo "Generated world failed the model whitelist: $run_dir/world.sdf" >&2
  exit 1
fi

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/world.sdf" >"$run_dir/gazebo.log" 2>&1 & gazebo_pid=$!
gzclient --verbose >"$run_dir/gzclient.log" 2>&1 & gui_pid=$!
rviz2 -d sim/scenarios/right_angle_corridor/ground_mid360.rviz >"$run_dir/rviz.log" 2>&1 & rviz_pid=$!
setsid ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 world mid360_pattern >"$run_dir/world_sensor_tf.log" 2>&1 & tf_pid=$!
setsid python3 sim/plugin_mid360_adapter.py >"$run_dir/adapter.log" 2>&1 & adapter_pid=$!
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$ws_dir/sim/fastlio_sim.yaml" \
  -p "map_file_path:=$run_dir/fastlio_map.pcd" >"$run_dir/fastlio.log" 2>&1 & fastlio_pid=$!
setsid ros2 bag record -o "$run_dir/trajectory_bag" \
  /clock /ground_truth/model_states /ground_truth/link_states \
  /mid360_plugin/points /mid360_sim/imu /livox/lidar /livox/imu /Odometry \
  >"$run_dir/rosbag.log" 2>&1 & bag_pid=$!

cleanup() {
  trap - EXIT INT TERM
  kill -TERM "$rviz_pid" "$gui_pid" 2>/dev/null || true
  kill -INT -- "-$bag_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  kill -TERM "$gazebo_pid" 2>/dev/null || true
  wait "$gazebo_pid" 2>/dev/null || true
  timeout 30s ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/map_save.log" 2>&1 || true
  kill -INT -- "-$fastlio_pid" "-$adapter_pid" "-$tf_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$fastlio_pid" "-$adapter_pid" "-$tf_pid" 2>/dev/null || true
  for pid in "$rviz_pid" "$gui_pid" "$bag_pid" "$gazebo_pid" "$fastlio_pid" "$adapter_pid" "$tf_pid"; do wait "$pid" 2>/dev/null || true; done
  if [[ -f "$run_dir/trajectory_bag/metadata.yaml" ]]; then
    python3 sim/scenarios/right_angle_corridor/evaluate_run.py "$run_dir" >"$run_dir/evaluation_stdout.json" 2>"$run_dir/evaluation_stderr.log" || true
  fi
  echo "Ground MID360 test saved: $run_dir"
  echo "Bag: $run_dir/trajectory_bag"
  echo "Metrics: $run_dir/metrics.json"
}
trap cleanup EXIT INT TERM

for attempt in {1..15}; do
  lidar_info="$(timeout 4s ros2 topic info /livox/lidar --no-daemon --spin-time 1 2>&1 || true)"
  imu_info="$(timeout 4s ros2 topic info /livox/imu --no-daemon --spin-time 1 2>&1 || true)"
  lidar_count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$lidar_info")"
  imu_count="$(awk '/Publisher count:/ {print $3; exit}' <<<"$imu_info")"
  [[ "$lidar_count" == 1 && "$imu_count" == 1 ]] && break
  sleep 1
done
if [[ "${lidar_count:-}" != 1 || "${imu_count:-}" != 1 ]]; then
  echo "Sensor topics did not come up; inspect $run_dir/gazebo.log and adapter.log" >&2
  exit 1
fi
echo "Gazebo is open with only a standalone MID360; sensor origin z=0 on the corridor floor."
echo "RViz uses world coordinates and displays /mid360_plugin/points."
echo "No vehicle or teleop is running."
echo "Recording with no time limit; close Gazebo or press Ctrl-C to finish and save."
wait "$gui_pid"
