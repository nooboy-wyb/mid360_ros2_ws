#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export GAZEBO_MODEL_PATH="$ws_dir/vendor_gazebo_models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ws_dir/install/ros2_livox_simulation/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"

mkdir -p sim/output
run_dir="$(mktemp -d sim/output/plugin_XXXXXXXX)"
echo "Run directory: $ws_dir/$run_dir"
export ROS_LOG_DIR="$ws_dir/$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$ws_dir/$run_dir/gazebo_logs"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"
python3 sim/create_classic_world.py "$run_dir/mesh_lab_classic.sdf" "${MID360_SCENE:-house}"

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/mesh_lab_classic.sdf" >"$run_dir/gazebo.log" 2>&1 &
sim_pid=$!
python3 sim/plugin_mid360_adapter.py >"$run_dir/adapter.log" 2>&1 &
adapter_pid=$!
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file sim/fastlio_sim.yaml \
  -p "map_file_path:=$ws_dir/$run_dir/map_3d.pcd" \
  >"$run_dir/fastlio.log" 2>&1 &
fastlio_pid=$!

cleanup() {
  kill -INT "$fastlio_pid" 2>/dev/null || true
  sleep 1
  kill -TERM "$fastlio_pid" "$adapter_pid" "$sim_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$fastlio_pid" "$adapter_pid" "$sim_pid" 2>/dev/null || true
  wait "$fastlio_pid" "$adapter_pid" "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 12
timeout 8s ros2 topic echo /mid360_plugin/points --once --field width >"$run_dir/plugin_width.txt"
timeout 8s ros2 topic echo /livox/lidar --once --field point_num >"$run_dir/custom_point_num.txt"
timeout 8s ros2 topic echo /livox/imu --once --field header.stamp >"$run_dir/imu_stamp.txt"
timeout 10s ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.12}, angular: {z: 0.15}}' >"$run_dir/drive.log" 2>&1 || test "$?" -eq 124
sleep 3
timeout 8s ros2 topic echo /Odometry --once --field pose.pose.position >"$run_dir/odometry.txt"
ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/save.txt"
test -s "$run_dir/map_3d.pcd"
grep -q 'success=True' "$run_dir/save.txt"
python3 sim/validate_run.py "$run_dir"
echo "Plugin points: $(cat "$run_dir/plugin_width.txt")"
echo "CustomMsg points: $(cat "$run_dir/custom_point_num.txt")"
echo "PCD: $ws_dir/$run_dir/map_3d.pcd"
