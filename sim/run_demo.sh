#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"

mkdir -p sim/output
run_dir="$(mktemp -d sim/output/run_XXXXXXXX)"
echo "Run directory: $ws_dir/$run_dir"

ign gazebo -s -r sim/worlds/mesh_lab.sdf >"$run_dir/gazebo.log" 2>&1 &
sim_pid=$!
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
  '/mid360_sim/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU' \
  '/mid360_sim/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked' \
  '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist' \
  >"$run_dir/bridge.log" 2>&1 &
bridge_pid=$!
python3 sim/approx_mid360_adapter.py >"$run_dir/adapter.log" 2>&1 &
adapter_pid=$!
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file sim/fastlio_sim.yaml \
  -p "map_file_path:=$ws_dir/$run_dir/map_3d.pcd" \
  >"$run_dir/fastlio.log" 2>&1 &
fastlio_pid=$!

cleanup() {
  kill -INT "$fastlio_pid" 2>/dev/null || true
  sleep 1
  kill -TERM "$fastlio_pid" "$adapter_pid" "$bridge_pid" "$sim_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$fastlio_pid" "$adapter_pid" "$bridge_pid" "$sim_pid" 2>/dev/null || true
  wait "$fastlio_pid" "$adapter_pid" "$bridge_pid" "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 10
timeout 10s ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.12}, angular: {z: 0.15}}' \
  >"$run_dir/drive.log" 2>&1 || test "$?" -eq 124
sleep 3
timeout 8s ros2 topic echo /Odometry --once --field pose.pose.position \
  >"$run_dir/odometry.txt"
ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$run_dir/save.txt"
test -s "$run_dir/map_3d.pcd"
python3 sim/validate_run.py "$run_dir"
echo "Odometry:"
cat "$run_dir/odometry.txt"
echo "Map save:"
cat "$run_dir/save.txt"
echo "PCD: $ws_dir/$run_dir/map_3d.pcd"
