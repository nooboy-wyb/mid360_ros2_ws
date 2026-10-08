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

scene=""
while (($#)); do
  case "$1" in
    --scene)
      (($# >= 2)) || { echo "--scene requires lab or home" >&2; exit 2; }
      scene="$2"; shift 2 ;;
    --scene=*) scene="${1#*=}"; shift ;;
    --help|-h)
      echo "Usage: $0 [--scene lab|home]"
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
if [[ -z "$scene" ]]; then
  read -rp "Select scene: [1] lab (simple TurtleBot3 World) [2] home (House): " choice
  case "$choice" in 1|lab) scene=lab ;; 2|home) scene=home ;; *) echo "Choose lab or home" >&2; exit 2 ;; esac
fi
case "$scene" in
  lab) world_scene=lab ;;
  home|house) scene=home; world_scene=house ;;
  *) echo "Unknown scene '$scene'; choose lab or home" >&2; exit 2 ;;
esac

mkdir -p sim/output
run_dir="$(mktemp -d sim/output/raw_teleop_XXXXXXXX)"
export ROS_LOG_DIR="$ws_dir/$run_dir/ros_logs"
export GAZEBO_LOG_PATH="$ws_dir/$run_dir/gazebo_logs"
mkdir -p "$ROS_LOG_DIR" "$GAZEBO_LOG_PATH"
python3 sim/create_classic_world.py "$run_dir/world.sdf" "$world_scene"

gzserver --verbose -s libgazebo_ros_init.so "$run_dir/world.sdf" >"$run_dir/gazebo.log" 2>&1 &
sim_pid=$!
python3 sim/raw_moving_capture.py "$run_dir" >"$run_dir/capture.log" 2>&1 &
capture_pid=$!
ros2 bag record -o "$run_dir/trajectory_bag" \
  /clock /cmd_vel /odom /ground_truth/model_states /ground_truth/link_states \
  /mid360_plugin/points /mid360_sim/imu /joint_states /tf /tf_static \
  >"$run_dir/rosbag.log" 2>&1 &
bag_pid=$!
gzclient --verbose >"$run_dir/gzclient.log" 2>&1 &
gui_pid=$!
rviz2 -d sim/raw_lidar_moving.rviz >"$run_dir/rviz.log" 2>&1 &
rviz_pid=$!

cleanup() {
  trap - EXIT INT TERM
  timeout 2s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  for pid in "$rviz_pid" "$gui_pid"; do kill -TERM "$pid" 2>/dev/null || true; done
  kill -INT "$bag_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  kill -INT "$capture_pid" 2>/dev/null || true
  wait "$capture_pid" 2>/dev/null || true
  kill -TERM "$sim_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$sim_pid" 2>/dev/null || true
  for pid in "$rviz_pid" "$gui_pid" "$sim_pid"; do wait "$pid" 2>/dev/null || true; done
  echo
  echo "Raw session saved in: $ws_dir/$run_dir"
  echo "Wheel-odometry trajectory: $ws_dir/$run_dir/trajectory.csv"
  echo "Synchronized ROS bag: $ws_dir/$run_dir/trajectory_bag"
  echo "Gazebo truth topics in bag: /ground_truth/model_states and /ground_truth/link_states"
  echo "MID360 truth link: turtlebot3_waffle_pi::mid360_mount"
  echo "Odometry-aligned raw PCD: $ws_dir/$run_dir/raw_accum_valid_odom.pcd"
  echo "View PCD: bash sim/view_raw_map.sh $ws_dir/$run_dir/raw_accum_valid_odom.pcd"
  echo "Replay sensor data later with: ros2 bag play $ws_dir/$run_dir/trajectory_bag"
  echo "The bag already contains /clock; do not add --clock when replaying it."
}
trap cleanup EXIT INT TERM

echo "Scene: $scene. Recording LiDAR, IMU, controls, wheel odometry, Gazebo ground truth and /clock; FAST-LIO2 is not started."
echo "Keyboard control: use teleop_twist_keyboard keys; press Ctrl-C in this terminal when finished."
sleep 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
