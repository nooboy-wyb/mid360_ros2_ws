#!/usr/bin/env bash
set -eo pipefail
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
map_path="${1:-$(ls -t sim/output/plugin_*/map_3d.pcd | head -n 1)}"
export ROS_LOG_DIR="$ws_dir/sim/output/rviz_logs"
mkdir -p "$ROS_LOG_DIR"
python3 sim/show_saved_map.py "$map_path" &
publisher_pid=$!
trap 'kill "$publisher_pid" 2>/dev/null || true' EXIT
rviz2 -d sim/map_review.rviz
