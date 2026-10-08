#!/usr/bin/env bash
set -eo pipefail
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
map_path="${1:-$(find sim/output -type f -name raw_accum_valid_odom.pcd -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)}"
if [[ -z "$map_path" || ! -s "$map_path" ]]; then
  echo "Raw PCD not found. Pass a path or run sim/run_raw_moving.sh first." >&2
  exit 2
fi
export ROS_LOG_DIR="$ws_dir/sim/output/raw_view_logs"
mkdir -p "$ROS_LOG_DIR"
python3 sim/show_raw_map.py "$map_path" &
publisher_pid=$!
trap 'kill "$publisher_pid" 2>/dev/null || true' EXIT
rviz2 -d sim/raw_saved_map.rviz
