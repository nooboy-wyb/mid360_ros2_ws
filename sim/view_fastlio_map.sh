#!/usr/bin/env bash
set -eo pipefail
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
if (($# != 1)) || [[ ! -s "$1" ]]; then
  echo "Usage: bash sim/view_fastlio_map.sh <FAST-LIO2 .pcd>" >&2
  exit 2
fi
map_path="$(realpath "$1")"
export ROS_LOG_DIR="$ws_dir/sim/output/fastlio_map_view_logs"
mkdir -p "$ROS_LOG_DIR"
python3 sim/show_saved_map.py "$map_path" &
publisher_pid=$!
trap 'kill "$publisher_pid" 2>/dev/null || true' EXIT
rviz2 -d sim/map_review.rviz
