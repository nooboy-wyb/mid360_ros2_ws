#!/usr/bin/env bash
set -eo pipefail

ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID=73

bag_dir="${1:?Usage: bash sim/run_fastlio_timing_sweep.sh <trajectory_bag> [output_dir] [comma-separated trials]}"
out_dir="${2:-$(dirname "$bag_dir")/fastlio_timing_sweep}"
trial_list="${3:-minus_5ms,zero,plus_5ms}"
bag_dir="$(realpath "$bag_dir")"
out_dir="$(realpath -m "$out_dir")"
mkdir -p "$out_dir"
cp "$(dirname "$bag_dir")/fastlio_eval/analyze_evaluation.py" "$out_dir/analyze_evaluation.py"

cleanup_pids=()
cleanup() {
  for pid in "${cleanup_pids[@]:-}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${cleanup_pids[@]:-}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${cleanup_pids[@]:-}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

IFS=',' read -r -a trials <<< "$trial_list"
for trial in "${trials[@]}"; do
  case "$trial" in
    minus_5ms) shift_ns=-5000000 ;;
    zero) shift_ns=0 ;;
    plus_5ms) shift_ns=5000000 ;;
    plus_10ms) shift_ns=10000000 ;;
    plus_20ms) shift_ns=20000000 ;;
    *) echo "Unknown trial '$trial'" >&2; exit 2 ;;
  esac
  trial_dir="$out_dir/$trial"
  rm -rf "$trial_dir"
  mkdir -p "$trial_dir"
  export ROS_LOG_DIR="$trial_dir/ros_logs"
  mkdir -p "$ROS_LOG_DIR"
  export MID360_DROP_FIRST_SCAN=1
  export MID360_LIDAR_STAMP_SHIFT_NS="$shift_ns"

  setsid python3 sim/plugin_mid360_adapter.py >"$trial_dir/adapter.log" 2>&1 & adapter_pid=$!
  setsid ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file "$ws_dir/sim/fastlio_sim.yaml" \
    -p "map_file_path:=$trial_dir/fastlio_map.pcd" \
    >"$trial_dir/fastlio.log" 2>&1 & fastlio_pid=$!
  setsid ros2 bag record -o "$trial_dir/fastlio_odom_bag" /Odometry \
    >"$trial_dir/recorder.log" 2>&1 & recorder_pid=$!
  cleanup_pids=("$adapter_pid" "$fastlio_pid" "$recorder_pid")

  sleep 5
  # The source bag carries the simulation clock. Do not use rosbag's --clock,
  # which would add a second clock based on wall-time recording stamps.
  ros2 bag play "$bag_dir" --rate 1.0 --disable-keyboard-controls >"$trial_dir/playback.log" 2>&1
  sleep 3
  ros2 service call /map_save std_srvs/srv/Trigger '{}' >"$trial_dir/map_save.log" 2>&1 || true
  kill -INT -- "-$recorder_pid" 2>/dev/null || true
  wait "$recorder_pid" 2>/dev/null || true
  kill -INT -- "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-$fastlio_pid" "-$adapter_pid" 2>/dev/null || true
  wait "$fastlio_pid" "$adapter_pid" 2>/dev/null || true
  cleanup_pids=()

  if rg -q 'jump back in time|lidar loop back' "$trial_dir/fastlio.log"; then
    echo "Clock or sensor time moved backwards in $trial; refusing to score this run." >&2
    exit 1
  fi

  FASTLIO_EVAL_OUT="$trial_dir" FASTLIO_ODOM_BAG="$trial_dir/fastlio_odom_bag" \
    python3 "$out_dir/analyze_evaluation.py" >"$trial_dir/metrics_stdout.json"
  echo "Finished $trial; metrics: $trial_dir/metrics.json"
done

python3 - "$out_dir" <<'PY'
import csv, json, pathlib, sys
root=pathlib.Path(sys.argv[1])
fields=['trial','se3_aligned_ape_rmse_m','first_position_anchored_rmse_m','first_position_anchored_endpoint_m','rpe_1s_rmse_m','rpe_5s_rmse_m','rpe_10s_rmse_m','path_length_ratio','odom_samples']
with (root/'comparison.csv').open('w', newline='') as f:
    w=csv.DictWriter(f, fieldnames=fields); w.writeheader()
    for name in ('minus_5ms','zero','plus_5ms','plus_10ms','plus_20ms'):
        metrics_path=root/name/'metrics.json'
        if not metrics_path.exists():
            continue
        m=json.loads(metrics_path.read_text())
        w.writerow({'trial':name, **{k:m.get(k,'') for k in fields[1:]}})
print(root/'comparison.csv')
PY
