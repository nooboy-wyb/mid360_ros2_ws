#!/usr/bin/env python3
"""Compare recorded FAST-LIO2 odometry to Gazebo MID360-link truth."""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock

run_dir = Path(sys.argv[1]).resolve()
bag_dir = run_dir / 'trajectory_bag'
target_links = ('mid360_standalone::mid360_mount', 'turtlebot3_waffle_pi::mid360_mount')
target_link = None
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
clocks, truth_rows, odom_rows = [], [], []
while reader.has_next():
    topic, data, recv_ns = reader.read_next()
    if topic == '/clock':
        msg = deserialize_message(data, Clock)
        clocks.append((recv_ns, msg.clock.sec + msg.clock.nanosec * 1e-9))
    elif topic == '/ground_truth/link_states':
        msg = deserialize_message(data, LinkStates)
        found = next(((name, msg.name.index(name)) for name in target_links if name in msg.name), None)
        if found is None:
            continue
        target_link, i = found
        p, q = msg.pose[i].position, msg.pose[i].orientation
        truth_rows.append((recv_ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    elif topic == '/Odometry':
        msg = deserialize_message(data, Odometry)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom_rows.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

if len(clocks) < 2 or len(truth_rows) < 2 or len(odom_rows) < 20:
    raise SystemExit(f'Insufficient data: clocks={len(clocks)} truth={len(truth_rows)} odom={len(odom_rows)}')
clock = np.asarray(clocks, dtype=float)
truth_raw = np.asarray(truth_rows, dtype=float)
truth_time = np.interp(truth_raw[:, 0], clock[:, 0], clock[:, 1])
truth_order = np.argsort(truth_time)
truth_time = truth_time[truth_order]
truth = truth_raw[truth_order, 1:4]
odom = np.asarray(odom_rows, dtype=float)
odom = odom[np.argsort(odom[:, 0])]
# Remove duplicate odometry stamps if bag recorder delivered repeated samples.
_, unique_idx = np.unique(odom[:, 0], return_index=True)
odom = odom[np.sort(unique_idx)]
times = odom[:, 0]
route_status_path = run_dir / 'route_status.json'
route_end = float(json.loads(route_status_path.read_text())['sim_time_s']) if route_status_path.exists() else float(truth_time[-1])
mask = (times >= truth_time[0]) & (times <= min(truth_time[-1], route_end + 0.1))
times, estimate = times[mask], odom[mask, 1:4]
truth_eval = np.column_stack([np.interp(times, truth_time, truth[:, k]) for k in range(3)])

src_mean, dst_mean = estimate.mean(axis=0), truth_eval.mean(axis=0)
u, _, vt = np.linalg.svd((estimate - src_mean).T @ (truth_eval - dst_mean))
fix = np.eye(3); fix[2, 2] = np.linalg.det(u @ vt)
rot = u @ fix @ vt
aligned = (estimate - src_mean) @ rot + dst_mean
ape = np.linalg.norm(aligned - truth_eval, axis=1)
anchored = (estimate - estimate[0]) @ rot + truth_eval[0]
anchored_error = np.linalg.norm(anchored - truth_eval, axis=1)


def path_length(x):
    return float(np.linalg.norm(np.diff(x, axis=0), axis=1).sum())


def interp_xyz(ts, xyz, query):
    return np.column_stack([np.interp(query, ts, xyz[:, k]) for k in range(3)])

rpe = {}
for seconds in (1.0, 2.0, 5.0):
    valid = times + seconds <= times[-1]
    if not valid.any():
        continue
    t0, t1 = times[valid], times[valid] + seconds
    e0, e1 = interp_xyz(times, estimate, t0), interp_xyz(times, estimate, t1)
    g0, g1 = interp_xyz(times, truth_eval, t0), interp_xyz(times, truth_eval, t1)
    err = np.linalg.norm((e1 - e0) @ rot - (g1 - g0), axis=1)
    rpe[f'{seconds:g}s'] = {'rmse_m': float(np.sqrt(np.mean(err**2))),
                             'p95_m': float(np.percentile(err, 95)), 'count': int(len(err))}

phases = []
events_path = run_dir / 'route_events.csv'
if events_path.exists():
    with events_path.open() as f:
        for row in csv.DictReader(f):
            row['start_sim_s'] = float(row['start_sim_s'])
            row['end_sim_s'] = float(row['end_sim_s'])
            row['start_yaw'] = float(row['start_yaw'])
            row['end_yaw'] = float(row['end_yaw'])
            row['delta_yaw_deg'] = float(np.degrees(np.arctan2(
                np.sin(row['end_yaw'] - row['start_yaw']),
                np.cos(row['end_yaw'] - row['start_yaw']))))
            phases.append(row)

phase_names = []
for t in times:
    phase = 'unlabeled'
    for event in phases:
        if event['start_sim_s'] <= t <= event['end_sim_s']:
            phase = event['phase']
            break
    phase_names.append(phase)

metrics = {
    'bag': str(bag_dir),
    'ground_truth_link': target_link,
    'odom_samples': int(len(times)),
    'sim_time_start_s': float(times[0]),
    'sim_time_end_s': float(times[-1]),
    'route_end_time_s': route_end,
    'global_se3_aligned_ape_rmse_m': float(np.sqrt(np.mean(ape**2))),
    'global_se3_aligned_ape_median_m': float(np.median(ape)),
    'global_se3_aligned_ape_p95_m': float(np.percentile(ape, 95)),
    'first_pose_anchored_rmse_m': float(np.sqrt(np.mean(anchored_error**2))),
    'first_pose_anchored_endpoint_error_m': float(anchored_error[-1]),
    'truth_path_length_m': path_length(truth_eval),
    'fastlio_path_length_m': path_length(estimate),
    'path_length_ratio': path_length(estimate) / max(path_length(truth_eval), 1e-9),
    'rpe': rpe,
    'route_events': phases,
}
(run_dir / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
with (run_dir / 'aligned_trajectory.csv').open('w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['sim_time_s', 'phase', 'truth_x', 'truth_y', 'truth_z',
                     'fastlio_aligned_x', 'fastlio_aligned_y', 'fastlio_aligned_z',
                     'ape_m', 'first_pose_anchored_error_m'])
    for row in zip(times, phase_names, truth_eval, aligned, ape, anchored_error):
        writer.writerow([row[0], row[1], *row[2], *row[3], row[4], row[5]])

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].plot(truth_eval[:, 0], truth_eval[:, 1], color='black', linewidth=2, label='Gazebo truth')
axes[0].plot(aligned[:, 0], aligned[:, 1], color='#e45756', linewidth=1.5, label='FAST-LIO2 (SE3 aligned)')
axes[0].set_aspect('equal', adjustable='datalim')
axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)'); axes[0].set_title('Straight → left 90° → straight')
axes[0].grid(True, alpha=.3); axes[0].legend()
axes[1].plot(times - times[0], ape, label='Global aligned APE')
axes[1].plot(times - times[0], anchored_error, label='First-pose anchored')
axes[1].set_xlabel('Simulation time from first odometry (s)'); axes[1].set_ylabel('Position error (m)')
axes[1].set_title('FAST-LIO2 position error'); axes[1].grid(True, alpha=.3); axes[1].legend()
fig.tight_layout(); fig.savefig(run_dir / 'localization_plot.png', dpi=180)

summary = (
    '# 直角走廊 FAST-LIO2 单程定位测试\n\n'
    f"- 轨迹样本：{len(times)}，仿真时间 {times[0]:.2f}–{times[-1]:.2f} s。\n"
    f"- 全局 SE(3) 对齐 APE：RMSE {metrics['global_se3_aligned_ape_rmse_m']:.3f} m，"
    f"p95 {metrics['global_se3_aligned_ape_p95_m']:.3f} m。\n"
    f"- 首位姿锚定误差：RMSE {metrics['first_pose_anchored_rmse_m']:.3f} m，"
    f"末端 {metrics['first_pose_anchored_endpoint_error_m']:.3f} m。\n"
    f"- 路程：真值 {metrics['truth_path_length_m']:.2f} m，FAST-LIO2 {metrics['fastlio_path_length_m']:.2f} m，"
    f"比例 {metrics['path_length_ratio']:.2f}。\n"
    f"- 转弯角度（真值控制器）：{next((p['delta_yaw_deg'] for p in phases if p['phase']=='TURN_LEFT_90'), float('nan')):.2f}°。\n\n"
    '全局 APE 已做刚体对齐，不表示绝对误差；同时查看首位姿锚定误差和轨迹图。此场景无回环重复路线，适合观察前端单程里程计，不用于评估闭环优化。\n'
)
(run_dir / 'evaluation.md').write_text(summary)
print(json.dumps(metrics, indent=2))
