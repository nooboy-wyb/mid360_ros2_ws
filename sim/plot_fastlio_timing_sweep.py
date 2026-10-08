#!/usr/bin/env python3
"""Plot FAST-LIO2 trajectories and localization errors from a timing sweep."""
import csv
import json
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(os.environ.get('FASTLIO_SWEEP_DIR', 'sim/output/raw_teleop_1jUuwzV7/fastlio_timing_sweep'))
trials = [
    ('minus_5ms', '-5 ms'),
    ('zero', '0 ms'),
    ('plus_5ms', '+5 ms'),
    ('plus_10ms', '+10 ms'),
    ('plus_20ms', '+20 ms'),
]
colors = ['#8c564b', '#1f77b4', '#2ca02c', '#ff7f0e', '#d62728']
fig, axes = plt.subplots(1, 3, figsize=(16, 5.4), gridspec_kw={'width_ratios': [1.2, 1.3, 1]})
metrics_rows = []

for (name, label), color in zip(trials, colors):
    csv_path = root / name / 'aligned_trajectory.csv'
    metrics_path = root / name / 'metrics.json'
    if not csv_path.exists() or not metrics_path.exists():
        continue
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    data = {k: np.asarray([float(row[k]) for row in rows]) for k in rows[0]}
    t = data['stamp_s'] - data['stamp_s'][0]
    axes[0].plot(data['truth_x'], data['truth_y'], color='black', linewidth=2.2,
                 label='Gazebo truth' if not axes[0].lines else None)
    axes[0].plot(data['fastlio_x'], data['fastlio_y'], color=color, alpha=.9,
                 linewidth=1.1, label=label)
    axes[1].plot(t, data['se3_aligned_error_m'], color=color, linewidth=1.0, label=label)
    metrics_rows.append((label, json.loads(metrics_path.read_text())))

axes[0].set_title('XY trajectory (SE3 aligned)')
axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
axes[0].axis('equal'); axes[0].grid(True, alpha=.25); axes[0].legend(fontsize=8)
axes[1].set_title('Position error over time')
axes[1].set_xlabel('Simulation time from first pose (s)'); axes[1].set_ylabel('APE (m)')
axes[1].grid(True, alpha=.25); axes[1].legend(fontsize=8)

labels = [row[0] for row in metrics_rows]
x = np.arange(len(labels)); width = .36
ape = [row[1]['se3_aligned_ape_rmse_m'] for row in metrics_rows]
anchored = [row[1]['first_position_anchored_rmse_m'] for row in metrics_rows]
axes[2].bar(x-width/2, ape, width, label='SE3 aligned APE', color='#4c78a8')
axes[2].bar(x+width/2, anchored, width, label='First-pose anchored', color='#f58518')
axes[2].set_title('Position RMSE')
axes[2].set_ylabel('RMSE (m)'); axes[2].set_xticks(x, labels, rotation=30, ha='right')
axes[2].grid(True, axis='y', alpha=.25); axes[2].legend(fontsize=8)

fig.suptitle('FAST-LIO2 localization vs LiDAR timestamp offset', fontsize=14)
fig.tight_layout()
output = root / 'fastlio_timing_sweep.png'
fig.savefig(output, dpi=180, bbox_inches='tight')
print(output)
