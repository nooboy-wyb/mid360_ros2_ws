#!/usr/bin/env python3
"""Create a like-for-like table from right-angle-corridor evaluation outputs."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_run(path: Path, warmup_s: float) -> dict:
    run = path.resolve()
    metrics = json.loads((run / 'metrics.json').read_text())
    with (run / 'aligned_trajectory.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError(f'{run}: aligned_trajectory.csv has fewer than two rows')

    times = np.array([float(row['sim_time_s']) for row in rows])
    truth = np.array([[float(row[k]) for k in ('truth_x', 'truth_y', 'truth_z')]
                      for row in rows])
    estimate = np.array([[float(row[k]) for k in ('fastlio_aligned_x', 'fastlio_aligned_y', 'fastlio_aligned_z')]
                         for row in rows])
    phases = np.array([row['phase'] for row in rows])
    stationary = float(metrics.get('truth_path_length_m', 0.0)) < 0.05
    duration = float(times[-1] - times[0])
    common = {
        'run': run.name,
        'type': 'static' if stationary else 'moving',
        'duration_s': duration,
        'samples': len(rows),
        'score_start_s': '',
        'static_rmse_cm': '',
        'static_p95_cm': '',
        'static_max_cm': '',
        'static_end_cm': '',
        'static_drift_cm_min': '',
        'ape_rmse_cm': '',
        'ape_p95_cm': '',
        'rpe_1s_rmse_cm': '',
        'rpe_5s_rmse_cm': '',
        'truth_path_m': '',
        'path_error_pct': '',
    }

    if stationary:
        # A static trajectory has no meaningful world-frame heading. Relative
        # distances are invariant to the one rigid transform in the eval CSV.
        score_start = times[0] + warmup_s
        idx = int(np.searchsorted(times, score_start, side='left'))
        if len(times) - idx < 2:
            raise ValueError(f'{run}: only {len(times) - idx} samples after {warmup_s:g}s warmup')
        scored = estimate[idx:]
        displacement = np.linalg.norm(scored - scored[0], axis=1)
        scored_duration_min = max((times[-1] - times[idx]) / 60.0, 1e-12)
        common.update({
            'score_start_s': f'{times[idx]:.3f}',
            'static_rmse_cm': f'{np.sqrt(np.mean(displacement**2)) * 100:.3f}',
            'static_p95_cm': f'{np.percentile(displacement, 95) * 100:.3f}',
            'static_max_cm': f'{displacement.max() * 100:.3f}',
            'static_end_cm': f'{displacement[-1] * 100:.3f}',
            'static_drift_cm_min': f'{displacement[-1] / scored_duration_min * 100:.3f}',
        })
        return common

    # Score the route phases when available. This excludes pre-route startup
    # only when the route logger explicitly labels that period as unlabeled.
    route_mask = phases != 'unlabeled'
    if route_mask.sum() >= 2:
        score_indices = np.flatnonzero(route_mask)
    else:
        score_indices = np.arange(len(rows))
    t = times[score_indices]
    g = truth[score_indices]
    e = estimate[score_indices]
    ape = np.linalg.norm(e - g, axis=1)
    gt_path = float(np.linalg.norm(np.diff(g, axis=0), axis=1).sum())
    est_path = float(np.linalg.norm(np.diff(e, axis=0), axis=1).sum())

    def rpe(seconds: float) -> float:
        errors = []
        for i, t0 in enumerate(t):
            j = int(np.searchsorted(t, t0 + seconds, side='left'))
            if j < len(t) and abs(t[j] - (t0 + seconds)) <= 0.12:
                de = e[j] - e[i]
                dg = g[j] - g[i]
                errors.append(np.linalg.norm(de - dg))
        return float(np.sqrt(np.mean(np.square(errors)))) if errors else float('nan')

    common.update({
        'score_start_s': f'{t[0]:.3f}',
        'ape_rmse_cm': f'{np.sqrt(np.mean(ape**2)) * 100:.3f}',
        'ape_p95_cm': f'{np.percentile(ape, 95) * 100:.3f}',
        'rpe_1s_rmse_cm': f'{rpe(1.0) * 100:.3f}',
        'rpe_5s_rmse_cm': f'{rpe(5.0) * 100:.3f}',
        'truth_path_m': f'{gt_path:.3f}',
        'path_error_pct': f'{abs(est_path - gt_path) / max(gt_path, 1e-9) * 100:.2f}',
    })
    return common


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path, help='run directories containing metrics.json and aligned_trajectory.csv')
    parser.add_argument('--warmup-seconds', type=float, default=10.0,
                        help='fixed startup exclusion for every static run (default: 10)')
    parser.add_argument('--output', type=Path, default=Path('comparison.csv'),
                        help='comparison CSV output path (default: ./comparison.csv)')
    args = parser.parse_args()
    if args.warmup_seconds < 0:
        parser.error('--warmup-seconds must be non-negative')

    results = [load_run(path, args.warmup_seconds) for path in args.runs]
    columns = list(results[0])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(results)

    print(f'Wrote {len(results)} runs to {args.output}')
    print('Static rows compare post-warmup displacement; moving rows compare route APE/RPE and path error.')


if __name__ == '__main__':
    main()
