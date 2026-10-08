#!/usr/bin/env python3
"""Combine static raw returns and track occupied voxels as scans accumulate."""

import argparse
import json
from pathlib import Path

import numpy as np

from raw_lidar_capture import DTYPE, write_pcd


def read_pcd(path):
    data = path.read_bytes()
    marker = b'DATA binary\n'
    start = data.index(marker) + len(marker)
    return np.frombuffer(data, dtype=DTYPE, offset=start).copy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--voxel-size', type=float, default=0.1)
    args = parser.parse_args()
    if args.voxel_size <= 0:
        parser.error('voxel size must be positive')
    files = sorted(args.output_dir.glob('raw_valid_*.pcd'))
    if len(files) < 30:
        parser.error(f'expected at least 30 raw valid scans, found {len(files)}')
    occupied = set()
    scans = []
    checkpoints = {}
    for index, path in enumerate(files, start=1):
        points = read_pcd(path)
        scans.append(points)
        xyz = np.column_stack((points['x'], points['y'], points['z']))
        cells = np.floor(xyz / args.voxel_size).astype(np.int32)
        occupied.update(map(tuple, cells))
        if index in (1, 10, 30, 100):
            checkpoints[str(index)] = {
                'valid_returns_total': sum(len(scan) for scan in scans),
                'occupied_voxels': len(occupied),
            }
    combined = np.concatenate(scans)
    write_pcd(args.output_dir / f'raw_accum_valid_{len(files):03d}.pcd', combined)
    report = {
        'source': f'{len(files)} static raw plugin scans; no motion correction or FAST-LIO2',
        'voxel_size_m': args.voxel_size,
        'checkpoints': checkpoints,
        'note': 'Voxel growth measures sampled endpoints, not map accuracy or complete coverage.',
    }
    (args.output_dir / 'raw_coverage.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
