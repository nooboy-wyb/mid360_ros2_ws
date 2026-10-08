#!/usr/bin/env python3
"""Reject empty maps and obviously diverged short-demo trajectories."""

import re
import sys
from pathlib import Path

import numpy as np


def main():
    run_dir = Path(sys.argv[1])
    odom = (run_dir / 'odometry.txt').read_text()
    xyz = [float(re.search(rf'^{axis}:\s*(\S+)', odom, re.M).group(1)) for axis in 'xyz']
    if any(abs(value) > 3.0 for value in xyz):
        raise SystemExit(f'FAST-LIO odometry diverged: {xyz}')

    with (run_dir / 'map_3d.pcd').open('rb') as pcd:
        while not pcd.readline().startswith(b'DATA binary'):
            pass
        points = np.fromfile(pcd, dtype=np.float32).reshape(-1, 8)
    if len(points) < 5000 or not np.isfinite(points[:, :3]).all():
        raise SystemExit(f'Invalid PCD: {len(points)} points')
    if np.abs(points[:, :3]).max() > 25.0:
        raise SystemExit('PCD contains large coordinate outliers')
    print(f'Validated {len(points)} PCD points and odometry {xyz}')


if __name__ == '__main__':
    main()
