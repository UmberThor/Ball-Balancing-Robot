# Metrics of the runs logged with --log: python3 metrics.py *.csv

import sys
import numpy as np

# METRICS CONSTANTS
PX_PER_MM = 50 / 20     # the ball radius is 50 px in the frame and 20 mm in reality
T_SKIP = 5.0            # [s] the start of the run is not steady state
GAP = 0.1               # [s] rows further apart than this are separated by a loss of the ball

print(f"{'run':<30}{'err mean':>10}{'err std':>10}{'u mean':>10}{'du mean':>10}")
print(f"{'':<30}{'[mm]':>10}{'[mm]':>10}{'[deg]':>10}{'[deg]':>10}")

for path in sys.argv[1:]:
    t, ex, ey, ux, uy = np.loadtxt(path, delimiter=",", skiprows=1, unpack=True)
    keep = t > t[0] + T_SKIP
    t = t[keep]
    e = np.column_stack([ex, ey])[keep] / PX_PER_MM
    u = np.column_stack([ux, uy])[keep]

    e_mean = e.mean(axis=0)
    err_mean = np.linalg.norm(e_mean)
    err_std = np.sqrt(np.mean(np.sum((e - e_mean)**2, axis=1)))

    # the slopes are small, so a slope is an angle in rad
    du = np.linalg.norm(np.diff(u, axis=0), axis=1)
    du = du[np.diff(t) < GAP]
    u_mean = np.degrees(np.mean(np.linalg.norm(u, axis=1)))
    du_mean = np.degrees(np.mean(du))

    print(f"{path:<30}{err_mean:>10.1f}{err_std:>10.1f}{u_mean:>10.2f}{du_mean:>10.2f}")
