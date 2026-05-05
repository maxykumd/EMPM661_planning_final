# experiments.py
# Runs all algorithms and collects metrics for paper

import numpy as np
import csv
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'algorithms'))

from mock_data  import (MAP_SIZE, OBSTACLES, START, GOAL,
                        STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS)
from rrt        import rrt
from birrt      import birrt
from birrt_star import birrt_star

RUNS = 50

results  = []

def path_length(path):
    if path is None:
        return None
    total = 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total += np.sqrt(dx**2 + dy**2)
    return round(total, 3)

print("=" * 60)
print(f"Running experiments — {RUNS} runs per algorithm")
print(f"Start: {START}  Goal: {GOAL}")
print(f"Obstacles: {len(OBSTACLES)}")
print("=" * 60)

# ── 1. Vanilla RRT ────────────────────────────────────────────
print(f"\n[1/3] Vanilla RRT...")
for run in range(RUNS):
    t0 = time.time()
    path, tree = rrt(
        start       = START,
        goal        = GOAL,
        obstacles   = OBSTACLES,
        map_size    = MAP_SIZE,
        max_iter    = 5000,
        step_size   = STEP_SIZE,
        goal_radius = 0.3,
        goal_bias   = GOAL_BIAS
    )
    elapsed = round(time.time() - t0, 3)
    plen    = path_length(path)
    results.append({
        'algorithm':    'RRT',
        'run':          run + 1,
        'success':      path is not None,
        'time':         elapsed,
        'path_length':  plen,
        'waypoints':    len(path) if path else 0,
        'nodes':        len(tree),
    })
    status = '✅' if path else '❌'
    print(f"  Run {run+1:2d}: {status} "
          f"time={elapsed:.3f}s  "
          f"length={plen}m  "
          f"nodes={len(tree)}")

# ── 2. Bi-RRT ─────────────────────────────────────────────────
print(f"\n[2/3] Bi-RRT...")
for run in range(RUNS):
    t0 = time.time()
    path, fw, rv = birrt(
        start     = START,
        goal      = GOAL,
        obstacles = OBSTACLES,
        map_size  = MAP_SIZE,
        max_iter  = 5000,
        step_size = STEP_SIZE,
        sigma     = 0.3,
        goal_bias = GOAL_BIAS
    )
    elapsed = round(time.time() - t0, 3)
    plen    = path_length(path)
    results.append({
        'algorithm':    'Bi-RRT',
        'run':          run + 1,
        'success':      path is not None,
        'time':         elapsed,
        'path_length':  plen,
        'waypoints':    len(path) if path else 0,
        'nodes':        len(fw) + len(rv),
    })
    status = '✅' if path else '❌'
    print(f"  Run {run+1:2d}: {status} "
          f"time={elapsed:.3f}s  "
          f"length={plen}m  "
          f"nodes={len(fw)+len(rv)}")

# ── 3. Bi-RRT* ────────────────────────────────────────────────
print(f"\n[3/3] Bi-RRT*...")
for run in range(RUNS):
    t0 = time.time()
    path, fw, rv = birrt_star(
        start         = START,
        goal          = GOAL,
        obstacles     = OBSTACLES,
        map_size      = MAP_SIZE,
        max_iter      = 5000,
        step_size     = STEP_SIZE,
        sigma         = 0.3,
        goal_bias     = GOAL_BIAS,
        rewire_radius = REWIRE_RADIUS,
        wa_path       = None
    )
    elapsed = round(time.time() - t0, 3)
    plen    = path_length(path)
    results.append({
        'algorithm':    'Bi-RRT*',
        'run':          run + 1,
        'success':      path is not None,
        'time':         elapsed,
        'path_length':  plen,
        'waypoints':    len(path) if path else 0,
        'nodes':        len(fw) + len(rv),
    })
    status = '✅' if path else '❌'
    print(f"  Run {run+1:2d}: {status} "
          f"time={elapsed:.3f}s  "
          f"length={plen}m  "
          f"nodes={len(fw)+len(rv)}")

# ── Save To CSV ───────────────────────────────────────────────
csv_path = os.path.join(os.path.dirname(__file__), 'results.csv')
with open(csv_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=[
        'algorithm', 'run', 'success',
        'time', 'path_length', 'waypoints', 'nodes'
    ])
    writer.writeheader()
    writer.writerows(results)

print(f"\nResults saved to results.csv")

# ── Print Summary ─────────────────────────────────────────────
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
print(f"{'Algorithm':<12} {'Success':>8} {'Avg Time':>10} "
      f"{'Avg Length':>12} {'Avg Waypoints':>15}")
print("-" * 60)

for algo in ['RRT', 'Bi-RRT', 'Bi-RRT*']:
    runs    = [r for r in results if r['algorithm'] == algo]
    success = [r for r in runs if r['success']]
    rate    = len(success) / RUNS * 100

    if success:
        avg_time = np.mean([r['time']        for r in success])
        avg_len  = np.mean([r['path_length'] for r in success])
        avg_wps  = np.mean([r['waypoints']   for r in success])
    else:
        avg_time = avg_len = avg_wps = 0

    print(f"{algo:<12} {rate:>7.0f}%  "
          f"{avg_time:>9.3f}s  "
          f"{avg_len:>11.2f}m  "
          f"{avg_wps:>14.1f}")

print("=" * 60)
print(f"\nFull results saved to: {csv_path}")