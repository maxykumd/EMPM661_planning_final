# algorithms/smoother.py
import sys
import os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rrt import is_collision_free_path


def smooth_path(path, obstacles, max_iterations=50, extra_clearance=0.05):
    """
    Shortcut smoother — tries to connect non-adjacent waypoints directly.
    If the straight line is collision-free, intermediate waypoints are removed.

    obstacles:       pre-inflated (CLEARANCE already baked in) — uses clearance=0.0
    extra_clearance: additional buffer on top of baked-in radius for smoothing safety
                     kept small (0.05) since obstacles are already inflated
    max_iterations:  max passes over the path — usually converges in 2-3
    """
    if path is None or len(path) < 3:
        return path

    # Add small extra buffer on top of already-inflated radii
    inflated = [(x, y, r + extra_clearance, vx, vy)
                for x, y, r, vx, vy in obstacles]

    smoothed  = list(path)
    improved  = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved  = False
        iteration += 1
        i = 0
        while i < len(smoothed) - 2:
            j = len(smoothed) - 1
            while j > i + 1:
                # clearance=0.0 — margin already in inflated radii
                if is_collision_free_path(smoothed[i], smoothed[j],
                                          inflated, clearance=0.0):
                    smoothed = smoothed[:i+1] + smoothed[j:]
                    improved = True
                    break
                j -= 1
            i += 1

    return smoothed


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mock_data  import START, GOAL, OBSTACLES, MAP_SIZE, CLEARANCE, \
                           STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS, SIGMA
    from birrt_star import birrt_star

    # Bake CLEARANCE in for standalone test
    obs = [(x, y, r+CLEARANCE, vx, vy) for x,y,r,vx,vy in OBSTACLES]

    path, fw_tree, rv_tree = birrt_star(
        start=START, goal=GOAL, obstacles=obs, map_size=MAP_SIZE,
        max_iter=5000, step_size=STEP_SIZE, sigma=SIGMA,
        goal_bias=GOAL_BIAS, rewire_radius=REWIRE_RADIUS
    )

    if path is None:
        print("No path found"); exit()

    smoothed = smooth_path(path, obs)

    def path_len(p):
        return sum(np.hypot(p[i][0]-p[i-1][0], p[i][1]-p[i-1][1])
                   for i in range(1, len(p)))

    print(f"Raw:      {len(path):3d} waypoints  {path_len(path):.3f}m")
    print(f"Smoothed: {len(smoothed):3d} waypoints  {path_len(smoothed):.3f}m")
    print(f"Saved:    {path_len(path)-path_len(smoothed):.3f}m  "
          f"({(1-path_len(smoothed)/path_len(path))*100:.1f}% reduction)")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    for ax, p, title in zip(axes, [path, smoothed], ['Raw Bi-RRT*', 'Smoothed']):
        ax.set_xlim(0, MAP_SIZE[0]); ax.set_ylim(0, MAP_SIZE[1])
        ax.set_aspect('equal'); ax.set_title(title); ax.grid(True, alpha=0.3)

        for obs_ in OBSTACLES:
            ax.add_patch(plt.Circle((obs_[0], obs_[1]), obs_[2],
                                    color='black', alpha=0.8))
            ax.add_patch(plt.Circle((obs_[0], obs_[1]), obs_[2]+CLEARANCE,
                                    color='red', fill=False,
                                    linestyle='--', linewidth=1.0, alpha=0.5))

        xs, ys = zip(*p)
        ax.plot(xs, ys, 'r-', linewidth=2, label='Path')
        ax.plot(xs, ys, 'ro', markersize=4)
        ax.plot(*p[0],  'go', markersize=12, label='Start')
        ax.plot(*p[-1], 'b*', markersize=12, label='Goal')
        ax.legend()

    plt.tight_layout(); plt.show()