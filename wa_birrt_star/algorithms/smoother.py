# algorithms/smoother.py
import sys
import os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rrt import is_collision_free_path

def smooth_path(path, obstacles, max_iterations=50, extra_clearance=0.10):
    """
    extra_clearance: additional buffer on top of CLEARANCE
                     makes shortcuts stay further from obstacles
    """
    if path is None or len(path) < 3:
        return path

    # Build inflated obstacles for smoothing check
    # Add extra_clearance on top of existing obstacle radius
    inflated_obstacles = []
    for obs in obstacles:
        x, y, r, vx, vy = obs
        inflated_obstacles.append(
            (x, y, r + extra_clearance, vx, vy)
        )

    smoothed  = list(path)
    improved  = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved  = False
        iteration += 1
        i         = 0

        while i < len(smoothed) - 2:
            j = len(smoothed) - 1

            while j > i + 1:
                # Check against inflated obstacles for safety
                if is_collision_free_path(smoothed[i],
                                          smoothed[j],
                                          inflated_obstacles,
                                          num_checks=25):
                    smoothed = smoothed[:i+1] + smoothed[j:]
                    improved = True
                    break
                j -= 1
            i += 1

    return smoothed

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mock_data import (START, GOAL, OBSTACLES,
                           MAP_SIZE, CLEARANCE,
                           STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS)
    from birrt_star import birrt_star

    # Get raw path
    path, fw_tree, rv_tree = birrt_star(
        start=START, goal=GOAL, obstacles=OBSTACLES,
        map_size=MAP_SIZE, max_iter=5000,
        step_size=STEP_SIZE, sigma=0.3,
        goal_bias=GOAL_BIAS, rewire_radius=REWIRE_RADIUS
    )

    if path is None:
        print("No path found")
        exit()

    smoothed = smooth_path(path, OBSTACLES)

    def path_len(p):
        total = 0
        for i in range(1, len(p)):
            dx = p[i][0] - p[i-1][0]
            dy = p[i][1] - p[i-1][1]
            total += np.sqrt(dx**2 + dy**2)
        return total

    print(f"Original: {len(path)} waypoints, {path_len(path):.2f}m")
    print(f"Smoothed: {len(smoothed)} waypoints, {path_len(smoothed):.2f}m")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    for ax, p, title in zip(axes,
                             [path, smoothed],
                             ['Raw Bi-RRT*', 'Smoothed']):
        ax.set_xlim(0, MAP_SIZE[0])
        ax.set_ylim(0, MAP_SIZE[1])
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

        for obs in OBSTACLES:
            circle = plt.Circle((obs[0], obs[1]), obs[2],
                                 color='black', alpha=0.8)
            ax.add_patch(circle)
            ring = plt.Circle((obs[0], obs[1]), obs[2] + CLEARANCE,
                               color='red', fill=False,
                               linestyle='--', linewidth=1.0, alpha=0.5)
            ax.add_patch(ring)

        xs = [pt[0] for pt in p]
        ys = [pt[1] for pt in p]
        ax.plot(xs, ys, 'r-', linewidth=2, label='Path')

        for pt in p:
            ax.plot(*pt, 'ro', markersize=4)

        ax.plot(*p[0],  'go', markersize=12, label='Start')
        ax.plot(*p[-1], 'b*', markersize=12, label='Goal')
        ax.legend()

    plt.tight_layout()
    plt.show()