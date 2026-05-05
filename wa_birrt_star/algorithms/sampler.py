# algorithms/sampler.py
import sys
import os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_data import CLEARANCE


def sample_state(tree, goal, obstacles, wa_path,
                 map_size, corridor_width=0.8, progress=0.0):
    """
    WA*-guided adaptive sampling with 4 behaviors:
      60% — sample near WA* corridor (guided)
      15% — sample away from predicted obstacle positions
      15% — sample toward goal
      10% — pure random

    tree:           list of nodes
    goal:           (x, y)
    obstacles:      list of (x, y, radius, vx, vy)
    wa_path:        list of (x,y) from Weighted A*
    map_size:       (width, height) in meters
    corridor_width: base width — adapts based on progress
    progress:       0.0 (start) to 1.0 (near goal)
    returns:        (x, y) sample point
    """
    # Adaptive corridor — wide early, narrow near goal
    adaptive_width = max(0.3, corridor_width * (1.0 - progress * 0.7))

    p = np.random.random()

    if p < 0.60 and wa_path is not None and len(wa_path) > 0:
        return sample_near_corridor(wa_path, adaptive_width, map_size)
    elif p < 0.75:
        return goal
    elif p < 0.90 and len(obstacles) > 0:
        return sample_away_from_obstacles(obstacles, map_size)
    else:
        return random_sample(map_size)


def random_sample(map_size):
    """Uniform random sample within map bounds."""
    x = np.random.uniform(CLEARANCE, map_size[0] - CLEARANCE)
    y = np.random.uniform(CLEARANCE, map_size[1] - CLEARANCE)
    return (x, y)


def sample_near_corridor(wa_path, corridor_width, map_size):
    """
    Sample near WA* corridor.
    Pick random waypoint then sample within corridor_width of it.
    """
    waypoint = wa_path[np.random.randint(len(wa_path))]
    angle    = np.random.uniform(0, 2 * np.pi)
    dist     = np.random.uniform(0, corridor_width)

    x = waypoint[0] + dist * np.cos(angle)
    y = waypoint[1] + dist * np.sin(angle)

    # Clamp to map bounds
    x = np.clip(x, CLEARANCE, map_size[0] - CLEARANCE)
    y = np.clip(y, CLEARANCE, map_size[1] - CLEARANCE)

    return (float(x), float(y))


def sample_away_from_obstacles(obstacles, map_size,
                                steps_ahead=5,
                                safety_margin=0.4):
    """
    Sample away from predicted obstacle positions.
    Predicts where obstacles will be and avoids those regions.
    """
    # Predict future positions
    predicted = []
    for obs in obstacles:
        x, y, r, vx, vy = obs
        px = x + vx * steps_ahead
        py = y + vy * steps_ahead
        predicted.append((px, py, r + safety_margin))

    # Try up to 10 times to find safe sample
    for _ in range(10):
        candidate = random_sample(map_size)
        cx, cy    = candidate
        safe      = True

        for px, py, pr in predicted:
            dist = np.sqrt((cx - px)**2 + (cy - py)**2)
            if dist < pr:
                safe = False
                break

        if safe:
            return candidate

    return random_sample(map_size)


def compute_progress(tree, goal):
    """
    Compute planning progress 0.0 to 1.0.
    Based on closest tree node to goal vs initial distance.
    """
    if not tree:
        return 0.0

    min_dist = float('inf')
    for node in tree:
        dx   = node["pos"][0] - goal[0]
        dy   = node["pos"][1] - goal[1]
        dist = np.sqrt(dx**2 + dy**2)
        if dist < min_dist:
            min_dist = dist

    dx0       = tree[0]["pos"][0] - goal[0]
    dy0       = tree[0]["pos"][1] - goal[1]
    init_dist = np.sqrt(dx0**2 + dy0**2)

    if init_dist == 0:
        return 1.0

    return float(np.clip(1.0 - (min_dist / init_dist), 0.0, 1.0))


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mock_data import START, GOAL, OBSTACLES, MAP_SIZE

    # Fake WA* path — diagonal from start to goal
    wa_path = [
        (START[0] + i * (GOAL[0] - START[0]) / 20,
         START[1] + i * (GOAL[1] - START[1]) / 20)
        for i in range(21)
    ]

    fake_tree = [{"pos": START, "parent": None, "cost": 0.0}]

    # Generate 1000 samples at progress=0.3
    samples = [
        sample_state(fake_tree, GOAL, OBSTACLES,
                     wa_path, MAP_SIZE,
                     corridor_width=0.8,
                     progress=0.3)
        for _ in range(1000)
    ]

    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, MAP_SIZE[0])
    ax.set_ylim(0, MAP_SIZE[1])
    ax.set_aspect('equal')
    ax.set_title("WA*-Guided Sampling — 1000 samples\n"
                 "Should cluster along diagonal corridor")
    ax.grid(True, alpha=0.3)

    for obs in OBSTACLES:
        circle = plt.Circle((obs[0], obs[1]), obs[2],
                             color='black', alpha=0.8)
        ax.add_patch(circle)

    ax.scatter(xs, ys, s=3, color='blue', alpha=0.4, label='Samples')

    wa_xs = [p[0] for p in wa_path]
    wa_ys = [p[1] for p in wa_path]
    ax.plot(wa_xs, wa_ys, 'y-', linewidth=3, label='WA* corridor')
    ax.plot(*START, 'go', markersize=12, label='Start')
    ax.plot(*GOAL,  'b*', markersize=12, label='Goal')
    ax.legend()

    plt.tight_layout()
    plt.show()

    print("✅ sample_state() working")