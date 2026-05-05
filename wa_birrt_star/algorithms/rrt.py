import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_data import MAP_SIZE, CLEARANCE, OBSTACLES, START, GOAL, GOAL_BIAS

def random_sample(map_size):
    "pick rand point inside map size 20*20m"

    x = np.random.uniform(0, map_size[0])
    y = np.random.uniform(0, map_size[1])
    return (x, y)

def get_nearest(tree, point):
    """
    Find the closest node in the tree to a selected point x,y.
    
    tree:   list of nodes — each node is a dict with "pos", "parent", "cost"
    returns: the nearest node dict
    """
    nearest_node = None
    nearest_dist_sq = float('inf')  # start with infinity so any real distance beats it

    for node in tree:
        dx = node["pos"][0] - point[0]  # difference in x
        dy = node["pos"][1] - point[1]  # difference in y
        dist_sq = dx**2 + dy**2  # Euclidean distance

        if dist_sq < nearest_dist_sq:
            nearest_dist_sq = dist_sq
            nearest_node = node

    return nearest_node


def steer(from_pos, to_pos, step_size):
    """
    Move step_size meters from from_pos toward to_pos.
    If to_pos is closer than step_size, just return to_pos.

    from_pos:  (x, y) — starting position (nearest node)
    to_pos:    (x, y) — target position (random sample)
    step_size: float  — max distance to move in one step
    returns:   (x, y) — new position
    """
    dx = to_pos[0] - from_pos[0]   # x difference
    dy = to_pos[1] - from_pos[1]   # y difference
    dist = np.sqrt(dx**2 + dy**2)  # actual distance between points

    # If we're already closer than step_size just go directly there
    if dist < step_size:
        return to_pos

    # Normalize direction to length 1 then scale by step_size
    # This gives us a step exactly step_size meters long
    ratio = step_size / dist
    new_x = from_pos[0] + dx * ratio
    new_y = from_pos[1] + dy * ratio

    return (new_x, new_y)

def is_collision(pos, obstacles):
    """
    Check if a single point is inside any obstacle.

    pos:       (x, y) — the point to check
    obstacles: list of (x, y, radius, vx, vy) tuples
    clerance: robot radius + safety margin (meter)
    returns:   True if collision, False if free
    """
    px, py = pos

    # Check map boundaries — stay clearance meters away from walls
    if (px < CLEARANCE or px > MAP_SIZE[0] - CLEARANCE or
        py < CLEARANCE or py > MAP_SIZE[1] - CLEARANCE):
        return True

    # Check obstacles with inflation
    for obs in obstacles:
        ox, oy, o_radius, vx, vy = obs

        # Distance from point to obstacle center
        dx   = px - ox
        dy   = py - oy
        dist = np.sqrt(dx**2 + dy**2)

        if dist < o_radius + CLEARANCE:
            return True

    # Point is outside all obstacles → free
    return False

def is_collision_free_path(pos_a, pos_b, obstacles, num_checks=20):
    """
    Check if the straight line between two points is clear of all obstacles.
    Samples num_checks points along the line and checks each one.

    pos_a:      (x, y) — start of line
    pos_b:      (x, y) — end of line
    obstacles:  list of (x, y, radius, vx, vy) tuples
    num_checks: how many points to sample along the line
    returns:    True if path is clear, False if blocked
    """
    for i in range(num_checks + 1):
        # t goes from 0.0 to 1.0 along the line
        t = i / num_checks

        # Interpolate between pos_a and pos_b
        x = pos_a[0] + t * (pos_b[0] - pos_a[0])
        y = pos_a[1] + t * (pos_b[1] - pos_a[1])

        # Check if this point hits any obstacle
        if is_collision((x, y), obstacles):
            return False  # blocked

    return True  # all points clear

def add_node(tree, pos, parent_node):
    """
    Create a new node and add it to the tree.

    tree:        list of nodes — the tree to add to
    pos:         (x, y) — position of new node
    parent_node: dict — the parent node
    returns:     the new node dict
    """
    # Calculate cost — distance from root to this new node
    # cost = parent's cost + distance from parent to this node
    dx   = pos[0] - parent_node["pos"][0]
    dy   = pos[1] - parent_node["pos"][1]
    dist = np.sqrt(dx**2 + dy**2)
    cost = parent_node["cost"] + dist

    # Create the new node
    new_node = {
        "pos":    pos,
        "parent": parent_node,
        "cost":   cost
    }
    # Add to tree
    tree.append(new_node)
    return new_node


def extract_path(goal_node):
    """
    Trace back through parent pointers from goal to root.
    Returns the full path from start to goal.

    goal_node: dict — the node closest to the goal
    returns:   list of (x,y) points from start to goal
    """
    path = []
    current = goal_node

    # Walk backwards through parent pointers until we hit root
    while current is not None:
        path.append(current["pos"])
        current = current["parent"]

    # Path is currently goal → start, reverse it
    path.reverse()

    return path

def rrt(start, goal, obstacles, map_size,
        max_iter=2000, step_size=0.5, goal_radius=0.5, goal_bias=0.15):
    """
    Main RRT loop. Grows a tree from start until it reaches goal.

    start:       (x, y) — starting position
    goal:        (x, y) — goal position
    obstacles:   list of (x, y, radius, vx, vy) tuples
    map_size:    (width, height) in meters
    max_iter:    maximum number of iterations before giving up
    step_size:   how far to move each step in meters
    goal_radius: how close we need to get to the goal to stop
    goal_bias:   probability of sampling the goal directly
    returns:     path as list of (x,y) points, or None if failed
    """

    # ── Initialize ────────────────────────────────────────────
    # Start with a single node at the start position
    root = {"pos": start, "parent": None, "cost": 0.0}
    tree = [root]

    # For visualization later — store all attempted steps
    all_new_positions = []

    # ── Main Loop ─────────────────────────────────────────────
    for i in range(max_iter):

        # Step 1 — SAMPLE
        # With goal_bias probability, sample the goal directly
        # Otherwise sample a random point
        if np.random.random() < goal_bias:
            sample = goal
        else:
            sample = random_sample(map_size)

        # Step 2 — NEAREST
        nearest = get_nearest(tree, sample)

        # Step 3 — STEER
        new_pos = steer(nearest["pos"], sample, step_size)

        # Step 4 — CHECK not in obstacle and path clear
        if is_collision(new_pos, obstacles):
            continue  # skip this sample, try again
        if not is_collision_free_path(nearest["pos"], new_pos, obstacles):
            continue  # path is blocked, try again

        # Step 5 — ADD
        new_node = add_node(tree, new_pos, nearest)
        all_new_positions.append(new_pos)

        # Step 6 — CHECK GOAL
        dx   = new_pos[0] - goal[0]
        dy   = new_pos[1] - goal[1]
        dist = np.sqrt(dx**2 + dy**2)

        if dist < goal_radius:
            # Check if we can cleanly reach the exact goal position
            if is_collision_free_path(new_pos, goal, obstacles):
                goal_node = add_node(tree, goal, new_node)
                path = extract_path(goal_node)
            else:
                # Close enough but can't reach exact goal — use current node
                path = extract_path(new_node)

            print(f"Path found in {i+1} iterations")
            print(f"Nodes in tree: {len(tree)}")
            print(f"Path length:   {len(path)} waypoints")
            return path, tree
        
    # If we get here we never found a path
    print(f"❌ No path found after {max_iter} iterations")
    return None, tree

def animate_exploration(forward_tree, obstacles, map_size,
                        start, goal, path,
                        reverse_tree=None,
                        interval=20, title="RRT Exploration"):
    """
    Animate the tree exploration process.
    Works for both vanilla RRT (one tree) and Bi-RRT (two trees).

    forward_tree:  list of nodes — forward tree (or only tree for RRT)
    obstacles:     list of (x, y, radius, vx, vy)
    map_size:      (width, height) in meters
    start:         (x, y)
    goal:          (x, y)
    path:          list of (x,y) — final solution path
    reverse_tree:  list of nodes — reverse tree (None for vanilla RRT)
    interval:      milliseconds between frames (lower = faster)
    title:         plot title string
    """
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    # ── Draw Static Elements ───────────────────────────────────

    # Obstacles + clearance zones
    for obs in obstacles:
        circle = plt.Circle((obs[0], obs[1]), obs[2],
                             color='black', alpha=0.8, zorder=3)
        ax.add_patch(circle)
        clearance_ring = plt.Circle((obs[0], obs[1]), obs[2] + CLEARANCE,
                                     color='red', fill=False,
                                     linestyle='--', linewidth=1.0,
                                     alpha=0.5, zorder=3)
        ax.add_patch(clearance_ring)

    # Boundary clearance
    boundary = plt.Rectangle(
        (CLEARANCE, CLEARANCE),
        map_size[0] - 2 * CLEARANCE,
        map_size[1] - 2 * CLEARANCE,
        color='red', fill=False,
        linestyle='--', linewidth=1.0, alpha=0.5
    )
    ax.add_patch(boundary)

    # Start and goal markers
    ax.plot(*start, 'go', markersize=15, label='Start', zorder=6)
    ax.plot(*goal,  'b*', markersize=15, label='Goal',  zorder=6)

    # ── Animation State ────────────────────────────────────────

    # Collect all forward tree edges in order they were added
    # skip root (index 0) since it has no parent
    fw_edges = []
    for node in forward_tree[1:]:
        fw_edges.append((node["pos"], node["parent"]["pos"]))

    # Collect reverse tree edges if provided
    rv_edges = []
    if reverse_tree is not None:
        for node in reverse_tree[1:]:
            rv_edges.append((node["pos"], node["parent"]["pos"]))

    # Total frames = max of both tree sizes
    total_frames = max(len(fw_edges),
                       len(rv_edges) if rv_edges else 0)

    # Lines that get drawn each frame
    fw_lines = []
    rv_lines = []
    path_line, = ax.plot([], [], 'r-', linewidth=3,
                          label='Path', zorder=5)

    # ── Update Function ────────────────────────────────────────

    def update(frame):
        # Draw forward tree edge for this frame
        if frame < len(fw_edges):
            p1, p2 = fw_edges[frame]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                             '-', color='blue',
                             linewidth=0.5, alpha=0.6, zorder=2)
            fw_lines.append(line)

        # Draw reverse tree edge for this frame
        if rv_edges and frame < len(rv_edges):
            p1, p2 = rv_edges[frame]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                             '-', color='green',
                             linewidth=0.5, alpha=0.6, zorder=2)
            rv_lines.append(line)

        # On the last frame draw the final path
        if frame == total_frames - 1 and path is not None:
            path_xs = [p[0] for p in path]
            path_ys = [p[1] for p in path]
            path_line.set_data(path_xs, path_ys)

        return fw_lines + rv_lines + [path_line]

    # ── Run Animation ──────────────────────────────────────────
    anim = FuncAnimation(fig, update,
                         frames=total_frames,
                         interval=interval,
                         blit=True,
                         repeat=False)

    # Legend
    ax.legend(fontsize=12)
    plt.tight_layout()
    plt.show()

    return anim   # return so caller can save it if needed




if __name__ == "__main__":
    from wa_birrt_star.algorithms.mock_data import STEP_SIZE

    print("=" * 50)
    print("Running Vanilla RRT...")
    print(f"Start:     {START}")
    print(f"Goal:      {GOAL}")
    print(f"Obstacles: {len(OBSTACLES)}")
    print(f"Clearance: {CLEARANCE}m")
    print(f"Step size: {STEP_SIZE}m")
    print("=" * 50)

    path, tree = rrt(
        start       = START,
        goal        = GOAL,
        obstacles   = OBSTACLES,
        map_size    = MAP_SIZE,
        max_iter    = 5000,
        step_size   = STEP_SIZE,
        goal_radius = 0.5,
        goal_bias   = GOAL_BIAS
    )

    if path is not None:
        total_length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += np.sqrt(dx**2 + dy**2)
        print(f"   Total path length: {total_length:.2f}m")
        print(f"   Tree size:         {len(tree)} nodes")
    else:
        print("No path found")

    # Animate exploration
    animate_exploration(
        forward_tree  = tree,
        obstacles     = OBSTACLES,
        map_size      = MAP_SIZE,
        start         = START,
        goal          = GOAL,
        path          = path,
        reverse_tree  = None,       # None = vanilla RRT
        interval      = 10,         # ms between frames — lower = faster
        title         = "Vanilla RRT Exploration\n"
                        "Blue=tree  Red=path"
    )