"""
rrt.py — RRT (Rapidly-exploring Random Tree)
Pick rand point in map -> find cloest node we already have -> take step that node to the rand point -> add to tree -> repeat till goal
This file also contains helper functions shared by all RRT variants:
  - is_collision() : p inside obstacle |  is_collision_free_path()— straight line btw 2points clear? | animate_exploration() — visualize the tree 
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_data import MAP_SIZE, STEP_SIZE, CLEARANCE, OBSTACLES, START, GOAL, GOAL_BIAS


# ── Point sampling ─────────────────────────────────────────────────────────────

def random_sample(map_size):
    """Pick a completely random (x, y) point anywhere inside the map."""
    x = np.random.uniform(0, map_size[0])
    y = np.random.uniform(0, map_size[1])
    return (x, y)


# ── Tree operations ────────────────────────────────────────────────────────────

def get_nearest(tree, point):
    """
    Find the node in the tree that is closest to the given point.
    We search every node and keep track of the closest one found so far.
    """
    nearest_node    = None
    nearest_dist_sq = float('inf')

    for node in tree:
        dx      = node["pos"][0] - point[0]
        dy      = node["pos"][1] - point[1]
        dist_sq = dx**2 + dy**2

        if dist_sq < nearest_dist_sq:
            nearest_dist_sq = dist_sq
            nearest_node    = node

    return nearest_node


def steer(from_pos, to_pos, step_size):
    """
    Move from from_pos toward to_pos by at most step_size meters.

    Think of it like: "I want to go there, but I can only take one step."
    If the destination is closer than one step, just go there directly.
    """
    dx   = to_pos[0] - from_pos[0]
    dy   = to_pos[1] - from_pos[1]
    dist = np.sqrt(dx**2 + dy**2)

    # Already close enough — go directly
    if dist < step_size:
        return to_pos

    # Scale the direction vector to exactly step_size meters long
    scale = step_size / dist
    new_x = from_pos[0] + dx * scale
    new_y = from_pos[1] + dy * scale
    return (new_x, new_y)


def add_node(tree, pos, parent_node):
    """
    Create a new tree node at pos and attach it to parent_node.
    The node stores its position, who it came from, and total cost from root.
    """
    dist_from_parent = np.sqrt(
        (pos[0] - parent_node["pos"][0])**2 +
        (pos[1] - parent_node["pos"][1])**2
    )
    new_node = {
        "pos":    pos,
        "parent": parent_node,
        "cost":   parent_node["cost"] + dist_from_parent
    }
    tree.append(new_node)
    return new_node


def extract_path(goal_node):
    """
    Trace back from goal to start by following parent pointers.
    Returns the path in order: [start, ..., goal].
    """
    path    = []
    current = goal_node

    while current is not None:
        path.append(current["pos"])
        current = current["parent"]

    path.reverse()   # was goal→start, now start→goal
    return path


# ── Collision checking ─────────────────────────────────────────────────────────

def is_collision(pos, obstacles, clearance=None):
    """
    Check whether a point is inside any obstacle or too close to the map edge.

    clearance=None  → use the default CLEARANCE from mock_data
    clearance=0.0   → obstacles are already inflated, no extra margin needed
    """
    if clearance is None:
        clearance = CLEARANCE

    px, py = pos

    # Too close to the map boundary
    too_close_to_edge = (
        px < clearance or px > MAP_SIZE[0] - clearance or
        py < clearance or py > MAP_SIZE[1] - clearance
    )
    if too_close_to_edge:
        return True

    # Inside or too close to any obstacle
    for ox, oy, radius, vx, vy in obstacles:
        dist_to_center = np.sqrt((px - ox)**2 + (py - oy)**2)
        if dist_to_center < radius + clearance:
            return True

    return False   # point is safe


def is_collision_free_path(pos_a, pos_b, obstacles, clearance=None):
    """
    Check if a straight line from pos_a to pos_b is completely clear.

    We walk along the line in small steps and check each point.
    The step size is small enough that no obstacle can fit between checks.
    """
    if clearance is None:
        clearance = CLEARANCE

    dx   = pos_b[0] - pos_a[0]
    dy   = pos_b[1] - pos_a[1]
    dist = np.sqrt(dx**2 + dy**2)

    # Check every 0.10m (half the clearance) so nothing slips through
    check_step = CLEARANCE / 2.0
    num_checks = max(10, int(dist / check_step))

    for i in range(num_checks + 1):
        t = i / num_checks
        x = pos_a[0] + t * (pos_b[0] - pos_a[0])
        y = pos_a[1] + t * (pos_b[1] - pos_a[1])
        if is_collision((x, y), obstacles, clearance=clearance):
            return False   # hit something

    return True   # path is clear


# ── Main RRT algorithm ─────────────────────────────────────────────────────────

def rrt(start, goal, obstacles, map_size,
        max_iter=2000, step_size=0.5, goal_radius=0.5, goal_bias=0.15):
    """
    Grow a tree from start until it reaches the goal.

    Each iteration:
      - Sample a random point (or the goal directly with probability goal_bias)
      - Find the closest existing node
      - Take one step toward the sample
      - If the step is collision-free, add it to the tree
      - If we're close enough to the goal, we're done

    Returns the path and the full tree (for visualization).
    """
    # Start the tree with just the start position
    root = {"pos": start, "parent": None, "cost": 0.0}
    tree = [root]

    for iteration in range(max_iter):

        # ── Sample ────────────────────────────────────────────
        # Pull toward the goal occasionally to avoid aimless wandering
        if np.random.random() < goal_bias:
            sample = goal
        else:
            sample = random_sample(map_size)

        # ── Extend ────────────────────────────────────────────
        nearest = get_nearest(tree, sample)
        new_pos = steer(nearest["pos"], sample, step_size)

        # Skip if the new position or path to it hits something
        if is_collision(new_pos, obstacles):
            continue
        if not is_collision_free_path(nearest["pos"], new_pos, obstacles):
            continue

        # Safe — add to tree
        new_node = add_node(tree, new_pos, nearest)

        # ── Check goal ────────────────────────────────────────
        dist_to_goal = np.sqrt(
            (new_pos[0] - goal[0])**2 +
            (new_pos[1] - goal[1])**2
        )

        if dist_to_goal < goal_radius:
            # Try to connect cleanly to the exact goal position
            if is_collision_free_path(new_pos, goal, obstacles):
                goal_node = add_node(tree, goal, new_node)
                path      = extract_path(goal_node)
            else:
                # Close enough — use the current node as the end
                path = extract_path(new_node)

            print(f"Path found in {iteration + 1} iterations")
            print(f"Tree size:   {len(tree)} nodes")
            print(f"Waypoints:   {len(path)}")
            return path, tree

    print(f"No path found after {max_iter} iterations")
    return None, tree


# ── Visualization ──────────────────────────────────────────────────────────────

def animate_exploration(forward_tree, obstacles, map_size,
                        start, goal, path,
                        reverse_tree=None,
                        interval=20, title="RRT Exploration"):
    """
    Animate the tree growing node by node, then show the final path.

    Works for both single-tree (RRT, RRT*) and dual-tree (Bi-RRT, Bi-RRT*) algorithms.
    Forward tree drawn in blue, reverse tree in green, final path in red.
    """
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    # Draw obstacles and their safety clearance rings
    for obs in obstacles:
        ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2],
                                color='black', alpha=0.8, zorder=3))
        ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2] + CLEARANCE,
                                color='red', fill=False,
                                linestyle='--', linewidth=1.0,
                                alpha=0.5, zorder=3))

    # Draw the boundary clearance zone
    ax.add_patch(plt.Rectangle(
        (CLEARANCE, CLEARANCE),
        map_size[0] - 2*CLEARANCE, map_size[1] - 2*CLEARANCE,
        color='red', fill=False, linestyle='--', linewidth=1.0, alpha=0.5
    ))

    ax.plot(*start, 'go', markersize=15, label='Start', zorder=6)
    ax.plot(*goal,  'b*', markersize=15, label='Goal',  zorder=6)

    # Collect tree edges in the order they were added
    fw_edges = [(node["pos"], node["parent"]["pos"])
                for node in forward_tree[1:]]
    rv_edges = [(node["pos"], node["parent"]["pos"])
                for node in reverse_tree[1:]] if reverse_tree else []

    total_frames = max(len(fw_edges), len(rv_edges) if rv_edges else 0)

    fw_lines   = []
    rv_lines   = []
    path_line, = ax.plot([], [], 'r-', linewidth=3, label='Path', zorder=5)

    def update(frame):
        # Add one forward tree edge per frame
        if frame < len(fw_edges):
            p1, p2  = fw_edges[frame]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                            '-', color='blue', linewidth=0.5, alpha=0.6, zorder=2)
            fw_lines.append(line)

        # Add one reverse tree edge per frame (if bidirectional)
        if rv_edges and frame < len(rv_edges):
            p1, p2  = rv_edges[frame]
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                            '-', color='green', linewidth=0.5, alpha=0.6, zorder=2)
            rv_lines.append(line)

        # On the last frame, draw the final path
        if frame == total_frames - 1 and path is not None:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            path_line.set_data(xs, ys)

        return fw_lines + rv_lines + [path_line]

    anim = FuncAnimation(fig, update, frames=total_frames,
                         interval=interval, blit=True, repeat=False)
    ax.legend(fontsize=12)
    plt.tight_layout()
    plt.show()
    return anim


# ── Standalone test ────────────────────────────────────────────────────────────

if __name__ == "__main__":
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
        total_length = sum(
            np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            for i in range(1, len(path))
        )
        print(f"Total path length: {total_length:.2f}m")
        print(f"Tree size:         {len(tree)} nodes")
    else:
        print("No path found")

    animate_exploration(
        forward_tree = tree,
        obstacles    = OBSTACLES,
        map_size     = MAP_SIZE,
        start        = START,
        goal         = GOAL,
        path         = path,
        reverse_tree = None,
        interval     = 10,
        title        = "Vanilla RRT Exploration\nBlue=tree  Red=path"
    )