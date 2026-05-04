import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mock_data import MAP_SIZE, OBSTACLES, START, GOAL, GOAL_BIAS, CLEARANCE, STEP_SIZE

# Import all helper functions from rrt.py — no need to rewrite them
from rrt import (random_sample, get_nearest, steer,
                 is_collision, is_collision_free_path,
                 add_node, extract_path, animate_exploration)


def try_connect_tree(fw_tree, rv_tree, sigma, obstacles):
    """
    Check if any node in forward_tree is close enough to
    any node in reverse_tree to connect them.

    forward_tree: list of nodes growing from start
    reverse_tree: list of nodes growing from goal
    sigma: float — max distance to attempt connection
    obstacles: list of (x, y, radius, vx, vy) tuples
    returns: (forward_node, reverse_node) if connected (None, None) if not connected
    """
    best_dist      = sigma  # only connect if within sigma
    best_fw_node    = None
    best_rv_node    = None

    for fw_node in fw_tree:
        for rv_node in rv_tree:
            dx   = fw_node["pos"][0] - rv_node["pos"][0]
            dy   = fw_node["pos"][1] - rv_node["pos"][1]
            dist = np.sqrt(dx**2 + dy**2)

            # Is this the closest pair we've found so far?
            if dist < best_dist:
                # Is path between them clear?
                if is_collision_free_path(fw_node["pos"],rv_node["pos"],obstacles):
                    best_dist   = dist
                    best_fw_node = fw_node
                    best_rv_node = rv_node

    return best_fw_node, best_rv_node


def merge_path(forward_node, reverse_node):
    """
    Merge forward and reverse tree paths into one complete path.

    forward_node: node in forward tree at connection point
    reverse_node: node in reverse tree at connection point
    returns:      complete path from start to goal as list of (x,y)
    """
    # Get path from start → forward connection point
    forward_path = extract_path(forward_node)

    # Get path from goal → reverse connection point and flip
    reverse_path = extract_path(reverse_node)
    reverse_path.reverse()   # now goes connection → goal

    # Stitch together the two tgt
    full_path = forward_path + reverse_path
    return full_path


def birrt(start, goal, obstacles, map_size,
          max_iter=2000, step_size=0.5,
          sigma=1.0, goal_bias=0.15):
    """
    Bidirectional RRT — grows two trees simultaneously
    from start and goal until they connect.

    start: (x, y) — starting position
    goal: (x, y) — goal position
    obstacles: list of (x, y, radius, vx, vy) tuples
    map_size:  (width, height) in meters
    max_iter:  max iterations before giving up
    step_size: how far to move each step
    sigma:  connection distance threshold
    goal_bias: probability of sampling goal/start directly
    returns:   (path, forward_tree, reverse_tree)
    """

    # Initialize both trees
    f_root = {"pos": start, "parent": None, "cost": 0.0}
    r_root = {"pos": goal,  "parent": None, "cost": 0.0}

    fw_tree  = [f_root]
    rv_tree  = [r_root]

    # Main loop
    for i in range(max_iter):

        # Grow Forward Tree -------------
        # Step 1 : Sample 
        if np.random.random() < goal_bias:
            fw_sample = goal
        else:
            fw_sample = random_sample(map_size)
        # Step 2 : Nearest
        fw_nearest = get_nearest(fw_tree, fw_sample)
        # Step 3 : Steer
        fw_new_pos = steer(fw_nearest["pos"], fw_sample, step_size)
        # Step 4 : Check obstacle and path clear
        if is_collision(fw_new_pos, obstacles):
            continue  # skip this sample, try again
        if not is_collision_free_path(fw_nearest["pos"], fw_new_pos, obstacles):
            continue  # path is blocked, try again
        add_node(fw_tree, fw_new_pos, fw_nearest)


        # Grow Reverse Tree -------------
        if np.random.random() < goal_bias:
            rv_sample = start
        else:
            rv_sample = random_sample(map_size)
        rv_nearest = get_nearest(rv_tree, rv_sample)
        rv_new_pos = steer(rv_nearest["pos"], rv_sample, step_size)
        if is_collision(rv_new_pos, obstacles):
            continue  
        if not is_collision_free_path(rv_nearest["pos"], rv_new_pos, obstacles):
            continue  
        add_node(rv_tree, rv_new_pos, rv_nearest)

        # Check connection btw two ----------------
        f_node, r_node = try_connect_tree(fw_tree, rv_tree, sigma, obstacles) 

        if f_node is not None:
            # Trees connected — merge into full path
            path = merge_path(f_node, r_node)
            print(f"Path found in {i+1} iterations")
            print(f"Forward tree: {len(fw_tree)} nodes")
            print(f"Reverse tree: {len(rv_tree)} nodes")
            print(f"Path length:  {len(path)} waypoints")
            return path, fw_tree, rv_tree

    print(f"❌ No path found after {max_iter} iterations")
    return None, fw_tree, rv_tree


if __name__ == "__main__":

    print("=" * 50)
    print("Running Bidirectional RRT...")
    print(f"Start:     {START}")
    print(f"Goal:      {GOAL}")
    print(f"Obstacles: {len(OBSTACLES)}")
    print(f"Clearance: {CLEARANCE}m")
    print(f"Step size: {STEP_SIZE}m")
    print("=" * 50)

    path, fw_tree, rv_tree = birrt(
        start     = START,
        goal      = GOAL,
        obstacles = OBSTACLES,
        map_size  = MAP_SIZE,
        max_iter  = 3000,
        step_size = STEP_SIZE,
        sigma     = 1.5,
        goal_bias = GOAL_BIAS
    )

    if path is not None:
        total_length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += np.sqrt(dx**2 + dy**2)
        print(f"Total path length: {total_length:.2f}m")

    # Animate — pass BOTH trees this time
    animate_exploration(
        forward_tree  = fw_tree,
        obstacles  = OBSTACLES,
        map_size  = MAP_SIZE,
        start  = START,
        goal  = GOAL,
        path  = path,
        reverse_tree  = rv_tree, # ← pass reverse tree
        interval  = 10,
        title  = "Bidirectional RRT Exploration\n"
                        "Blue=forward  Green=reverse  Red=path"
    )