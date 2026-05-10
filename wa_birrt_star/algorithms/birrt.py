"""
birrt.py — Bidirectional RRT (Bi-RRT)
  Grow TWO trees simultaneously — one from start, one from goal.
  Every iteration we check if the two trees are close enough to connect.

Two helper functions are also defined here and reused by Bi-RRT*:
  try_connect_tree() — check if the two trees can be joined  |  merge_path()— stitch the two partial paths into one complete path
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_data import MAP_SIZE, CLEARANCE, STEP_SIZE, GOAL_BIAS, START, GOAL, OBSTACLES
from rrt import (random_sample, get_nearest, steer,
                 is_collision, is_collision_free_path,
                 add_node, extract_path, animate_exploration)


# ── Tree connection ────────────────────────────────────────────────────────────

def try_connect_tree(fw_tree, rv_tree, sigma, obstacles, clearance=None):
    """
    Check if any node in the forward tree is close enough to any node
    in the reverse tree to bridge the gap between them.

    sigma:     maximum allowed distance between the two nodes to connect
    clearance: pass 0.0 when obstacles are already inflated (live planner)
               pass None for standalone use with raw obstacles
    """
    best_dist    = sigma      # only connect if closer than this
    best_fw_node = None
    best_rv_node = None

    for fw_node in fw_tree:
        for rv_node in rv_tree:
            dx   = fw_node["pos"][0] - rv_node["pos"][0]
            dy   = fw_node["pos"][1] - rv_node["pos"][1]
            dist = np.sqrt(dx**2 + dy**2)

            if dist < best_dist:
                # Verify the straight line between them is actually clear
                if is_collision_free_path(fw_node["pos"], rv_node["pos"],
                                          obstacles, clearance=clearance):
                    best_dist    = dist
                    best_fw_node = fw_node
                    best_rv_node = rv_node

    return best_fw_node, best_rv_node


def merge_path(forward_node, reverse_node):
    """ Stitch the two partial paths into one complete start→goal path. """
    forward_path = extract_path(forward_node)   # start → forward_node
    reverse_path = extract_path(reverse_node)   # goal  → reverse_node
    reverse_path.reverse()                      # now:  reverse_node → goal

    # Bridge point between the two connection nodes
    fx, fy   = forward_node["pos"]
    rx, ry   = reverse_node["pos"]
    midpoint = ((fx + rx) / 2, (fy + ry) / 2)

    return forward_path + [midpoint] + reverse_path


# ── Main Bi-RRT algorithm ──────────────────────────────────────────────────────

def birrt(start, goal, obstacles, map_size,
          max_iter=5000,
          step_size=0.15,
          sigma=0.5,
          goal_bias=0.10):
    """ Grow two trees simultaneously — one from start, one from goal. Stop when close enough to connect. """
    # Initialize both trees with their root nodes
    f_root  = {"pos": start, "parent": None, "cost": 0.0}
    r_root  = {"pos": goal,  "parent": None, "cost": 0.0}
    fw_tree = [f_root]
    rv_tree = [r_root]

    for iteration in range(max_iter):

        # ── Extend forward tree (start → goal direction) ───────────────────
        fw_sample  = goal if np.random.random() < goal_bias else random_sample(map_size)
        fw_nearest = get_nearest(fw_tree, fw_sample)
        fw_new_pos = steer(fw_nearest["pos"], fw_sample, step_size)

        if is_collision(fw_new_pos, obstacles):
            continue
        if not is_collision_free_path(fw_nearest["pos"], fw_new_pos, obstacles):
            continue
        add_node(fw_tree, fw_new_pos, fw_nearest)

        # ── Extend reverse tree (goal → start direction) ───────────────────
        rv_sample  = start if np.random.random() < goal_bias else random_sample(map_size)
        rv_nearest = get_nearest(rv_tree, rv_sample)
        rv_new_pos = steer(rv_nearest["pos"], rv_sample, step_size)

        if is_collision(rv_new_pos, obstacles):
            continue
        if not is_collision_free_path(rv_nearest["pos"], rv_new_pos, obstacles):
            continue
        add_node(rv_tree, rv_new_pos, rv_nearest)

        # ── Try to connect the two trees ───────────────────────────────────
        f_node, r_node = try_connect_tree(fw_tree, rv_tree, sigma, obstacles)

        if f_node is not None:
            path = merge_path(f_node, r_node)
            print(f"Path found in {iteration + 1} iterations")
            print(f"Forward tree: {len(fw_tree)} nodes")
            print(f"Reverse tree: {len(rv_tree)} nodes")
            print(f"Waypoints:    {len(path)}")
            return path, fw_tree, rv_tree

    print(f"No path found after {max_iter} iterations")
    return None, fw_tree, rv_tree


# ── Standalone test ────────────────────────────────────────────────────────────

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
        total_length = sum(
            np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            for i in range(1, len(path))
        )
        print(f"Total path length: {total_length:.2f}m")

    animate_exploration(
        forward_tree = fw_tree,
        obstacles    = OBSTACLES,
        map_size     = MAP_SIZE,
        start        = START,
        goal         = GOAL,
        path         = path,
        reverse_tree = rv_tree,
        interval     = 10,
        title        = "Bidirectional RRT\nBlue=forward  Green=reverse  Red=path"
    )