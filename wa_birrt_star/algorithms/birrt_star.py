# algorithms/birrt_star.py
import sys
import os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_data import MAP_SIZE, SIGMA, CLEARANCE, STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS, START, GOAL, OBSTACLES
from rrt import (random_sample, get_nearest, steer,
                 is_collision, is_collision_free_path,
                 add_node, extract_path, animate_exploration)
from birrt import try_connect_tree, merge_path
from sampler import sample_state, compute_progress
from smoother import smooth_path


def get_all_nearby(tree, pos, radius):
    nearby = []
    radius_sq = radius ** 2
    for node in tree:
        dx = node["pos"][0] - pos[0]
        dy = node["pos"][1] - pos[1]
        if dx**2 + dy**2 <= radius_sq:
            nearby.append(node)
    return nearby


def choose_parent(new_pos, nearest_node, nearby_nodes, obstacles):
    best_parent = nearest_node
    dx = new_pos[0] - nearest_node["pos"][0]
    dy = new_pos[1] - nearest_node["pos"][1]
    best_cost = nearest_node["cost"] + np.sqrt(dx**2 + dy**2)

    for node in nearby_nodes:
        dx = new_pos[0] - node["pos"][0]
        dy = new_pos[1] - node["pos"][1]
        dist = np.sqrt(dx**2 + dy**2)
        cost_from_node = node["cost"] + dist
        if cost_from_node < best_cost:
            if is_collision_free_path(node["pos"], new_pos, obstacles, clearance=0.0):
                best_cost = cost_from_node
                best_parent = node
    return best_parent


def rewire(tree, new_node, nearby_nodes, obstacles):
    for node in nearby_nodes:
        if node is new_node["parent"]:
            continue
        dx = node["pos"][0] - new_node["pos"][0]
        dy = node["pos"][1] - new_node["pos"][1]
        dist = np.sqrt(dx**2 + dy**2)
        cost_from_new = new_node["cost"] + dist
        if cost_from_new < node["cost"]:
            if is_collision_free_path(new_node["pos"], node["pos"], obstacles, clearance=0.0):
                node["parent"] = new_node
                node["cost"]   = cost_from_new
                propagate_cost(node, tree)


def propagate_cost(node, tree):
    for n in tree:
        if n["parent"] is node:
            dx = n["pos"][0] - node["pos"][0]
            dy = n["pos"][1] - node["pos"][1]
            n["cost"] = node["cost"] + np.sqrt(dx**2 + dy**2)
            propagate_cost(n, tree)


def birrt_star(start, goal, obstacles, map_size,
               max_iter=5000,
               step_size=0.15,
               sigma=0.3,
               goal_bias=0.10,
               rewire_radius=0.45,
               wa_path=None):
    """
    Bi-RRT* with pre-inflated obstacles.
    All collision checks use clearance=0.0 — margin is baked into obstacle
    radii by get_planning_obstacles() in planner_node before being passed here.

    FIXES applied:
      1. choose_parent: was referencing new_node before it existed
      2. reverse tree: was checking fw_ variables instead of rv_
      3. rewire: was missing clearance=0.0
      4. try_connect_tree: now receives clearance=0.0 — KEY FIX for static
         obstacle crashes, connection segment was double-counting CLEARANCE
    """
    f_root  = {"pos": start, "parent": None, "cost": 0.0}
    r_root  = {"pos": goal,  "parent": None, "cost": 0.0}
    fw_tree = [f_root]
    rv_tree = [r_root]

    for i in range(max_iter):

        # ── Forward tree ──────────────────────────────────────────────────────
        progress  = compute_progress(fw_tree, goal)
        fw_sample = sample_state(
            tree=fw_tree, goal=goal, obstacles=obstacles,
            wa_path=wa_path, map_size=map_size,
            corridor_width=1.2, progress=progress
        )
        fw_nearest = get_nearest(fw_tree, fw_sample)
        fw_new_pos = steer(fw_nearest["pos"], fw_sample, step_size)

        if is_collision(fw_new_pos, obstacles, clearance=0.0):
            continue
        if not is_collision_free_path(fw_nearest["pos"], fw_new_pos, obstacles, clearance=0.0):
            continue

        fw_nearby   = get_all_nearby(fw_tree, fw_new_pos, rewire_radius)
        fw_parent   = choose_parent(fw_new_pos, fw_nearest, fw_nearby, obstacles)
        fw_new_node = add_node(fw_tree, fw_new_pos, fw_parent)
        rewire(fw_tree, fw_new_node, fw_nearby, obstacles)

        # ── Reverse tree ──────────────────────────────────────────────────────
        rv_progress = compute_progress(rv_tree, start)
        rv_sample   = sample_state(
            tree=rv_tree, goal=start, obstacles=obstacles,
            wa_path=wa_path, map_size=map_size,
            corridor_width=1.2, progress=rv_progress
        )
        rv_nearest = get_nearest(rv_tree, rv_sample)
        rv_new_pos = steer(rv_nearest["pos"], rv_sample, step_size)

        if is_collision(rv_new_pos, obstacles, clearance=0.0):
            continue
        if not is_collision_free_path(rv_nearest["pos"], rv_new_pos, obstacles, clearance=0.0):
            continue

        rv_nearby   = get_all_nearby(rv_tree, rv_new_pos, rewire_radius)
        rv_parent   = choose_parent(rv_new_pos, rv_nearest, rv_nearby, obstacles)
        rv_new_node = add_node(rv_tree, rv_new_pos, rv_parent)
        rewire(rv_tree, rv_new_node, rv_nearby, obstacles)

        # ── Connect — clearance=0.0 matches planning world ────────────────────
        f_node, r_node = try_connect_tree(fw_tree, rv_tree, sigma, obstacles,
                                          clearance=0.0)
        if f_node is not None:
            path = merge_path(f_node, r_node)
            raw_length = len(path)
            #path = smooth_path(path, obstacles)
            print(f" Path found in {i+1} iterations")
            print(f" Forward tree: {len(fw_tree)} nodes")
            print(f" Reverse tree: {len(rv_tree)} nodes")
            print(f" Raw path:     {raw_length} waypoints")
            print(f" Smoothed:     {len(path)} waypoints")

            return path, fw_tree, rv_tree

    print(f"No path found after {max_iter} iterations")
    return None, fw_tree, rv_tree


if __name__ == "__main__":
    print("=" * 50)
    print("Running Bi-RRT* standalone...")
    print(f"Start: {START}  Goal: {GOAL}")
    print(f"Obstacles: {len(OBSTACLES)}  Clearance: {CLEARANCE}m")
    print("=" * 50)

    # Standalone: bake CLEARANCE into radii so clearance=0.0 works correctly
    standalone_obs = [(x, y, r + CLEARANCE, vx, vy) for x, y, r, vx, vy in OBSTACLES]

    path, fw_tree, rv_tree = birrt_star(
        start=START, goal=GOAL, obstacles=standalone_obs,
        map_size=MAP_SIZE, max_iter=5000, step_size=STEP_SIZE,
        sigma=SIGMA, goal_bias=GOAL_BIAS, rewire_radius=REWIRE_RADIUS,
        wa_path=None
    )

    if path is not None:
        total = sum(
            np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            for i in range(1, len(path))
        )
        print(f"Total path length: {total:.2f}m")

    animate_exploration(
        forward_tree=fw_tree, obstacles=OBSTACLES, map_size=MAP_SIZE,
        start=START, goal=GOAL, path=path, reverse_tree=rv_tree,
        interval=10, title="Bi-RRT*\nBlue=forward  Green=reverse  Red=path"
    )