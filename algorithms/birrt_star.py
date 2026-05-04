# algorithms/birrt_star.py

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mock_data import (MAP_SIZE, OBSTACLES, START, GOAL,
                       GOAL_BIAS, CLEARANCE, STEP_SIZE, REWIRE_RADIUS)

# Import helpers from rrt.py
from rrt import (random_sample, get_nearest, steer,
                 is_collision, is_collision_free_path,
                 add_node, extract_path, animate_exploration)

# Import connection and merge from birrt.py
from birrt import try_connect_tree, merge_path


def get_all_nearby(tree, pos, radius):
    """
    Find all nodes within radius meters of pos.

    tree: list of nodes
    pos: (x, y) — center point to search around
    radius: float — search radius in meters
    returns: list of all nodes within radius
    """
    nearby = []
    radius_sq = radius ** 2
    for node in tree:
        dx = node["pos"][0] - pos[0]
        dy = node["pos"][1] - pos[1]
        dist_sq = dx**2 + dy**2

        if dist_sq <= radius_sq:
            nearby.append(node)

    return nearby


def choose_parent(new_pos, nearest_node, nearby_nodes, obstacles):
    """
    Find best parent for new_pos among nearby nodes.
    Best = gives new_pos lowest total cost from root.

    new_pos: (x, y) — position of new node being added
    nearest_node: dict — default parent (nearest node)
    nearby_nodes: list of nodes within rewire_radius
    obstacles: list of (x, y, radius, vx, vy)
    returns: best parent node dict
    """
    # Start with nearest as best parent
    best_parent = nearest_node
    dx = new_pos[0] - nearest_node["pos"][0]
    dy = new_pos[1] - nearest_node["pos"][1]
    best_cost = nearest_node["cost"] + np.sqrt(dx**2 + dy**2)

    # Check if any nearby node gives a cheaper path
    for node in nearby_nodes:
        dx = new_pos[0] - node["pos"][0]
        dy = new_pos[1] - node["pos"][1]
        dist = np.sqrt(dx**2 + dy**2)
        cost_from_node = node["cost"] + dist

        # Is this cheaper than our current best?
        if cost_from_node < best_cost:
            # Is the path from this node to new_pos clear?
            if is_collision_free_path(node["pos"], new_pos, obstacles):
                best_cost = cost_from_node
                best_parent = node

    return best_parent


def rewire(tree, new_node, nearby_nodes, obstacles):
    """
    Check if any nearby node is cheaper to reach through new_node.
    If yes — update that node's parent and cost.

    new_node:  the node just added
    nearby_nodes: list of nodes within rewire_radius of new_node
    """
    for node in nearby_nodes:

        # Never rewire back to new_node's own parent else its a cycle
        if node is new_node["parent"]:
            continue

        # What would this node cost if reached through new_node?
        dx = node["pos"][0] - new_node["pos"][0]
        dy = node["pos"][1] - new_node["pos"][1]
        dist = np.sqrt(dx**2 + dy**2)
        cost_from_new = new_node["cost"] + dist

        # Is that cheaper than its current cost?
        if cost_from_new < node["cost"]:

            # Is the path from new_node to this node clear?
            if is_collision_free_path(new_node["pos"], node["pos"], obstacles):
                # Rewire — update parent and cost
                node["parent"] = new_node
                node["cost"] = cost_from_new

                # Update all descendants of this node
                propagate_cost(node, tree)


def propagate_cost(node, tree):
    """
    After rewiring a node, update costs of all its descendants.
    Walks entire tree to find children of node.

    node: the rewired node whose children need updating
    tree: full list of all nodes
    """
    for n in tree:
        # Is n a direct child of node?
        if n["parent"] is node:
            # Recalculate n's cost based on updated parent cost
            dx = n["pos"][0] - node["pos"][0]
            dy = n["pos"][1] - node["pos"][1]
            dist = np.sqrt(dx**2 + dy**2)
            n["cost"] = node["cost"] + dist

            # Recursively update n's children 
            propagate_cost(n, tree)


def birrt_star(start, goal, obstacles, map_size,
               max_iter=3000, step_size=0.5,
               sigma=1.5, goal_bias=0.15,
               rewire_radius=1.0):
    """
    Same as birrt() but adds choose_parent() and rewire() to improve path quality

    step_size: meters per step
    sigma: tree connection distance threshold
    goal_bias: probability of sampling goal/start
    rewire_radius: radius to search for rewiring candidates
    """

    # Initialize both trees
    f_root  = {"pos": start, "parent": None, "cost": 0.0}
    r_root  = {"pos": goal,  "parent": None, "cost": 0.0}
    fw_tree = [f_root]
    rv_tree = [r_root]

    for i in range(max_iter):

        # Grow forward tree -------------------------------------------

        # Sample
        if np.random.random() < goal_bias:
            fw_sample = goal
        else:
            fw_sample = random_sample(map_size)
        # Nearest
        fw_nearest = get_nearest(fw_tree, fw_sample)
        # Steer
        fw_new_pos = steer(fw_nearest["pos"], fw_sample, step_size)
        # Collision check
        if is_collision(fw_new_pos, obstacles):
            continue
        if not is_collision_free_path(fw_nearest["pos"],fw_new_pos, obstacles):
            continue

        # RRT* — find best parent among nearby nodes
        fw_nearby  = get_all_nearby(fw_tree, fw_new_pos, rewire_radius)
        fw_parent  = choose_parent(fw_new_pos, fw_nearest,fw_nearby, obstacles)

        # Add with best parent
        fw_new_node = add_node(fw_tree, fw_new_pos, fw_parent)

        # RRT* — rewire nearby nodes through new node
        rewire(fw_tree, fw_new_node, fw_nearby, obstacles)


        #  Grow reverse tree --------------------------------------------
        # Sample — bias toward start
        if np.random.random() < goal_bias:
            rv_sample = start
        else:
            rv_sample = random_sample(map_size)
        # Nearest
        rv_nearest = get_nearest(rv_tree, rv_sample)
        # Steer
        rv_new_pos = steer(rv_nearest["pos"], rv_sample, step_size)
        # Collision check
        if is_collision(rv_new_pos, obstacles):
            continue
        if not is_collision_free_path(rv_nearest["pos"],rv_new_pos, obstacles):
            continue

        # RRT* — find best parent
        rv_nearby   = get_all_nearby(rv_tree, rv_new_pos, rewire_radius)
        rv_parent   = choose_parent(rv_new_pos, rv_nearest,rv_nearby, obstacles)

        # Add with best parent
        rv_new_node = add_node(rv_tree, rv_new_pos, rv_parent)

        # RRT* — rewire
        rewire(rv_tree, rv_new_node, rv_nearby, obstacles)

        # Check Connection ------------------------------------------------
        f_node, r_node = try_connect_tree(fw_tree, rv_tree,sigma, obstacles)

        if f_node is not None:
            path = merge_path(f_node, r_node)
            print(f" Path found in {i+1} iterations")
            print(f" Forward tree: {len(fw_tree)} nodes")
            print(f" Reverse tree: {len(rv_tree)} nodes")
            print(f" Path length: {len(path)} waypoints")
            return path, fw_tree, rv_tree

    print(f"❌ No path found after {max_iter} iterations")
    return None, fw_tree, rv_tree


if __name__ == "__main__":

    print("=" * 50)
    print("Running Bi-RRT*...")
    print(f"Start:         {START}")
    print(f"Goal:          {GOAL}")
    print(f"Obstacles:     {len(OBSTACLES)}")
    print(f"Clearance:     {CLEARANCE}m")
    print(f"Step size:     {STEP_SIZE}m")
    print(f"Rewire radius: {REWIRE_RADIUS}m")
    print("=" * 50)

    path, fw_tree, rv_tree = birrt_star(
        start = START,
        goal = GOAL,
        obstacles = OBSTACLES,
        map_size = MAP_SIZE,
        max_iter = 3000,
        step_size = STEP_SIZE,
        sigma = 1.5,
        goal_bias = GOAL_BIAS,
        rewire_radius = REWIRE_RADIUS
    )

    if path is not None:
        total_length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += np.sqrt(dx**2 + dy**2)
        print(f"Total path length: {total_length:.2f}m")

    # Animate
    animate_exploration(
        forward_tree = fw_tree,
        obstacles = OBSTACLES,
        map_size  = MAP_SIZE,
        start = START,
        goal = GOAL,
        path = path,
        reverse_tree = rv_tree,
        interval = 10,
        title = "Bi-RRT* Exploration\n""Blue=forward  Green=reverse  Red=path"
    )