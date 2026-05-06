# algorithms/wa_birrt_star.py
# WA* + Bi-RRT* combined planner — minimal new code.

import heapq
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_data   import MAP_SIZE, SIGMA, CLEARANCE, STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS, START, GOAL, OBSTACLES
from rrt         import is_collision, animate_exploration
from birrt_star  import birrt_star

# ── Constants ─────────────────────────────────────────────────────────────────

CELL_SIZE  = 0.1    # 10cm — half your STEP_SIZE, guarantees no gap between checks
WA_EPSILON = 2.5    # >1 = faster but suboptimal — good for dynamic replanning

# 8-connected moves (d_row, d_col, cost)
_MOVES = [(-1,0,1),( 1,0,1),(0,-1,1),(0,1,1),
          (-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),
          ( 1,-1,math.sqrt(2)),( 1,1,math.sqrt(2))]

# ── Coordinate helpers (2 one-liners) ────────────────────────────────────────

def xy_to_cell(pos, cs=CELL_SIZE):
    return (int(pos[1]/cs), int(pos[0]/cs))

def cell_to_xy(cell, cs=CELL_SIZE):
    return (cell[1]*cs + cs/2, cell[0]*cs + cs/2)

# ── Neighbors — reuses is_collision, no new obstacle logic ───────────────────

def get_neighbors(cell, obstacles, map_size, cs=CELL_SIZE):
    """
    Yield valid 8-connected neighbors.
    Free/blocked check delegates entirely to is_collision() from rrt.py.
    Diagonal corner cuts blocked to prevent clipping obstacle corners.
    """
    r, c   = cell
    rows   = int(map_size[1] / cs)
    cols   = int(map_size[0] / cs)

    for dr, dc, cost in _MOVES:
        nr, nc = r+dr, c+dc
        if not (0 <= nr < rows and 0 <= nc < cols):
            continue
        if is_collision(cell_to_xy((nr,nc),cs), obstacles, clearance=0.0):
            continue
        # Block diagonal if either cardinal neighbour is blocked
        if dr and dc and (
            is_collision(cell_to_xy((r+dr,c),cs), obstacles, clearance=0.0) or
            is_collision(cell_to_xy((r,c+dc),cs), obstacles, clearance=0.0)
        ):
            continue
        yield (nr, nc), cost

# ── Weighted A* ───────────────────────────────────────────────────────────────

def weighted_astar(start, goal, obstacles, map_size,
                   epsilon=WA_EPSILON, cs=CELL_SIZE):
    """
    WA* corridor planner. Uses is_collision() — no separate grid array.

    start, goal : (x,y) meters
    obstacles   : (x,y,r,vx,vy) list — radius must already include CLEARANCE
    epsilon     : heuristic weight (2.5 = fast replan)
    Returns     : list of (x,y) waypoints, or None if blocked
    """
    rows, cols = int(map_size[1]/cs), int(map_size[0]/cs)

    # Convert and clamp
    sc = (max(0,min(rows-1, int(start[1]/cs))), max(0,min(cols-1, int(start[0]/cs))))
    gc = (max(0,min(rows-1, int(goal[1] /cs))), max(0,min(cols-1, int(goal[0] /cs))))

    if is_collision(cell_to_xy(sc,cs), obstacles, clearance=0.0):
        print("[WA*] Start in collision — skipping"); return None
    if is_collision(cell_to_xy(gc,cs), obstacles, clearance=0.0):
        print("[WA*] Goal in collision — skipping");  return None

    h          = lambda c: math.hypot(gc[0]-c[0], gc[1]-c[1])
    heap       = [(epsilon*h(sc), 0.0, 0, sc)]
    visited    = set()
    parent     = {sc: None}
    g_cost     = {sc: 0.0}
    tie        = 0

    while heap:
        _, g, _, cur = heapq.heappop(heap)
        if cur in visited: continue
        visited.add(cur)

        if cur == gc:
            # Reconstruct
            path, node = [], cur
            while node: path.append(cell_to_xy(node,cs)); node = parent[node]
            return path[::-1]

        for nb, step in get_neighbors(cur, obstacles, map_size, cs):
            ng = g + step
            if ng < g_cost.get(nb, float('inf')):
                g_cost[nb] = ng; parent[nb] = cur; tie += 1
                heapq.heappush(heap, (ng + epsilon*h(nb), ng, tie, nb))

    print("[WA*] No corridor — Bi-RRT* will use random sampling")
    return None

# ── Combined entry point — 6 lines of new logic ──────────────────────────────

def wa_birrt_star(start, goal, obstacles, map_size,
                  max_iter=5000, step_size=STEP_SIZE, sigma=SIGMA,
                  goal_bias=GOAL_BIAS, rewire_radius=REWIRE_RADIUS,
                  wa_epsilon=WA_EPSILON):
    """
    Run WA* for a corridor, then feed it straight into birrt_star().
    If WA* fails, birrt_star() automatically falls back to random sampling
    because wa_path=None is already handled inside birrt_star.
    """
    print("[WA*] Generating corridor...")
    wa_path = weighted_astar(start, goal, obstacles, map_size, epsilon=wa_epsilon)
    print(f"[WA*] {'Corridor: '+str(len(wa_path))+' waypoints' if wa_path else 'No corridor — random fallback'}")

    return birrt_star(
        start=start, goal=goal, obstacles=obstacles, map_size=map_size,
        max_iter=max_iter, step_size=step_size, sigma=sigma,
        goal_bias=goal_bias, rewire_radius=rewire_radius,
        wa_path=wa_path   # None = random sampling, corridor = guided sampling
    )

# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("="*50)
    print(f"WA* + Bi-RRT* | Start:{START} Goal:{GOAL} | ε={WA_EPSILON}")
    print("="*50)

    # Bake CLEARANCE into radii — required for clearance=0.0 contract
    obs = [(x, y, r+CLEARANCE, vx, vy) for x,y,r,vx,vy in OBSTACLES]

    path, fw_tree, rv_tree = wa_birrt_star(
        start=START, goal=GOAL, obstacles=obs, map_size=MAP_SIZE
    )

    if path:
        total = sum(math.dist(path[i], path[i-1]) for i in range(1,len(path)))
        print(f"Path length: {total:.2f}m  Waypoints: {len(path)}")

    animate_exploration(
        forward_tree=fw_tree, obstacles=OBSTACLES, map_size=MAP_SIZE,
        start=START, goal=GOAL, path=path, reverse_tree=rv_tree,
        interval=10, title="WA* + Bi-RRT*\nBlue=forward  Green=reverse  Red=path"
    )