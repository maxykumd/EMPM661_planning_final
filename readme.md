# WA*-Bi-RRT* — Weighted A* Guided Bidirectional RRT* with Dynamic Obstacle Avoidance
> ENPM661 Final Project | TurtleBot3 Waffle | Gazebo + ROS2

---

## 🎯 Project Goal

This project implements and compares five path planning algorithms for a TurtleBot3 Waffle robot in Gazebo simulation, culminating in a novel hybrid algorithm — **WA\*-Bi-RRT\*** — that combines Weighted A\* guidance with Bidirectional RRT\* to achieve fast, high-quality path planning in dynamic environments with moving obstacles.

**Core innovation:** Instead of sampling randomly like standard RRT, our algorithm uses the Weighted A\* solution path as a corridor to bias RRT sampling — making tree growth faster and smarter. On top of this, an obstacle projection bias steers sampling away from predicted obstacle positions in real time.

**Base paper:** [Bi-AM-RRT*: A Fast and Efficient Sampling-Based Motion Planning Algorithm in Dynamic Environments](https://arxiv.org/abs/2301.11816) — Zhang et al., IEEE Transactions on Intelligent Vehicles, 2023

---

## 📊 Algorithm Progression

| Algorithm | Speed | Path Quality | Dynamic Obstacles | Role |
|---|---|---|---|---|
| A* | Medium | Optimal | ❌ | Baseline 1 |
| Weighted A* | Fast | Suboptimal | ❌ | Baseline 2 |
| Vanilla RRT | Slow | Poor | ✅ | Baseline 3 |
| Bi-RRT* | Medium | Good | ✅ | Baseline 4 |
| **WA\*-Bi-RRT\*** | **Fast** | **Good** | **✅** | **⭐ Our contribution** |

**Paper argument:**
> A\* and Weighted A\* are optimal/fast but fail in dynamic environments. Standard RRT handles dynamics but samples blindly. Our WA\*-Bi-RRT\* uses Weighted A\* to guide RRT sampling through a corridor toward the goal, achieving faster convergence while maintaining dynamic obstacle avoidance.

---

## 👥 Team

| Person | Role |
|---|---|
| [Person 1 Name] | Bi-RRT* Core + ROS2 Integration |
| [Person 2 Name] | A\* + Weighted A\* + Replanning + Gazebo |
| [Person 3 Name] | WA\*-Guided Sampling + Smoothing + Visualization + Metrics |

---

## 🗂️ Project Structure

```
├── algorithms/
│   ├── rrt.py                  # Vanilla RRT (Person 1 — starting point)
│   ├── birrt_star.py           # Bi-RRT* core + tree connection logic (Person 1)
│   ├── astar.py                # A* on grid map (Person 2)
│   ├── weighted_astar.py       # Weighted A* — epsilon parameter (Person 2)
│   ├── path_checker.py         # is_path_blocked() (Person 2)
│   ├── replanner.py            # replan() (Person 2)
│   ├── sampler.py              # WA*-guided + obstacle bias sampling (Person 3)
│   └── smoother.py             # smooth_path() shortcutting (Person 3)
├── ros2/
│   └── rrt_node.py             # ROS2 node wrapping the planner (Person 1)
├── gazebo/
│   ├── worlds/                 # Gazebo world files (Person 2)
│   ├── moving_obstacles.py     # Moving obstacle ROS2 publisher (Person 2)
│   └── launch/                 # Launch files
├── visualization/
│   └── rviz_publisher.py       # RViz tree + path display (Person 3)
├── metrics/
│   └── logger.py               # Logs all metrics for all algorithms (Person 3)
├── mock_data.py                # Shared mock data for standalone testing
└── README.md
```

---

## ⚠️ Shared Data Structures — Read Before Writing Any Code

Everyone builds around these. **Do NOT change them.**

```python
# Node — one point in the RRT tree
node = {
    "pos": (x, y),      # position in 2D continuous space (meters)
    "parent": node,     # parent node (None for root)
    "cost": float       # total distance from root
}

# Tree — list of nodes
tree = [node, node, ...]

# Obstacle — position, size, and velocity
obstacle = (x, y, radius, vx, vy)  # vx, vy = velocity (meters/step)

# Path — ordered list of positions from start to goal
path = [(x, y), (x, y), ...]

# WA* corridor — output of Weighted A*, used to guide RRT sampling
# NOTE: must be converted from grid cells to (x,y) meters before use
wa_path = [(x, y), (x, y), ...]

# Grid cell — used internally by A* and Weighted A* only
cell = (row, col)

# Coordinate conversion (Person 2 must provide this)
def cell_to_xy(cell, cell_size=0.5):
    return (cell[1] * cell_size, cell[0] * cell_size)
```

> ⚠️ **Important:** A\* and Weighted A\* work on a grid internally but must output paths in **(x, y) meters** — not grid cells. Everything in RRT lives in continuous space.

---

## 🔌 Function Signatures — Do NOT Change These

All modules connect through these interfaces.

```python
# ── Person 2 implements ──────────────────────────────────────

def astar(grid, start_cell, goal_cell, cell_size=0.5):
    # grid: 2D array (0 = free, 1 = obstacle)
    # start_cell: (row, col)
    # goal_cell: (row, col)
    # cell_size: meters per cell (for coordinate conversion)
    # returns: path as list of (x,y) points in meters, or None
    pass

def weighted_astar(grid, start_cell, goal_cell, epsilon=1.5, cell_size=0.5):
    # same as astar but f = g + epsilon * h
    # epsilon > 1.0 = faster but suboptimal
    # returns: path as list of (x,y) points in meters, or None
    pass

def is_path_blocked(path, obstacles):
    # path: list of (x,y) points in meters
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: (is_blocked: bool, segment_index: int or None)
    pass

def replan(tree, blocked_segment, obstacles):
    # tree: list of nodes
    # blocked_segment: index of blocked path segment
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: new path as list of (x,y) points, or None if failed
    pass


# ── Person 3 implements ──────────────────────────────────────

def sample_state(tree, goal, obstacles, wa_path, map_size,
                 corridor_width=1.0):
    # Four sampling behaviors:
    #   60% — sample near WA* corridor (guided exploration)
    #   15% — sample away from predicted obstacle positions (safety bias)
    #   15% — sample toward goal (convergence speed)
    #   10% — pure random (maintain probabilistic completeness)
    #
    # tree: list of nodes
    # goal: (x, y) in meters
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # wa_path: list of (x,y) points in meters from Weighted A*
    # map_size: (width, height) in meters
    # corridor_width: sampling corridor radius around wa_path in meters
    # returns: (x, y) sample point in meters
    pass

def smooth_path(path, obstacles):
    # Shortcutting: skip intermediate nodes if direct connection is clear
    # path: list of (x,y) points in meters
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: smoothed path as list of (x,y) points
    pass
```

---

## 🧪 Mock Data for Standalone Testing

Use this to develop and test your functions **independently** before integration day.

```python
# mock_data.py

MAP_SIZE = (20.0, 20.0)   # meters
CELL_SIZE = 0.5            # meters per grid cell

mock_path = [(0,0), (2,2), (4,4), (6,6), (8,8), (10,10)]

mock_wa_path = [(0,0), (2,1), (4,2), (6,4), (8,6), (10,10)]

mock_obstacles_static = [
    (5.0, 5.0, 0.3, 0.0, 0.0),    # on the path — should trigger
    (15.0, 15.0, 0.3, 0.0, 0.0),  # off the path — should not trigger
]

mock_obstacles_moving = [
    (2.0, 2.0, 0.3, 0.5, 0.0),    # moving right, will cross path
    (15.0, 15.0, 0.3, 0.1, 0.1),  # off path — should not trigger
]

mock_tree = [
    {"pos": (0,0), "parent": None, "cost": 0.0},
    {"pos": (1,0), "parent": None, "cost": 1.0},
    {"pos": (2,0), "parent": None, "cost": 2.0},
    {"pos": (1,1), "parent": None, "cost": 1.41},
]

mock_grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0],
]  # 0 = free, 1 = obstacle

# Use in Gazebo before real planner is ready
hardcoded_path = [(0,0), (2,0), (4,0), (6,0), (8,0), (10,0)]
```

Expected test results:
```python
# Person 2 tests
print(is_path_blocked(mock_path, mock_obstacles_static))   # → (True, 1)
print(is_path_blocked(mock_path, mock_obstacles_moving))   # → (True, 0)
print(weighted_astar(mock_grid, (0,0), (4,4), epsilon=1.5))  # → valid path

# Person 3 tests
print(smooth_path([(0,0),(2,0),(4,0),(6,0),(8,0)], []))    # → [(0,0), (8,0)]
```

---

## 📋 Task Breakdown

### [Person 1 Name] — Bi-RRT* Core + ROS2
- [ ] Share data structures + mock data with team *(Fri May 2 AM)*
- [ ] Vanilla RRT working in Python with matplotlib *(Fri May 2 PM)*
- [ ] Bidirectional RRT working — both trees connecting *(Sat May 3 AM)*
- [ ] RRT* rewiring step added — cost tracking + `rewire()` *(Sat May 3 PM)*
- [ ] Tree connection logic — σ merge of forward and reverse trees *(Sat May 3 PM)*
- [ ] ROS2 node wrapping the full WA*-Bi-RRT* algorithm *(Sun May 4)*
- [ ] Final integration of all modules *(Mon May 5)*
- [ ] Run Bi-RRT* and WA*-Bi-RRT* experiments + collect metrics *(Mon May 5)*
- [ ] **If finished early → help Person 2 with ROS2 node wrapping**

### [Person 2 Name] — A* + Weighted A* + Replanning + Gazebo
- [ ] Read paper + data structures + mock data *(Fri May 2 AM)*
- [ ] A* working on grid map, tested standalone *(Fri May 2 PM)*
- [ ] Weighted A* working — epsilon parameter added *(Sat May 3 AM)*
- [ ] `is_path_blocked()` fully tested against mock data *(Sat May 3 PM)*
- [ ] `replan()` — reroute using existing nearby tree nodes *(Sun May 4 AM)*
- [ ] Gazebo world setup + TurtleBot3 spawning *(Sun May 4)*
- [ ] Moving obstacles scripted and publishing to ROS2 topic *(Sun May 4)*
- [ ] Robot following hardcoded path in Gazebo *(Sun May 4 PM)*
- [ ] Swap hardcoded path for real planner output *(Mon May 5)*
- [ ] Run A* and Weighted A* experiments + collect metrics *(Mon May 5)*
- [ ] **If finished early → help with ROS2 node wrapping**

### [Person 3 Name] — Sampling + Smoothing + Visualization + Metrics
- [ ] Read paper + data structures + mock data *(Fri May 2 AM)*
- [ ] RViz basic setup *(Fri May 2)*
- [ ] `sample_state()` random + goal biased, tested standalone *(Sat May 3)*
- [ ] `sample_state()` WA* corridor guidance added *(Sun May 4 AM)*
- [ ] `sample_state()` obstacle projection bias added *(Sun May 4 AM)*
- [ ] `smooth_path()` shortcutting algorithm working *(Sun May 4 PM)*
- [ ] RViz: forward tree (blue), reverse tree (green), WA* corridor (yellow), final path (red) *(Sun May 4)*
- [ ] Metrics logger running for all 5 algorithms *(Mon May 5)*
- [ ] Generate comparison plots for paper *(Tue May 6)*
- [ ] **If finished early → help Person 2 with `replan()`**

---

## 📅 Timeline

| Day | Milestone |
|---|---|
| Fri May 2 | Data structures + mock data shared. Vanilla RRT done. A* done. RViz setup. |
| Sat May 3 | Bi-RRT* done. Weighted A* done. `is_path_blocked()` done. Basic sampling done. |
| Sun May 4 | ROS2 node done. `replan()` done. Gazebo pipeline done. WA*-guided sampling + smoothing + full RViz done. |
| **Mon May 5** | **Full integration day. Everyone available. Run all experiments. Collect all metrics.** |
| Tue May 6 | Paper writing + presentation slides. Record simulation demo. |
| Wed May 7 | Buffer — polish, proofread, final checks. Submit. |
| Thu May 8 | 🚨 Hard deadline @ 11:59PM |

---

## ⚠️ Ground Rules

1. Read the shared data structures and mock data **Friday morning before writing any code**
2. **Do NOT change the function signatures** — everything connects through these
3. Test your functions **standalone with mock data** before Mon May 5
4. **Coordinate system:** everything in meters in continuous 2D space — A\* grid cells must be converted to (x,y) meters before passing to RRT
