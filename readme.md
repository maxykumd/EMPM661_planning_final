# Bi-AM-RRT* — Bidirectional RRT with Dynamic Obstacle Avoidance
> ENPM661 Final Project | TurtleBot3 Waffle | Gazebo + ROS2

---

## 🎯 Project Goal

Implementation of a simplified Bidirectional RRT algorithm with dynamic obstacle avoidance for a TurtleBot3 Waffle robot in Gazebo simulation. The robot plans a path from a start point to a goal point and reroutes in real time when moving obstacles block its way.

**Originality contribution:** An obstacle projection bias in the sampling strategy that steers the tree away from where moving obstacles are predicted to be.

**Base paper:** [Bi-AM-RRT*: A Fast and Efficient Sampling-Based Motion Planning Algorithm in Dynamic Environments](https://arxiv.org/abs/2301.11816) — Zhang et al., IEEE Transactions on Intelligent Vehicles, 2023

---

## 👥 Team

| Person | Role |
|---|---|
| [Max] | Bi-RRT Core + ROS2 Integration |
| [Person 2 Name] | Replanning + Obstacle System + Gazebo |
| [Person 3 Name] | Sampling + Smoothing + Visualization + Metrics |

---

## 🗂️ Project Structure

```
├── birrt_core/
│   ├── birrt.py              # Bidirectional RRT tree + connection logic (Person 1)
│   ├── rrt_node.py           # ROS2 node wrapping the planner (Person 1)
│   └── mock_data.py          # Shared mock data for standalone testing
├── obstacle_system/
│   ├── path_checker.py       # is_path_blocked() (Person 2)
│   ├── replanner.py          # replan() (Person 2)
│   └── moving_obstacles.py   # Moving obstacle ROS2 publisher (Person 2)
├── sampling/
│   ├── sampler.py            # sample_state() (Person 3)
│   └── smoother.py           # smooth_path() (Person 3)
├── visualization/
│   └── rviz_publisher.py     # RViz tree + path display (Person 3)
├── gazebo/
│   ├── worlds/               # Gazebo world files (Person 2)
│   └── launch/               # Launch files
├── metrics/
│   └── logger.py             # Planning time, path length, replan count (Person 3)
└── README.md
```

---

## ⚠️ Shared Data Structures — Read Before Writing Any Code

Everyone builds around these. **Do NOT change them.**

```python
# Node — one point in the tree
node = {
    "pos": (x, y),      # position in 2D space
    "parent": node,     # parent node (None for root)
    "cost": float       # total distance from root
}

# Tree — list of nodes
tree = [node, node, ...]

# Obstacle — position, size, and velocity
obstacle = (x, y, radius, vx, vy)  # vx, vy = velocity direction

# Path — list of positions from start to goal
path = [(x, y), (x, y), ...]
```

---

## 🔌 Function Signatures — Do NOT Change These

Everything plugs together through these interfaces.

```python
# --- Person 2 implements ---

def is_path_blocked(path, obstacles):
    # path: list of (x,y) points
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: (is_blocked: bool, segment_index: int or None)
    pass

def replan(tree, blocked_segment, obstacles):
    # tree: list of nodes
    # blocked_segment: index of blocked path segment
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: new path as list of (x,y) points, or None if failed
    pass

# --- Person 3 implements ---

def sample_state(tree, goal, obstacles):
    # tree: list of nodes
    # goal: (x, y)
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: (x, y) sample point
    pass

def smooth_path(path, obstacles):
    # path: list of (x,y) points
    # obstacles: list of (x, y, radius, vx, vy) tuples
    # returns: smoothed path as list of (x,y) points
    pass
```

---

## 🧪 Mock Data for Standalone Testing

Use this to develop and test your functions independently before integration day.

```python
# mock_data.py
mock_path = [(0,0), (1,1), (2,2), (3,3), (4,4), (5,5)]

mock_obstacles_static = [
    (2.5, 2.5, 0.3, 0.0, 0.0),  # on the path — should trigger
    (8.0, 8.0, 0.3, 0.0, 0.0),  # off the path — should not trigger
]

mock_obstacles_moving = [
    (1.0, 1.0, 0.3, 0.5, 0.0),  # moving right, will cross path
    (8.0, 8.0, 0.3, 0.1, 0.1),  # off path — should not trigger
]

mock_tree = [
    {"pos": (0,0), "parent": None, "cost": 0},
    {"pos": (1,0), "parent": None, "cost": 1},
    {"pos": (2,0), "parent": None, "cost": 2},
]

# Use this in Gazebo before real planner is ready
hardcoded_path = [(0,0), (1,0), (2,0), (3,0), (4,0)]
```

Expected test results:
```python
print(is_path_blocked(mock_path, mock_obstacles_static))  # → (True, 1)
print(is_path_blocked(mock_path, mock_obstacles_moving))  # → (True, 0)
print(smooth_path([(0,0),(1,0),(2,0),(3,0),(4,0)], []))   # → [(0,0), (4,0)]
```

---

## 📋 Task Breakdown

### [Max] — Bi-RRT Core + ROS2
- [ ] Share data structures + mock data with team *(Fri May 2 AM)*
- [ ] Vanilla RRT working in Python with matplotlib *(Fri May 2 PM)*
- [ ] Bidirectional RRT working in Python — both trees connecting *(Sat May 3)*
- [ ] Tree connection logic — σ merge of forward and reverse trees *(Sat May 3)*
- [ ] ROS2 node wrapping the full algorithm *(Sun May 4)*
- [ ] Final integration of all modules *(Mon May 5)*

### [Person 2 Name] — Replanning + Obstacle System + Gazebo
- [ ] Read paper + data structures + mock data *(Fri May 2 AM)*
- [ ] Gazebo world setup + TurtleBot3 spawning *(Fri May 2)*
- [ ] `is_path_blocked()` fully tested against mock data *(Sat May 3)*
- [ ] `replan()` — reroute using existing nearby tree nodes *(Sun May 4)*
- [ ] Moving obstacles publishing positions to ROS2 topic *(Sun May 4)*
- [ ] Robot following hardcoded path in Gazebo *(Sun May 4)*
- [ ] Swap hardcoded path for real planner output *(Mon May 5)*
- [ ] **If finished early → help with ROS2 node wrapping**

### [Person 3 Name] — Sampling + Smoothing + Visualization + Metrics
- [ ] Read paper + data structures + mock data *(Fri May 2 AM)*
- [ ] RViz basic setup *(Fri May 2)*
- [ ] `sample_state()` random + goal biased sampling tested *(Sat May 3)*
- [ ] `sample_state()` obstacle projection bias added — originality contribution *(Sun May 4)*
- [ ] `smooth_path()` shortcutting algorithm working *(Sun May 4)*
- [ ] RViz showing forward tree (blue), reverse tree (green), final path (red) *(Sun May 4)*
- [ ] Metrics logging — planning time, path length, replan count *(Mon May 5)*
- [ ] **If finished early → help Person 2 with `replan()`**

---

## 📅 Timeline

| Day | Milestone |
|---|---|
| Fri May 2 | Data structures shared. Vanilla RRT. Gazebo + RViz setup. |
| Sat May 3 | Bi-RRT working in Python. `is_path_blocked()` done. Basic sampling done. |
| Sun May 4 | ROS2 node done. `replan()` done. Gazebo pipeline done. Smoothing + bias + RViz done. |
| Mon May 5 | **Full integration day. Everyone available.** |
| Tue May 6 | Buffer. Bug fixes. Record simulation demo. |
| Wed May 7 | True buffer. Only touch if broken. Submit. |
| Thu May 8 | 🚨 Hard deadline @ 11:59PM |

---

## 📏 Metrics Logged Per Run

| Metric | Description |
|---|---|
| Planning time | Time from start to first valid path found |
| Path length | Total Euclidean distance of final path |
| Replan count | Number of times path was blocked and rerouted |

---

## ⚠️ Ground Rules
1. Read the shared data structures before writing any code
2. Do NOT change the function signatures
3. Test your functions standalone with mock data before Mon May 5
