# WA*-Bi-RRT* — Weighted A* Guided Bidirectional RRT* with Dynamic Obstacle Avoidance
> ENPM661 Final Project | TurtleBot3 Waffle | Gazebo + ROS2

---

## 🎯 Project Goal

This project implements and compares five path planning algorithms for a TurtleBot3 Waffle robot in Gazebo simulation, culminating in a novel hybrid algorithm — **WA\*-Bi-RRT\*** — that combines Weighted A\* guidance with Bidirectional RRT\* to achieve fast, high-quality path planning in dynamic environments with moving obstacles.

**Core innovation:** Instead of sampling randomly like standard RRT, our algorithm uses the Weighted A\* solution path as a corridor to bias RRT sampling — making tree growth faster and smarter. On top of this, an obstacle projection bias steers sampling away from predicted obstacle positions in real time. The corridor width adapts dynamically based on planning progress, widening early to explore and narrowing as the tree approaches the goal.

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
> A\* and Weighted A\* are optimal/fast but fail in dynamic environments. Standard RRT handles dynamics but samples blindly. Our WA\*-Bi-RRT\* uses Weighted A\* to guide RRT sampling through an adaptive corridor toward the goal, achieving faster convergence while maintaining dynamic obstacle avoidance.

---

## 👥 Team & Work Division

| Person | Role | Load |
|---|---|---|
| [Your Name] | Bi-RRT* Core + ROS2 + Gazebo Integration | ~40% |
| [Person 2 Name] | A\* + Weighted A\* + Replanning + Maps + Gazebo Worlds | ~30% |
| [Person 3 Name] | Sampling + Originality + Smoothing + Visualization + Metrics | ~30% |

---

## 🗂️ Project Structure

```
├── algorithms/
│   ├── rrt.py                  # Vanilla RRT (Person 1)
│   ├── birrt_star.py           # Bi-RRT* core + rewiring (Person 1)
│   ├── astar.py                # A* on grid map (Person 2)
│   ├── weighted_astar.py       # Weighted A* (Person 2)
│   ├── path_checker.py         # is_path_blocked() (Person 2)
│   ├── replanner.py            # replan() (Person 2)
│   ├── sampler.py              # WA*-guided adaptive sampling (Person 3)
│   └── smoother.py             # smooth_path() (Person 3)
├── ros2/
│   ├── rrt_node.py             # ROS2 planner node (Person 1)
│   └── obstacle_publisher.py   # Moving obstacle ROS2 publisher (Person 1)
├── gazebo/
│   ├── worlds/                 # Gazebo world files (Person 2)
│   ├── maps/                   # Grid map numpy arrays (Person 2)
│   └── launch/                 # Launch files (Person 1)
├── visualization/
│   └── rviz_publisher.py       # RViz display node (Person 3)
├── metrics/
│   └── logger.py               # Metrics logger + plots (Person 3)
├── mock_data.py                # Shared mock data — read before coding
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

# WA* corridor — output of Weighted A* converted to meters
wa_path = [(x, y), (x, y), ...]

# Grid — used internally by A* and Weighted A* only
grid = [[0, 1, 0, ...], ...]   # 0 = free, 1 = obstacle
```

> ⚠️ **Coordinate rule:** Everything in RRT lives in continuous (x, y) meters.
> A\* works on grid cells internally but must output (x, y) meters.
> Person 2 provides `cell_to_xy()` conversion.

---

## 🔌 Function Signatures — Do NOT Change These

```python
# ── Person 2 ─────────────────────────────────────────────────

def astar(grid, start_cell, goal_cell, cell_size=0.5):
    # returns: path as list of (x,y) meters, or None
    pass

def weighted_astar(grid, start_cell, goal_cell, epsilon=1.5, cell_size=0.5):
    # returns: path as list of (x,y) meters, or None
    pass

def is_path_blocked(path, obstacles):
    # returns: (is_blocked: bool, segment_index: int or None)
    pass

def replan(tree, blocked_segment, obstacles):
    # returns: new path as list of (x,y) points, or None
    pass

def cell_to_xy(cell, cell_size=0.5):
    # returns: (x, y) in meters
    pass


# ── Person 3 ─────────────────────────────────────────────────

def sample_state(tree, goal, obstacles, wa_path, map_size,
                 corridor_width=1.0, progress=0.0):
    # corridor_width adapts based on progress (0.0 to 1.0)
    # returns: (x, y) sample point
    pass

def smooth_path(path, obstacles):
    # returns: smoothed path as list of (x,y) points
    pass
```

---

## 🧪 Mock Data

```python
# mock_data.py

MAP_SIZE = (20.0, 20.0)
CELL_SIZE = 0.5

mock_path = [(0,0), (2,2), (4,4), (6,6), (8,8), (10,10)]
mock_wa_path = [(0,0), (2,1), (4,2), (6,4), (8,6), (10,10)]

mock_obstacles_static = [
    (5.0, 5.0, 0.3, 0.0, 0.0),
    (15.0, 15.0, 0.3, 0.0, 0.0),
]
mock_obstacles_moving = [
    (2.0, 2.0, 0.3, 0.5, 0.0),
    (15.0, 15.0, 0.3, 0.1, 0.1),
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
]
hardcoded_path = [(0,0), (2,0), (4,0), (6,0), (8,0), (10,0)]
```

---

---

# 👤 PERSON 1 — Bi-RRT* Core + ROS2 + Gazebo Integration
### [Your Name] | ~40%

---

### 📌 Overview
You own the core planning algorithm and all ROS2 + Gazebo integration. You are the backbone of the project — everything plugs into what you build. You also lead integration day on Monday.

---

### 📋 Detailed Task List

#### Before Writing Any Code — Friday May 2 Morning
- [ ] Read the paper (pages 1–4, Section III and IV only)
- [ ] Write and share `mock_data.py` with the team
- [ ] Write and share the shared data structures with the team
- [ ] Agree on coordinate system with Person 2 (meters, continuous space)
- [ ] Set up Python dev environment (numpy, matplotlib)
- [ ] Create the GitHub repo and push initial folder structure

---

#### Step 1 — Vanilla RRT in Python (Friday May 2)
Do not move to Step 2 until this works and is visualized correctly.

- [ ] Implement `random_sample(map_size)` — random (x,y) within map bounds
- [ ] Implement `get_nearest(tree, point)` — closest node in tree by Euclidean distance
- [ ] Implement `steer(from_pos, to_pos, step_size)` — move step_size units toward target
- [ ] Implement `is_collision(pos, obstacles)` — circle collision check
- [ ] Implement `is_collision_free_path(pos_a, pos_b, obstacles)` — sample points along segment and check each
- [ ] Implement `extract_path(tree, goal_node)` — trace parent pointers back to root
- [ ] Implement main `rrt()` loop combining all above
- [ ] Add 10–15% goal bias — sample goal directly with that probability
- [ ] Visualize with matplotlib: tree (grey), path (red), start (green), goal (blue), obstacles (black)
- [ ] Test in 3 different static environments
- [ ] Verify path is collision free visually

---

#### Step 2 — Bidirectional RRT (Saturday May 3 Morning)
- [ ] Initialize two trees — forward (from start) and reverse (from goal)
- [ ] Implement alternating growth loop — grow forward one step, reverse one step, repeat
- [ ] Implement `try_connect(forward_tree, reverse_tree, sigma)` — check if any node pair is within sigma with no obstacle between
- [ ] When connected, merge into one full path — forward path + reverse path
- [ ] Handle max_iter exceeded — return None gracefully
- [ ] Update visualization — forward tree blue, reverse tree green
- [ ] Verify both trees grow and connect correctly
- [ ] Compare planning time to vanilla RRT — Bi-RRT should be faster

---

#### Step 3 — RRT* Rewiring (Saturday May 3 Afternoon)
- [ ] Add cost field to every node — total Euclidean distance from root
- [ ] When adding new node, find all nearby nodes within rewire_radius
- [ ] For each nearby node check if reaching it through new node is cheaper
- [ ] If cheaper and collision free — rewire (update parent and cost)
- [ ] Also find minimum cost parent for the new node itself before adding
- [ ] Apply rewiring to both forward and reverse trees
- [ ] Verify path length measurably improves vs without rewiring over 20 runs
- [ ] Recommended rewire_radius = 1.5–2.0x step_size

---

#### Step 4 — Moving Obstacle ROS2 Publisher (Sunday May 4 Morning)
- [ ] Create ROS2 Python node `obstacle_publisher.py`
- [ ] Implement 3 moving obstacles with different behaviors:
  - Obstacle 1: straight line bouncing off walls
  - Obstacle 2: circular path
  - Obstacle 3: random direction changes
- [ ] Publish all obstacle positions to `/obstacle_positions` at 10Hz
  - Format: array of (x, y, radius, vx, vy) tuples
- [ ] Add Gazebo visual models for each obstacle
- [ ] Verify published positions match visual positions in Gazebo
- [ ] Make obstacle speed configurable for experiments

---

#### Step 5 — ROS2 Node Wrapping (Sunday May 4 Afternoon)
- [ ] Create ROS2 Python node `rrt_node.py`
- [ ] Subscribe to `/map` — receive occupancy grid from Gazebo
- [ ] Subscribe to `/obstacle_positions` — receive moving obstacle positions
- [ ] Subscribe to `/goal_pose` — receive goal position
- [ ] Subscribe to `/odom` — receive robot current position
- [ ] Convert occupancy grid to obstacle list format
- [ ] On receiving goal: run `weighted_astar()` → get `wa_path`
- [ ] Run Bi-RRT* using `sample_state(wa_path=wa_path)` from Person 3
- [ ] Apply `smooth_path()` from Person 3
- [ ] Publish final path to `/planned_path` as nav_msgs/Path
- [ ] Periodically call `is_path_blocked()` from Person 2 on current path
- [ ] If blocked, call `replan()` from Person 2 and republish updated path
- [ ] Log planning time and path length to terminal each run
- [ ] Create launch file that starts everything in one command
- [ ] Test node compiles and runs without errors before integration day

---

#### Step 6 — Gazebo Integration (Monday May 5)
- [ ] Confirm all Person 2 functions import and work correctly
- [ ] Confirm all Person 3 functions import and work correctly
- [ ] Plug all functions into ROS2 node — replace placeholders
- [ ] Launch Gazebo world with TurtleBot3
- [ ] Launch planner node
- [ ] Launch RViz visualization node (Person 3)
- [ ] Verify robot receives planned path and moves in Gazebo
- [ ] Trigger moving obstacles and verify replanning occurs
- [ ] Debug integration issues with team
- [ ] Record working simulation once stable

---

#### Step 7 — Experiments (Monday May 5 Afternoon)
- [ ] Run Bi-RRT* without WA* guidance — 20 runs × 3 environments
- [ ] Run WA*-Bi-RRT* with WA* guidance — 20 runs × 3 environments
- [ ] Run both with and without dynamic obstacles
- [ ] Save all results to CSV for Person 3 to plot

---

#### Escalation If Finished Early
- [ ] Add a 4th more complex environment
- [ ] Help Person 2 with `replan()` if needed

---

---

# 👤 PERSON 2 — A* + Weighted A* + Replanning + Maps + Gazebo Worlds
### [Person 2 Name] | ~30%

---

### 📌 Overview
You own all grid-based planning algorithms and the environment. Since we already implemented A* in a previous project, your A* and Weighted A* tasks are mostly adapting existing code. Your main new algorithm work is `is_path_blocked()` and `replan()`. You also create all Gazebo world files and matching grid maps. You do NOT touch ROS2 integration — Person 1 handles that.

---

### 📋 Detailed Task List

#### Before Writing Any Code — Friday May 2 Morning
- [ ] Read the paper (pages 1–4, Section III and IV only)
- [ ] Read shared data structures and mock_data.py from Person 1
- [ ] Set up Python dev environment
- [ ] Pull A* implementation from previous project

---

#### Step 1 — Map Creation (Friday May 2 Morning)
This is a critical shared dependency — everyone needs the maps to test realistically.

- [ ] Design Environment 1: simple open room with one L-shaped wall (10m × 10m)
- [ ] Create Gazebo `.world` file for Environment 1 — box room with wall models
- [ ] Create matching grid map as 2D numpy array (cell_size = 0.5m, 0 = free, 1 = obstacle)
- [ ] Implement `cell_to_xy(cell, cell_size=0.5)` — grid (row,col) → (x,y) meters
- [ ] Implement `xy_to_cell(pos, cell_size=0.5)` — (x,y) meters → grid (row,col)
- [ ] Verify grid map visually with matplotlib — must match Gazebo world exactly
- [ ] Share map files and conversion functions with team immediately
- [ ] Design Environments 2 and 3 (build Sunday):
  - Environment 2: narrow corridor world
  - Environment 3: cluttered office with many small obstacles

---

#### Step 2 — A* Adaptation (Friday May 2 Afternoon)
Adapt from previous project — do not rewrite from scratch.

- [ ] Pull existing A* code from previous project
- [ ] Verify it runs correctly on mock_grid
- [ ] Update output — path must be returned as list of (x,y) meters using `cell_to_xy()`
- [ ] Verify output format matches the shared `path` data structure
- [ ] Test on Environment 1 grid map — verify path visually with matplotlib
- [ ] Measure and log planning time for 20 runs

---

#### Step 3 — Weighted A* (Saturday May 3 Morning)
- [ ] Copy A* implementation
- [ ] Add epsilon parameter (default 1.5)
- [ ] Change heuristic calculation to `h_weighted = epsilon * h`
- [ ] Everything else identical to A*
- [ ] Test with epsilon = 1.0, 1.5, 2.0, 3.0
- [ ] Verify higher epsilon = faster time but longer path
- [ ] Log planning time and path length for each epsilon value
- [ ] Make epsilon easily configurable for experiments

---

#### Step 4 — Path Validity Checker (Saturday May 3 Afternoon)
- [ ] Implement `is_path_blocked(path, obstacles)`:
  - Iterate through each consecutive pair of points (segment) in path
  - For each segment check if any obstacle circle intersects the line segment
  - Use point-to-segment distance formula for accurate detection
  - Return `(True, segment_index)` for first blocked segment
  - Return `(False, None)` if path is clear
- [ ] Test against `mock_obstacles_static` — verify `(True, 1)`
- [ ] Test against `mock_obstacles_moving` — verify `(True, 0)`
- [ ] Test with obstacle clearly off the path — verify `(False, None)`
- [ ] Test with empty obstacle list — verify `(False, None)`
- [ ] Test edge cases: obstacle touching path boundary, very short segments

---

#### Step 5 — Replanning Logic (Sunday May 4 Morning)
- [ ] Implement `replan(tree, blocked_segment, obstacles)`:
  - Find the last safe node on current path before blocked segment
  - Search existing tree nodes near the blocked area for alternate routes
  - Connect alternate nodes into a new valid path to the goal
  - Do NOT rebuild tree from scratch — use existing nodes only
  - Return new path if found, None if no alternate exists
- [ ] Test with `mock_tree` and `mock_obstacles_static` — verify valid alternate path returned
- [ ] Test when no alternate exists — verify None returned gracefully
- [ ] Verify returned path is collision free
- [ ] Test that replanned path connects correctly from last safe node to goal

---

#### Step 6 — Environments 2 and 3 (Sunday May 4 Afternoon)
- [ ] Create Gazebo world file for Environment 2 (narrow corridor)
- [ ] Create matching grid map for Environment 2
- [ ] Create Gazebo world file for Environment 3 (cluttered office)
- [ ] Create matching grid map for Environment 3
- [ ] Verify all grid maps match their Gazebo worlds visually
- [ ] Test A* and Weighted A* on all 3 environments

---

#### Step 7 — Experiments (Monday May 5)
- [ ] Run A* on all 3 environments — 20 runs each — log planning time, path length
- [ ] Run Weighted A* on all 3 environments — 20 runs each — log same metrics
- [ ] Test Weighted A* with epsilon = 1.0, 1.5, 2.0, 3.0
- [ ] Test both with and without dynamic obstacles
- [ ] Save all results to CSV for Person 3 to plot

---

#### Escalation If Finished Early
- [ ] Help Person 1 with ROS2 node wrapping
- [ ] Add obstacle speed variation to experiments

---

---

# 👤 PERSON 3 — Sampling + Originality + Smoothing + Visualization + Metrics
### [Person 3 Name] | ~30%

---

### 📌 Overview
You own the originality contribution of the entire project — the adaptive WA*-guided sampling strategy. Your `sample_state()` function is what makes WA*-Bi-RRT* novel and different from everything else in the literature. You also own path smoothing, RViz visualization, and all metrics and plots. If you finish early, there are clear originality extensions that will strengthen the paper further.

---

### 📋 Detailed Task List

#### Before Writing Any Code — Friday May 2 Morning
- [ ] Read the paper (pages 1–4, Section III and IV only)
- [ ] Read shared data structures and mock_data.py from Person 1
- [ ] Set up Python dev environment (numpy, matplotlib)
- [ ] Set up ROS2 + RViz on your machine
- [ ] Verify RViz launches and displays a basic marker

---

#### Step 1 — RViz Setup (Friday May 2)
Get RViz ready before the algorithm exists so you can visualize from Day 1.

- [ ] Create ROS2 Python visualization node `rviz_publisher.py`
- [ ] Create RViz configuration file (.rviz)
- [ ] Implement publisher for forward tree edges — blue lines (MarkerArray)
- [ ] Implement publisher for reverse tree edges — green lines (MarkerArray)
- [ ] Implement publisher for WA* corridor path — yellow line (Marker)
- [ ] Implement publisher for final planned path — red line (Marker)
- [ ] Implement publisher for start point — green sphere
- [ ] Implement publisher for goal point — blue sphere
- [ ] Implement publisher for obstacle positions — grey spheres, update in real time
- [ ] Test all publishers with hardcoded mock data — verify everything displays in RViz
- [ ] Ensure visualization updates in real time as tree grows

---

#### Step 2 — Basic Sampling (Saturday May 3)
- [ ] Implement `random_sample(map_size)` — uniform random (x,y) within bounds
- [ ] Implement `goal_biased_sample(goal)` — returns goal point directly
- [ ] Implement probability switch in `sample_state()`:
  - p < 0.15 → return goal (goal bias)
  - else → return random sample
- [ ] Test: call `sample_state()` 1000 times, verify ~15% are exactly the goal
- [ ] Plot distribution of 1000 samples — verify uniform map coverage

---

#### Step 3 — WA* Corridor Guidance (Sunday May 4 Morning)
This is the core originality contribution. The corridor guides RRT toward the goal faster.

- [ ] Implement `sample_near_corridor(wa_path, corridor_width)`:
  - Pick a random waypoint from wa_path
  - Sample random point within corridor_width radius of that waypoint
  - Clamp to map bounds
  - Return sampled (x,y)
- [ ] Update probability switch:
  - p < 0.60 → sample near WA* corridor
  - p < 0.75 → sample toward goal
  - p < 0.90 → sample away from obstacles (next step)
  - else → pure random
- [ ] Test with mock_wa_path — plot 1000 samples, verify clustering around corridor
- [ ] Test corridor_width = 0.5, 1.0, 2.0 — verify wider = more spread visually
- [ ] Handle edge case: wa_path is empty or None → fall back to pure random

---

#### Step 4 — Obstacle Projection Bias (Sunday May 4 Morning)
- [ ] Implement `predict_obstacle_position(obstacle, steps_ahead=5)`:
  - Given (x, y, radius, vx, vy), predict position in steps_ahead steps
  - Return predicted (x, y)
- [ ] Implement `sample_away_from_obstacles(obstacles, map_size, steps_ahead=5)`:
  - Predict all obstacle positions steps_ahead into future
  - Generate random candidate sample
  - Reject if within safety_margin of any predicted position
  - Retry up to 10 times, return best candidate
- [ ] Integrate into probability switch at p < 0.90
- [ ] Test with mock_obstacles_moving — plot 1000 samples, verify avoidance of predicted areas
- [ ] Tune safety_margin (start at 2x obstacle radius)

---

#### Step 5 — Path Smoothing (Sunday May 4 Afternoon)
- [ ] Implement `smooth_path(path, obstacles)`:
  - Start at i = 0
  - Try connecting node i directly to i+2, i+3, skipping intermediates
  - Use collision free path check for each shortcut attempt
  - If clear, remove intermediate nodes
  - If blocked, advance i by 1
  - Continue until no more shortcuts possible
  - Return smoothed path
- [ ] Test with straight line + no obstacles — verify all intermediates removed
- [ ] Test with obstacles — verify intermediates kept where needed
- [ ] Verify smoothed path length always ≤ original path length
- [ ] Verify smoothed path is still collision free

---

#### Step 6 — Metrics Logger (Monday May 5)
- [ ] Create `MetricsLogger` class in `logger.py`
- [ ] Implement `start_timer()` and `stop_timer()` — compute planning time
- [ ] Implement `log_path(path)` — compute total path length in meters
- [ ] Implement `log_replan()` — increment replan counter
- [ ] Implement `log_run(algorithm, environment, success, epsilon=None)` — save one run
- [ ] Implement `save_to_csv(filename)` with columns:
  - algorithm, environment, run_number, planning_time, path_length, replan_count, success, epsilon
- [ ] Test standalone — simulate 5 fake runs, verify CSV written correctly
- [ ] Coordinate with Person 1 to integrate logger into ROS2 node on Monday

---

#### Step 7 — Results Plots (Tuesday May 6)
- [ ] Load all CSV results
- [ ] Plot 1: Bar chart — average planning time for all 5 algorithms × 3 environments
- [ ] Plot 2: Bar chart — average path length for all 5 algorithms × 3 environments
- [ ] Plot 3: Line plot — planning time vs epsilon for Weighted A*
- [ ] Plot 4: Line plot — path length vs epsilon for Weighted A*
- [ ] Plot 5: Bar chart — replan count for dynamic obstacle experiments
- [ ] Plot 6: Bar chart — success rate for all 5 algorithms with dynamic obstacles
- [ ] Save all plots as high-resolution PNG for the paper
- [ ] All plots must have proper axis labels, titles, and legends

---

### 🚀 Escalation — If Finished Early

#### Level 1 — Adaptive Corridor Width (First Priority)
This directly strengthens the originality contribution.

- [ ] Implement `compute_corridor_width(progress, base_width=1.0, min_width=0.3)`:
  - `progress` = how close the nearest tree node is to the goal (0.0 to 1.0)
  - `corridor_width = max(min_width, base_width * (1 - progress))`
  - Wide early (explore broadly) → narrow near goal (exploit corridor)
- [ ] Pass `progress` into `sample_state()` and use adaptive width
- [ ] Add corridor_width over time plot to metrics
- [ ] Compare fixed vs adaptive corridor in experiments — add to paper

#### Level 2 — Adaptive Obstacle Prediction (Second Priority)
- [ ] Implement `compute_steps_ahead(obstacle_speed, base_steps=5, max_speed=1.0)`:
  - `steps_ahead = base_steps * (obstacle_speed / max_speed)`
  - Fast obstacles → predict further ahead
  - Slow obstacles → predict closer
- [ ] Update `predict_obstacle_position()` to use adaptive steps_ahead
- [ ] Compare fixed vs adaptive prediction in experiments — add to paper

#### Level 3 — Help Others (Third Priority)
- [ ] Help Person 2 with `replan()` if they are behind
- [ ] Help Person 1 with ROS2 integration testing

---

---

## 📅 Timeline Summary

| Day | Person 1 | Person 2 | Person 3 |
|---|---|---|---|
| Fri May 2 | Share data structures. Vanilla RRT done. | Share maps. A* adapted. Gazebo Env 1 ready. | RViz setup. Basic sampling started. |
| Sat May 3 | Bi-RRT* + rewiring done. | Weighted A* done. is_path_blocked() done. | Basic sampling + WA* corridor started. |
| Sun May 4 | Moving obstacle publisher. ROS2 node done. | replan() done. Envs 2+3 done. | WA* guidance + obstacle bias + smooth_path + full RViz done. |
| **Mon May 5** | **INTEGRATION DAY — everyone available all day** | | |
| Tue May 6 | Support paper writing | Support paper writing | All plots done. Lead paper + slides. |
| Wed May 7 | Buffer — polish and submit | | |
| Thu May 8 | 🚨 Hard deadline @ 11:59PM | | |

---

## 📄 Paper Outline (IEEE Format, 6-8 pages)

```
I.   Introduction
       — why dynamic environments break grid-based planners
       — motivation for combining WA* and Bi-RRT*

II.  Background
       A. A* and Weighted A*
       B. RRT and Bi-RRT*
       C. Related work (cite base paper)

III. Proposed Algorithm — WA*-Bi-RRT*
       A. WA* corridor generation
       B. Adaptive guided sampling strategy
       C. Adaptive obstacle projection bias
       D. Bidirectional tree growth + connection logic
       E. RRT* rewiring

IV.  Experiments
       — 3 environments × 5 algorithms × 20 runs
       — Metrics: planning time, path length, replan count, success rate
       — Comparison table: all 5 algorithms × 3 environments
       — Plots: planning time vs epsilon, path length over iterations,
                corridor width effect, adaptive vs fixed bias

V.   Results & Discussion
VI.  Conclusion
```

---

## 📏 Metrics (All 5 Algorithms, 20 runs each)

| Metric | Description |
|---|---|
| Planning time (s) | Time to first valid path |
| Path length (m) | Total length of smoothed path |
| Replan count | Times path was blocked and rerouted |
| Success rate (%) | % of runs with valid path found |
| Path optimality | Path length / A* optimal path length |

---

## ⚠️ Ground Rules

1. Read data structures + mock data **Friday morning before writing any code**
2. **Do NOT change function signatures** — everything connects through these
3. Test your functions **standalone with mock data** before Mon May 5
4. **Coordinate system:** everything in meters — A\* must convert grid cells to (x,y) meters
5. Stuck for **more than 2 hours** → message the group immediately
6. Finish early → follow the escalation path in your section
