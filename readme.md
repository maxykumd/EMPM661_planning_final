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

---

## 👥 Team

| Person | Role |
|---|---|
| [Your Name] | Bi-RRT* Core + ROS2 Integration |
| [Person 2 Name] | A\* + Weighted A\* + Replanning + Map + Gazebo |
| [Person 3 Name] | WA\*-Guided Sampling + Smoothing + Visualization + Metrics |

---

## 🗂️ Project Structure

```
├── algorithms/
│   ├── rrt.py                  # Vanilla RRT (Person 1)
│   ├── birrt_star.py           # Bi-RRT* core (Person 1)
│   ├── astar.py                # A* on grid map (Person 2)
│   ├── weighted_astar.py       # Weighted A* (Person 2)
│   ├── path_checker.py         # is_path_blocked() (Person 2)
│   ├── replanner.py            # replan() (Person 2)
│   ├── sampler.py              # WA*-guided sampling (Person 3)
│   └── smoother.py             # smooth_path() (Person 3)
├── ros2/
│   └── rrt_node.py             # ROS2 node (Person 1)
├── gazebo/
│   ├── worlds/                 # Gazebo world files (Person 2)
│   ├── maps/                   # Grid map numpy arrays (Person 2)
│   ├── moving_obstacles.py     # Moving obstacle publisher (Person 2)
│   └── launch/                 # Launch files (Person 2)
├── visualization/
│   └── rviz_publisher.py       # RViz display (Person 3)
├── metrics/
│   └── logger.py               # Metrics logger (Person 3)
├── mock_data.py                # Shared mock data
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

# WA* corridor — output of Weighted A* in meters (not grid cells)
wa_path = [(x, y), (x, y), ...]

# Grid — used internally by A* and Weighted A* only
grid = [[0, 1, 0, ...], ...]   # 0 = free, 1 = obstacle
```

> ⚠️ **Coordinate rule:** Everything in RRT lives in continuous (x, y) meters.
> A\* works on grid cells internally but must output (x, y) meters.
> Person 2 provides the `cell_to_xy()` conversion function.

---

## 🔌 Function Signatures — Do NOT Change These

```python
# Person 2
def astar(grid, start_cell, goal_cell, cell_size=0.5): ...
def weighted_astar(grid, start_cell, goal_cell, epsilon=1.5, cell_size=0.5): ...
def is_path_blocked(path, obstacles): ...
def replan(tree, blocked_segment, obstacles): ...
def cell_to_xy(cell, cell_size=0.5): ...

# Person 3
def sample_state(tree, goal, obstacles, wa_path, map_size, corridor_width=1.0): ...
def smooth_path(path, obstacles): ...
```

---

## 🧪 Mock Data

```python
# mock_data.py — shared across all three people

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

# 👤 PERSON 1 — Bi-RRT* Core + ROS2 Integration
### [Your Name]

---

### 📌 Responsibility Overview
You own the core planning algorithm — the heart of the entire project. Everything else plugs into what you build. You also wrap the final algorithm into a ROS2 node and lead the integration of all three modules on Monday.

---

### 📋 Detailed Task List

#### BEFORE WRITING ANY CODE — Friday May 2 Morning
- [ ] Read the paper (pages 1–4, Section III and IV only)
- [ ] Write and share `mock_data.py` with the team
- [ ] Write and share the shared data structures with the team
- [ ] Agree on coordinate system with Person 2 (meters, continuous space)
- [ ] Set up your Python dev environment (numpy, matplotlib)
- [ ] Create the project GitHub repo and push the initial folder structure

---

#### STEP 1 — Vanilla RRT in Python (Friday May 2)

The goal is a working single-tree RRT that finds a path in a static 2D environment and visualizes it with matplotlib. This is your foundation — do not move to Step 2 until this works.

- [ ] Define the node data structure (pos, parent, cost)
- [ ] Implement `random_sample(map_size)` — returns a random (x,y) point within the map bounds
- [ ] Implement `get_nearest(tree, point)` — finds the closest node in the tree to a given point using Euclidean distance
- [ ] Implement `steer(from_pos, to_pos, step_size)` — moves step_size units from from_pos toward to_pos, returns new position
- [ ] Implement `is_collision(pos, obstacles)` — checks if a position is inside any obstacle (circle collision check)
- [ ] Implement `is_collision_free_path(pos_a, pos_b, obstacles)` — checks if the straight line between two positions is clear of all obstacles (sample several points along the line)
- [ ] Implement `extract_path(tree, goal_node)` — traces back through parent pointers from goal node to root and returns the full path as a list of (x,y) points
- [ ] Implement the main `rrt(start, goal, obstacles, map_size, max_iter, step_size)` loop combining all the above
- [ ] Add 10–15% goal bias — with that probability, sample the goal point directly instead of a random point (speeds up convergence)
- [ ] Visualize the result with matplotlib: show the tree growing (grey lines), the final path (red line), start (green dot), goal (blue dot), obstacles (black circles)
- [ ] Test in at least 3 different static environments with different obstacle configurations
- [ ] Verify the path is actually collision free by visual inspection

---

#### STEP 2 — Bidirectional RRT in Python (Saturday May 3 Morning)

Add a second tree growing from the goal toward the start simultaneously. When the two trees get close enough, connect them.

- [ ] Initialize two separate trees — forward tree (from start) and reverse tree (from goal)
- [ ] Implement the alternating growth loop — grow forward tree one iteration, then reverse tree one iteration, repeat
- [ ] Implement `try_connect(forward_tree, reverse_tree, sigma)` — checks if any node in the forward tree is within distance sigma of any node in the reverse tree with no obstacle between them
- [ ] When connection is found, merge the two trees into one path — forward path from start to connection point + reverse path from connection point to goal
- [ ] Handle the case where trees never connect within max_iter — return None
- [ ] Update matplotlib visualization to show forward tree in blue and reverse tree in green
- [ ] Test and verify both trees are visually growing and connecting correctly
- [ ] Compare planning time between vanilla RRT and Bi-RRT on the same environment — Bi-RRT should be noticeably faster

---

#### STEP 3 — Add RRT* Rewiring (Saturday May 3 Afternoon)

Add the rewiring step that makes RRT → RRT*. This continuously improves path quality as the tree grows.

- [ ] Add cost tracking to every node — cost = total Euclidean distance from root to that node
- [ ] When adding a new node, search all nearby nodes within rewire radius r
- [ ] For each nearby node, check if reaching it through the new node is cheaper than its current cost
- [ ] If cheaper and path is collision free, update that node's parent and cost (rewire)
- [ ] Also check if the new node itself is better reached through any nearby node (choose minimum cost parent)
- [ ] Implement rewiring for both forward and reverse trees
- [ ] Test that path length measurably improves with rewiring vs without rewiring over 20 runs
- [ ] Recommended rewire radius: 1.5–2.0x step size

---

#### STEP 4 — ROS2 Node Wrapping (Sunday May 4)

Wrap your Bi-RRT* algorithm into a ROS2 node so it can communicate with the robot and the rest of the system.

- [ ] Create a ROS2 Python node file
- [ ] Subscribe to `/map` topic — receive the occupancy grid map from Gazebo
- [ ] Subscribe to `/obstacle_positions` topic — receive moving obstacle positions published by Person 2's moving obstacle script (list of obstacle tuples)
- [ ] Subscribe to `/goal_pose` topic — receive the goal position
- [ ] Subscribe to `/odom` or `/robot_pose` topic — receive the robot's current position (start point)
- [ ] Convert the occupancy grid map into your obstacle list format
- [ ] On receiving a goal, run `weighted_astar()` (from Person 2) to generate `wa_path`
- [ ] Pass `wa_path` to Person 3's `sample_state()` as the sampling corridor
- [ ] Run Bi-RRT* using Person 3's `sample_state()` for sampling
- [ ] Apply Person 3's `smooth_path()` to the result
- [ ] Publish the final path to `/planned_path` topic as a ROS2 nav_msgs/Path message
- [ ] On receiving updated obstacle positions, check `is_path_blocked()` (Person 2)
- [ ] If blocked, call `replan()` (Person 2) and republish the updated path
- [ ] Add basic logging — print planning time and path length to terminal each run
- [ ] Test the node compiles and runs without errors before integration day

---

#### STEP 5 — Integration (Monday May 5)

Lead the full integration of all three modules.

- [ ] Confirm Person 2's functions (`is_path_blocked`, `replan`, `astar`, `weighted_astar`, `cell_to_xy`) work when imported
- [ ] Confirm Person 3's functions (`sample_state`, `smooth_path`) work when imported
- [ ] Plug all functions into your ROS2 node — replace placeholders with real implementations
- [ ] Launch Gazebo world with TurtleBot3 (Person 2's setup)
- [ ] Launch your ROS2 planner node
- [ ] Launch Person 3's RViz visualization node
- [ ] Verify robot receives planned path and starts moving in Gazebo
- [ ] Trigger moving obstacles and verify replanning occurs
- [ ] Debug any integration issues as a team
- [ ] Record a working simulation run once stable

---

#### STEP 6 — Experiments (Monday May 5 Afternoon)

Once integrated, run controlled experiments to collect data for the paper.

- [ ] Run Bi-RRT* (without WA* guidance) 20 times in each of the 3 environments — log planning time, path length, replan count
- [ ] Run WA*-Bi-RRT* (with WA* guidance) 20 times in each of the 3 environments — log same metrics
- [ ] Run both with and without dynamic obstacles
- [ ] Save all results to CSV files for Person 3 to plot

---

#### If Finished Early
- [ ] Help Person 2 with ROS2 node wrapping
- [ ] Add a 4th more complex environment for extra experiments

---

---

# 👤 PERSON 2 — A* + Weighted A* + Replanning + Map + Gazebo
### [Person 2 Name]

---

### 📌 Responsibility Overview
You own everything related to the environment — creating the maps, setting up the simulation, and implementing the grid-based planners (A* and Weighted A*). You also implement the path validity checking and replanning logic that lets the robot react to moving obstacles. Your maps and coordinate conversion function are critical dependencies for the whole team — get those done Friday morning.

---

### 📋 Detailed Task List

#### BEFORE WRITING ANY CODE — Friday May 2 Morning
- [ ] Read the paper (pages 1–4, Section III and IV only)
- [ ] Read and understand the shared data structures from Person 1
- [ ] Read and understand mock_data.py
- [ ] Set up your Python dev environment (numpy, matplotlib)
- [ ] Set up ROS2 Humble + Gazebo + TurtleBot3 on your machine
- [ ] Verify TurtleBot3 launches correctly in an empty Gazebo world

---

#### STEP 1 — Map Creation (Friday May 2 Morning)

Create the environment that everyone works in. Start simple — complexity comes later.

- [ ] Design Environment 1: Simple open room with one L-shaped wall in the middle (10m × 10m)
- [ ] Create the Gazebo `.world` file for Environment 1 — box room with wall models
- [ ] Create the matching grid map for Environment 1 as a 2D numpy array (cell size = 0.5m)
  - Each cell is 0 (free) or 1 (obstacle)
  - Grid dimensions must match the Gazebo world dimensions exactly
- [ ] Implement `cell_to_xy(cell, cell_size=0.5)` — converts grid (row, col) to continuous (x, y) meters
- [ ] Implement `xy_to_cell(pos, cell_size=0.5)` — converts continuous (x, y) to grid (row, col)
- [ ] Verify the grid map visually by plotting it with matplotlib — it must look identical to the Gazebo world
- [ ] Share the map files and conversion functions with the team so everyone can use them for testing
- [ ] Design Environment 2: Corridor world — narrow passage the robot must navigate through (build Sunday)
- [ ] Design Environment 3: Cluttered office — many small obstacles scattered (build Sunday)

---

#### STEP 2 — A* Implementation (Friday May 2 Afternoon)

Implement standard A* on the grid map. This is your first baseline algorithm.

- [ ] Implement the open list (priority queue ordered by f = g + h)
- [ ] Implement the closed list (set of already visited cells)
- [ ] Implement the heuristic function h — use Euclidean distance from current cell to goal cell
- [ ] Implement the main A* search loop:
  - Pop lowest f node from open list
  - If it's the goal, reconstruct and return path
  - Otherwise expand all 8 neighbors (including diagonals)
  - For each neighbor, compute g, h, f and add to open list if not visited
- [ ] Implement path reconstruction — trace back through parent pointers from goal to start
- [ ] Convert the final path from grid cells to (x,y) meters using `cell_to_xy()`
- [ ] Test A* on the mock grid — verify it finds a valid path around obstacles
- [ ] Test A* on Environment 1 map — verify visually with matplotlib
- [ ] Verify the path returned is in (x,y) meters not grid cells

---

#### STEP 3 — Weighted A* Implementation (Saturday May 3 Morning)

Add the epsilon weight parameter to A*. This is your second baseline algorithm.

- [ ] Copy A* implementation and add epsilon parameter (default 1.5)
- [ ] Change heuristic to `h_weighted = epsilon * h`
- [ ] Everything else stays identical to A*
- [ ] Test with multiple epsilon values: 1.0 (= regular A*), 1.5, 2.0, 3.0
- [ ] Verify that higher epsilon = faster planning time but longer path
- [ ] Log planning time and path length for each epsilon value for the paper
- [ ] Make epsilon easily configurable (will be used in experiments)

---

#### STEP 4 — Path Validity Checker (Saturday May 3 Afternoon)

Implement the function that detects when a moving obstacle blocks the current path.

- [ ] Implement `is_path_blocked(path, obstacles)`:
  - Iterate through each segment of the path (consecutive pairs of points)
  - For each segment, check if any obstacle's circle intersects the line segment
  - Use point-to-segment distance formula for accurate collision detection
  - Return `(True, segment_index)` for the first blocked segment found
  - Return `(False, None)` if path is completely clear
- [ ] Test against `mock_obstacles_static` — verify (True, 1) returned
- [ ] Test against `mock_obstacles_moving` — verify (True, 0) returned
- [ ] Test with an obstacle clearly off the path — verify (False, None) returned
- [ ] Test with an empty obstacle list — verify (False, None) returned
- [ ] Test with a single-segment path (just two points)
- [ ] Make sure it handles edge cases: obstacle exactly touching path boundary, very short path segments

---

#### STEP 5 — Replanning Logic (Sunday May 4 Morning)

Implement the function that finds an alternate route when the path is blocked.

- [ ] Implement `replan(tree, blocked_segment, obstacles)`:
  - Identify the nodes in the existing tree near the blocked segment
  - Find the last safe node on the current path before the blocked segment
  - From that safe node, search the existing tree for alternate nodes that bypass the obstacle
  - Connect alternate nodes into a new valid path to the goal
  - Return the new path if found, or None if no alternate route exists
  - Do NOT rebuild the tree from scratch — use existing nodes only
- [ ] Test with `mock_tree` and `mock_obstacles_static` — verify a valid alternate path is returned
- [ ] Test when no alternate exists — verify None is returned gracefully
- [ ] Test that returned path is actually collision free

---

#### STEP 6 — Moving Obstacle Scripts (Sunday May 4 Morning)

Create the moving obstacles that make the environment dynamic.

- [ ] Create a ROS2 Python node that publishes obstacle positions
- [ ] Implement at least 3 moving obstacles with different behaviors:
  - Obstacle 1: moves in a straight line and bounces off walls
  - Obstacle 2: moves in a circular path
  - Obstacle 3: moves randomly (changes direction occasionally)
- [ ] Publish all obstacle positions to `/obstacle_positions` topic as a custom message or array
  - Each obstacle: (x, y, radius, vx, vy)
  - Publish at 10Hz
- [ ] Add Gazebo visual models for each moving obstacle so they are visible in simulation
- [ ] Verify obstacle positions published to ROS2 topic match their visual position in Gazebo
- [ ] Make obstacle speed configurable (for experiments with different speeds)

---

#### STEP 7 — Gazebo Robot Pipeline (Sunday May 4 Afternoon)

Get the TurtleBot3 moving in Gazebo along a planned path.

- [ ] Spawn TurtleBot3 Waffle in Environment 1 Gazebo world at position (0, 0)
- [ ] Implement a path follower node:
  - Subscribe to `/planned_path` topic (published by Person 1's planner node)
  - Convert path waypoints into velocity commands
  - Publish velocity commands to `/cmd_vel` to move the robot
  - Move to each waypoint in sequence, stop when goal reached
- [ ] Test path follower with `hardcoded_path` from mock data — verify robot moves along it in Gazebo
- [ ] Verify robot stops correctly when it reaches the goal
- [ ] Handle the case where the robot slightly overshoots a waypoint
- [ ] Create launch file that starts Gazebo world + TurtleBot3 + obstacle publisher in one command
- [ ] Build Environments 2 and 3 Gazebo worlds + matching grid maps

---

#### STEP 8 — Integration (Monday May 5)

- [ ] Swap hardcoded path for Person 1's real planner output
- [ ] Verify the full pipeline: planner publishes path → robot follows path → obstacles move → replan triggers → robot follows new path
- [ ] Run A* and Weighted A* experiments:
  - 20 runs each in all 3 environments
  - Log: planning time, path length
  - Test with epsilon = 1.0, 1.5, 2.0, 3.0 for Weighted A*
  - Save results to CSV

---

#### If Finished Early
- [ ] Help Person 1 with ROS2 node wrapping
- [ ] Add obstacle speed variation experiments

---

---

# 👤 PERSON 3 — Sampling + Smoothing + Visualization + Metrics
### [Person 3 Name]

---

### 📌 Responsibility Overview
You own the originality contribution of the project — the WA*-guided sampling strategy that makes WA*-Bi-RRT* faster than vanilla Bi-RRT*. You also own path smoothing, RViz visualization, and all metrics logging and plotting. Your `sample_state()` function is what makes this project novel — it needs to be solid and well-tested.

---

### 📋 Detailed Task List

#### BEFORE WRITING ANY CODE — Friday May 2 Morning
- [ ] Read the paper (pages 1–4, Section III and IV only)
- [ ] Read and understand the shared data structures from Person 1
- [ ] Read and understand mock_data.py
- [ ] Set up your Python dev environment (numpy, matplotlib)
- [ ] Set up ROS2 + RViz on your machine
- [ ] Verify RViz launches and displays a basic marker

---

#### STEP 1 — RViz Basic Setup (Friday May 2)

Get RViz ready to visualize the planning process before the algorithm exists.

- [ ] Create a ROS2 Python visualization node
- [ ] Set up RViz configuration file (.rviz) with the panels you need
- [ ] Implement publisher for forward tree edges — blue lines in RViz (MarkerArray)
- [ ] Implement publisher for reverse tree edges — green lines in RViz (MarkerArray)
- [ ] Implement publisher for WA* corridor path — yellow line in RViz (Marker)
- [ ] Implement publisher for final planned path — red line in RViz (Marker)
- [ ] Implement publisher for start point — green sphere marker
- [ ] Implement publisher for goal point — blue sphere marker
- [ ] Implement publisher for obstacle positions — grey sphere markers, update in real time
- [ ] Test all publishers with hardcoded mock data — verify everything displays correctly in RViz before the real algorithm exists
- [ ] Make sure the visualization updates in real time as the tree grows (subscribe to tree update topic)

---

#### STEP 2 — Basic Sampling (Saturday May 3)

Implement the first two sampling behaviors. These are the foundation that the WA* guidance builds on top of.

- [ ] Implement `random_sample(map_size)` — returns a uniformly random (x,y) within map bounds
- [ ] Implement `goal_biased_sample(goal)` — returns the goal point directly
- [ ] Implement the probability switch in `sample_state()`:
  - Generate random number p between 0 and 1
  - If p < 0.15, return goal (goal bias)
  - Otherwise return random sample
- [ ] Test standalone with mock_tree — call sample_state() 1000 times and verify ~15% of samples are exactly the goal point
- [ ] Plot the distribution of 1000 samples with matplotlib — verify uniform coverage of the map

---

#### STEP 3 — WA* Corridor Guidance (Sunday May 4 Morning)

Add the core originality contribution — guided sampling along the WA* path corridor. This is what makes WA*-Bi-RRT* different from vanilla Bi-RRT*.

- [ ] Implement `sample_near_corridor(wa_path, corridor_width)`:
  - Pick a random waypoint from wa_path
  - Sample a random point within corridor_width radius of that waypoint
  - Make sure the sampled point stays within map bounds
  - Return the sampled (x,y) point
- [ ] Update the probability switch in `sample_state()`:
  - p < 0.60 → sample near WA* corridor (guided)
  - p < 0.75 → sample toward goal (goal bias)
  - p < 0.90 → sample away from obstacle projections (bias — next step)
  - else → pure random sample
- [ ] Test with mock_wa_path — plot 1000 samples and verify they cluster around the WA* corridor
- [ ] Test with corridor_width = 0.5, 1.0, 2.0 — visually verify wider corridor = more spread
- [ ] Verify that when wa_path is empty or None, falls back to pure random sampling gracefully

---

#### STEP 4 — Obstacle Projection Bias (Sunday May 4 Morning)

Add the dynamic safety bias — steer sampling away from where obstacles are predicted to be.

- [ ] Implement `predict_obstacle_position(obstacle, steps_ahead=5)`:
  - Given obstacle (x, y, radius, vx, vy), predict where it will be in steps_ahead steps
  - Return predicted (x, y) position
- [ ] Implement `sample_away_from_obstacles(obstacles, map_size, steps_ahead=5)`:
  - Predict positions of all obstacles steps_ahead into the future
  - Generate a random candidate sample
  - Reject the sample if it's within safety_margin of any predicted obstacle position
  - Keep trying until a safe sample is found (max 10 attempts, then return best)
  - Return the safe sample
- [ ] Integrate into the probability switch — p < 0.90 → call sample_away_from_obstacles()
- [ ] Test with mock_obstacles_moving — plot 1000 samples and verify they avoid predicted obstacle areas
- [ ] Tune safety_margin (recommend starting at 2x obstacle radius)

---

#### STEP 5 — Path Smoothing (Sunday May 4 Afternoon)

Implement the shortcutting algorithm that cleans up the jagged RRT path.

- [ ] Implement `smooth_path(path, obstacles)`:
  - Start with i = 0 (first node)
  - Try to connect node i directly to node i+2, i+3, etc. skipping intermediate nodes
  - Use `is_collision_free_path()` to check if the shortcut is clear
  - If clear, remove the intermediate nodes and update i
  - If not clear, advance i by 1 and try again
  - Continue until no more shortcuts can be made
  - Return the smoothed path
- [ ] Test with a straight line path and no obstacles — verify all intermediate nodes are removed
- [ ] Test with obstacles in the way — verify intermediate nodes are kept where needed
- [ ] Measure path length before and after smoothing — verify smoothing always reduces or maintains path length
- [ ] Test that smoothed path is still collision free

---

#### STEP 6 — Metrics Logger (Monday May 5)

Build the metrics system that collects data from all 5 algorithms for the paper.

- [ ] Create `logger.py` with a `MetricsLogger` class
- [ ] Implement `start_timer()` — records planning start time
- [ ] Implement `stop_timer()` — records planning end time, computes planning time in seconds
- [ ] Implement `log_path(path)` — computes and stores total path length in meters
- [ ] Implement `log_replan()` — increments replan counter by 1
- [ ] Implement `log_run(algorithm_name, environment_name, success)` — saves one complete run result
- [ ] Implement `save_to_csv(filename)` — saves all logged runs to a CSV file with columns:
  - algorithm, environment, run_number, planning_time, path_length, replan_count, success, epsilon (for WA*)
- [ ] Test logger standalone — simulate 5 fake runs and verify CSV is written correctly
- [ ] Integrate logger into Person 1's ROS2 node (coordinate with Person 1 on Monday)
- [ ] Run all 5 algorithms, 20 runs each, in all 3 environments — save results

---

#### STEP 7 — Results Plots (Tuesday May 6)

Generate the comparison plots for the paper from the collected CSV data.

- [ ] Load all CSV results into pandas or numpy
- [ ] Plot 1: Bar chart — average planning time for all 5 algorithms across 3 environments
- [ ] Plot 2: Bar chart — average path length for all 5 algorithms across 3 environments
- [ ] Plot 3: Line plot — planning time vs epsilon for Weighted A* (epsilon = 1.0, 1.5, 2.0, 3.0)
- [ ] Plot 4: Line plot — path length vs epsilon for Weighted A*
- [ ] Plot 5: Bar chart — replan count for dynamic obstacle experiments (RRT variants only)
- [ ] Plot 6: Bar chart — success rate for all 5 algorithms with dynamic obstacles
- [ ] Save all plots as high-resolution PNG files for the paper
- [ ] Make sure all plots have proper axis labels, titles, and legends

---

#### If Finished Early
- [ ] Help Person 2 with `replan()` implementation
- [ ] Add corridor width experiment — run WA*-Bi-RRT* with corridor_width = 0.5, 1.0, 2.0, 3.0 and plot effect on planning time

---

---

## 📅 Timeline Summary

| Day | Person 1 | Person 2 | Person 3 |
|---|---|---|---|
| Fri May 2 | Share data structures. Vanilla RRT done. | Share maps. A* done. Gazebo basic setup. | RViz setup. Basic sampling started. |
| Sat May 3 | Bi-RRT* + rewiring done. | Weighted A* done. is_path_blocked() done. | Basic sampling done. WA* guidance started. |
| Sun May 4 | ROS2 node done. | replan() done. Moving obstacles. Robot following path. Envs 2+3. | WA* guidance + obstacle bias + smooth_path + full RViz done. |
| **Mon May 5** | **INTEGRATION DAY — everyone available all day** | | |
| Tue May 6 | Support paper writing | Support paper writing | Generate all plots. Paper + slides. |
| Wed May 7 | Buffer — polish and submit | | |
| Thu May 8 | 🚨 Hard deadline @ 11:59PM | | |

---

## 📄 Paper Outline (IEEE Format, 6-8 pages)

```
I.   Introduction
II.  Background — A*, Weighted A*, RRT, Bi-RRT*
III. Proposed Algorithm — WA*-Bi-RRT*
IV.  Experiments — 5 algorithms × 3 environments × 20 runs
V.   Results & Discussion
VI.  Conclusion
```

---

## 📏 Metrics Collected (All 5 Algorithms, 20 runs each)

| Metric | Description |
|---|---|
| Planning time (s) | Time to first valid path |
| Path length (m) | Total Euclidean length of smoothed path |
| Replan count | Times path was blocked and rerouted |
| Success rate (%) | % of runs with valid path found |
| Path optimality | Path length / A* optimal path length |

---

## ⚠️ Ground Rules

1. Read data structures + mock data **Friday morning before writing any code**
2. **Do NOT change function signatures**
3. Test your functions **standalone with mock data** before Mon May 5
4. **Coordinate system:** everything in meters — A\* must convert grid cells to (x,y) meters
5. Stuck for **more than 2 hours** → message the group immediately
6. Finish early → pick up the escalation task in your section
