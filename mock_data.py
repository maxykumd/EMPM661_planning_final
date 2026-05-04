# mock_data.py
# Shared test data

MAP_SIZE = (20.0, 20.0)   # map is 20m x 20m
CELL_SIZE = 0.5            # each grid cell = 0.5 meters
STEP_SIZE = 0.5            # how far RRT moves each step
GOAL_BIAS = 0.15           # 15% chance of sampling the goal directly
REWIRE_RADIUS = 1.0        # radius for RRT* rewiring

ROBOT_RADIUS   = 0.10   # TurtleBot3 Waffle ≈ 0.20m radius
SAFETY_MARGIN  = 0.10   # extra buffer for safety
CLEARANCE      = ROBOT_RADIUS + SAFETY_MARGIN  # total = 0.30m

# A simple path for testing
mock_path = [(0,0), (2,2), (4,4), (6,6), (8,8), (10,10)]

# Some obstacles for testing
# Format: (x, y, radius, vx, vy)
# vx, vy = velocity — 0.0 means static
mock_obstacles_static = [
    (5.0, 5.0, 0.3, 0.0, 0.0),    # sits right on the mock path
    (15.0, 15.0, 0.3, 0.0, 0.0),  # off the path
]
mock_obstacles_moving = [
    (2.0, 2.0, 0.3, 0.5, 0.0),    # moving right
    (15.0, 15.0, 0.3, 0.1, 0.1),  # off path
]

# A simple tree for testing
# Format: {"pos": (x,y), "parent": node or None, "cost": float}
mock_tree = [
    {"pos": (0,0),  "parent": None, "cost": 0.0},
    {"pos": (1,0),  "parent": None, "cost": 1.0},
    {"pos": (2,0),  "parent": None, "cost": 2.0},
    {"pos": (1,1),  "parent": None, "cost": 1.41},
]

# Start and goal for testing
START = (0.3, 0.3)
GOAL  = (16.5, 16.5)

# Simple obstacles for RRT testing
# A few circles scattered around the map
OBSTACLES = [
    (5.0,  5.0,  0.8, 0.0, 0.0),
    (10.0, 5.0,  0.8, 0.0, 0.0),
    (5.0,  10.0, 0.8, 0.0, 0.0),
    (10.0, 10.0, 0.8, 0.0, 0.0),
    (15.0, 5.0,  0.8, 0.0, 0.0),
    (8.0,  15.0, 0.8, 0.0, 0.0),
    # (10.0, 6.0,  0.8, 0.0, 0.0),
    # (10.0, 7.0,  0.8, 0.0, 0.0),

]