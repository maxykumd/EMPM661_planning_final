MAP_SIZE      = (6.0, 6.0)
CELL_SIZE     = 0.1
STEP_SIZE     = 0.20
GOAL_BIAS     = 0.10
REWIRE_RADIUS = 0.45
ROBOT_RADIUS  = 0.10
SAFETY_MARGIN = 0.10
CLEARANCE     = ROBOT_RADIUS + SAFETY_MARGIN  # 0.20m total
SIGMA         = 0.3

START = (0.3, 5.5)
GOAL  = (5.5, 0.3)

# Static obstacles match planning_world.world
OBSTACLES = [
    (1.5, 1.5, 0.25, 0.0, 0.0),
    (4.5, 1.5, 0.25, 0.0, 0.0),
    (2.5, 4.5, 0.25, 0.0, 0.0),
]

# Moving obstacles initial positions
MOVING_OBSTACLES = [
    (3.0, 1.5, 0.20, 0.08, 0.0),
    (1.5, 3.5, 0.20, 0.0,  0.06),
    (3.5, 3.5, 0.20, 0.05, 0.05),
]