MAP_SIZE      = (6.0, 6.0)
CELL_SIZE     = 0.10
STEP_SIZE     = 0.15
GOAL_BIAS     = 0.10
REWIRE_RADIUS = 0.60
ROBOT_RADIUS  = 0.10
SAFETY_MARGIN = 0.10
CLEARANCE     = ROBOT_RADIUS + SAFETY_MARGIN  # 0.20m total
SIGMA         = 0.2
START = (0.3, 5.5)
GOAL  = (5.5, 0.3)

# ── OBSTACLES ─────────────────────────────────────────────────────────────────
# Must match BOTH the publisher AND the world file exactly.
# Format: (x, y, radius, vx, vy)
OBSTACLES = [
    (2.8, 5.0,  0.65, 0.0, 0.0),   # wall top (large)
    (2.8, 4.0,  0.35, 0.0, 0.0),   # wall upper-mid
    (2.8, 3.0,  0.35, 0.0, 0.0),   # wall lower-mid — diagonal crosses HERE
    (2.8, 1.0,  0.35, 0.0, 0.0),   # wall bottom — gap above at y≈2.0
    (2.0, 1.0,  0.40, 0.0, 0.0),   # bottom blocker
    (3.8, 1.25, 0.45, 0.0, 0.0),   # guardian
]

MOVING_OBSTACLES = [
    [1.5, 2.0, 0.20, 0.0, 0.0, 'moving_obs_1'], #mid
    [1.5, 3.5, 0.20, 0.0, 0.0, 'moving_obs_2'],#top
    [4.5, 2.5, 0.20, 0.0, 0.0, 'moving_obs_3'],
]

