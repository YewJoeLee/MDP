"""Map input stays in cells; planner distances are centimetres.
Read constraints.md before hardware use. Restart laptop server after edits.
"""
GRID = 20
CELL_CM = 10
ROBOT_WIDTH_CM = 19.0
ROBOT_LENGTH_CM = 24.0
MIN_CLEARANCE_CM = 5.0
POSITION_MARGIN_CM = 1.0  # provisional allowance, not a measured error bound
BOUNDARY_MARGIN_CM = 0.0  # separate from obstacle clearance
START_POSE_CM = (15.0, 15.0, "N")
# Unchanged client sends this old default. Server treats it as selection of
# START_POSE_CM above; other legacy starts are converted as cell centres.
LEGACY_DEFAULT_START = (1, 1, "N")
DIRECTION_STEP = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
# (forward cm, right cm, clockwise quarter-turns): modal measured endpoints.
TURN_MODELS = {
    "RT90": (31, 47, 1),  "LT90": (20, -34, -1),
    "XL90": (-31, -25, 1), "XR90": (-47, 27, -1),
}
ALL_MOVES = ("FW", "BW", "LT90", "RT90", "XL90", "XR90")
STRAIGHT_STEP_CM = 1
MAX_STRAIGHT_COMMAND_CM = 90  # long drives split for STM timeout
STRAIGHT_COST_PER_CM = {"FW": 0.10, "BW": 0.15}
TURN_COST = {"LT90": 4.0, "RT90": 4.0, "XL90": 15.0, "XR90": 15.0}
TURN_SAMPLE_DEGREES = 3.0
MAX_SEARCH_EXPANSIONS = 180000
PLAN_TIME_LIMIT_S = 45.0  # below unchanged client's 60-second timeout
ALIGN_ENABLED = False
ALIGN_TARGET_CM = 25
SENSOR_FORWARD_CM = 12.0  # assumed sensor distance ahead of centre: measure!
# Maximum TOTAL distance AC25 may move from where it starts, in either
# direction, across all its attempts. The planner keeps this strip clear.
# Only valid once the STM firmware enforces the same cap (see CHANGES.md).
# Old uncapped firmware (4 x 15 cm) corresponds to 60.
ALIGN_MAX_TRAVEL_CM = 10
VIEW_SENSOR_DISTANCES_CM = (25,)
VIEW_POSITION_TOLERANCE_CM = 0.5  # allows half-cm start offsets without snapping state
SNAP_PREFIX = "SNAP"
