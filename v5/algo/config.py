"""Map input stays in cells; planner distances are centimetres.
"""
"""
Flow 
1. If align_enabled is false ( ultrasonic sensor ) is off, view_sensor_dist + sensor_forward_cm  = Dist to obstacle from center of robot
2. If align_enabled is true ( ultrasonic sensor ) is on, view_sensor_dist + sensor_forward_cm = Dist to obstacle from center of robot
But, if view sensor_dist is more than +-2 cm off, then align_target_cm will be triggered where it will make adjustment such that it is within 18-22cm
from the obstacle, with constrains like the maximum it can move being 10 cm in total, and min clearance of 5cm from any obstacle at all sides being obeyed
"""
GRID = 20
CELL_CM = 10
ROBOT_WIDTH_CM = 19.0
ROBOT_LENGTH_CM = 24.0
MIN_CLEARANCE_CM = 5.0
POSITION_MARGIN_CM = 0.0  # provisional allowance, not a measured error bound
BOUNDARY_MARGIN_CM = 0.0  # separate from obstacle clearance
START_POSE_CM = (15.0, 15.0, "N")
# Unchanged client sends this old default. Server treats it as selection of
# START_POSE_CM above; other legacy starts are converted as cell centres.
LEGACY_DEFAULT_START = (1, 1, "N")
DIRECTION_STEP = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
# (forward cm, right cm, clockwise quarter-turns): modal measured endpoints.
TURN_MODELS = {
    "RT90": (24, 40, 1),  "LT90": (16, -26, -1),
    "XL90": (-26, -17, 1), "XR90": (-39, 26, -1),
}
ALL_MOVES = ("FW", "BW", "LT90", "RT90", "XL90", "XR90")
STRAIGHT_STEP_CM = 1
MAX_STRAIGHT_COMMAND_CM = 100  # long drives split for STM timeout
STRAIGHT_COST_PER_CM = {"FW": 0.10, "BW": 0.15}
TURN_COST = {"LT90": 4.0, "RT90": 4.0, "XL90": 5.0, "XR90": 5.0}
TURN_SAMPLE_DEGREES = 3.0
MAX_SEARCH_EXPANSIONS = 180000
PLAN_TIME_LIMIT_S = 59.0  # below unchanged client's 60-second timeout
ERROR_CORRECTION_ENABLED = True  # True: at most one correction per obstacle; False: none
ALIGN_ENABLED = False
ALIGN_TARGET_CM = 10
SENSOR_FORWARD_CM = 12.0  # Center-sensor distance
# Maximum TOTAL distance AC20 may move from where it starts, adding up
# every step in both directions. The planner keeps this strip clear.
# The STM firmware enforces the same 10 cm cap (AC_MAX_TOTAL_CM in stm_code.c).
ALIGN_MAX_TRAVEL_CM = 10
VIEW_SENSOR_DISTANCES_CM = (15,16,17,18,19,20) #Sensor to face of obstacle distance 
VIEW_POSITION_TOLERANCE_CM = 3.00 # allows half-cm start offsets without snapping state
SNAP_PREFIX = "SNAP"