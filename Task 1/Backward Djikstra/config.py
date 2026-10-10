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
MIN_CLEARANCE_CM = 7.0
POSITION_MARGIN_CM = 0.0  # provisional allowance, not a measured error bound
BOUNDARY_MARGIN_CM = 0.0  # separate from obstacle clearance
START_POSE_CM = (15.0, 15.0, "N")
# Unchanged client sends this old default. Server treats it as selection of
# START_POSE_CM above; other legacy starts are converted as cell centres.
LEGACY_DEFAULT_START = (1, 1, "N")
DIRECTION_STEP = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
# (forward cm, right cm, clockwise quarter-turns): modal measured endpoints.
TURN_MODELS = {
    "RT90": (20, 35, 1),  "LT90": (16, -30, -1),
    "XL90": (-35, -20, 1), "XR90": (-40, 28, -1),
}
ALL_MOVES = ("FW", "BW", "LT90", "RT90", "XR90", "XL90")
STRAIGHT_STEP_CM = 1
MAX_STRAIGHT_COMMAND_CM = 100  # long drives split for STM timeout
STRAIGHT_COST_PER_CM = {"FW": 0.10, "BW": 0.15}
TURN_COST = {"LT90": 4.0, "RT90": 4.0, "XL90": 20.0, "XR90": 10.0}
TURN_SAMPLE_DEGREES = 3.0
MAX_SEARCH_EXPANSIONS = 180000
PLAN_TIME_LIMIT_S = 59.0  # below unchanged client's 60-second timeout
ERROR_CORRECTION_ENABLED = False  # v5: not used (no angle correction in the v5 photo flow)
ALIGN_ENABLED = False
ALIGN_TARGET_CM = 10
SENSOR_FORWARD_CM = 12.0  # Center-sensor distance
# Maximum TOTAL distance AC20 may move from where it starts, adding up
# every step in both directions. The planner keeps this strip clear.
# The STM firmware enforces the same 10 cm cap (AC_MAX_TOTAL_CM in stm_code.c).
ALIGN_MAX_TRAVEL_CM = 10
VIEW_SENSOR_DISTANCES_CM = tuple(range(20,40))  #Sensor to face of obstacle distance 
VIEW_LATERAL_OFFSETS_CM = (-2, -1, 0, 1, 2)  # Lateral offset from center of obstacle (cm)
LATERAL_COST_PER_CM = 2.0  # Cost penalty per cm of lateral offset (0cm = 0, +-1cm = 1.0, +-2cm = 2.0)
# v5 distance preference (added on top of the lateral penalty above).
# Sensor->face 30 cm costs 0 and is tried first. Each cm further away (robot
# sits further back) costs 1.0, each cm closer (robot further forward) costs 2.0,
# so further is preferred over closer. Set VIEW_PREFERRED_SENSOR_CM = None for
# the old behaviour (every distance equal).
VIEW_PREFERRED_SENSOR_CM = 30
VIEW_FURTHER_COST_PER_CM = 1.0
VIEW_NEARER_COST_PER_CM = 2.0
VIEW_POSITION_TOLERANCE_CM = 0.50 # allows half-cm start offsets without snapping state
SNAP_PREFIX = "SNAP"

# ---------------------------------------------------------------------------
# v5 photo retries (used by the RPi; the planner works out how far it is safe
# to drive for them and sends it inside each segment's "target").
#
# The extra photos are taken by driving STRAIGHT along the photo heading only,
# then driving back, so the planned route is never changed.
#   FAR  photo : BW from the planned photo pose, out to SNAP_FAR_MAX_SENSOR_CM
#   NEAR photo : FW from the planned photo pose, in to SNAP_NEAR_MIN_SENSOR_CM
# Each is cut short if MIN_CLEARANCE_CM (any obstacle) or the arena wall would
# be broken on the way. A bound with less than SNAP_RETRY_MIN_MOVE_CM of room
# is reported as "no room" and that photo is skipped.
# ---------------------------------------------------------------------------
SNAP_FAR_MAX_SENSOR_CM = max(VIEW_SENSOR_DISTANCES_CM)   # furthest retry photo (sensor -> face, cm)
SNAP_NEAR_MIN_SENSOR_CM = 12                             # nearest retry photo (sensor -> face, cm)
SNAP_RETRY_MIN_MOVE_CM = 2                               # skip a retry photo that would move less than this

# Image recognition thresholds (RPi only).
MODEL_MIN_CONF = 0.10     # boxes below this are dropped by YOLO itself (they never count)
RECOGNISED_CONF = 0.50    # CASE A needs one image at or above this; below it -> CASE B retries
ANDROID_MIN_CONF = 0.50   # only results at or above this are sent to Android (everything is shown in the terminal)

# ---------------------------------------------------------------------------
# v7 CASE C (IDEAL photo found no image) backup: FAR photo, then one angled photo.
# The robot turns towards the side where the obstacle's dark body shows in the
# photo: body on the RIGHT -> RT<deg> (camera swings right), LEFT -> LT<deg>.
# The turn is undone with XR<deg> / XL<deg> and the robot drives back to IDEAL.
# ---------------------------------------------------------------------------
ANGLE_TURN_DEG = 20       # >= ~20 needed: the STM stops at (angle - 14..18 deg) and smaller turns do nothing
BODY_CENTRE_BAND = 0.15   # no "centre": anything left of the middle = LEFT, right = RIGHT (like v4)
BODY_MIN_AREA_PX = 800    # same value the old v4 correction used

"""
Alternative
BODY_CENTRE_BAND = 0.00   # dark body centre within +-15 % of frame width of the middle = "centred"
BODY_MIN_AREA_PX = 800    # smaller dark regions are ignored (shadows, specks)
"""

# Small turns as (forward cm, right cm, clockwise degrees) from the pose the turn starts at.
# PLACEHOLDERS: each 90-degree TURN_MODELS entry cut at 20 degrees with the same
# ellipse the planner already uses (forward = a*sin t, right = r*(1 - cos t)).
# Measure the real turns and replace these numbers.
SMALL_TURN_MODELS = {
    "RT20": (6.84, 2.11, 20),     # from RT90 (20, 35)
    "LT20": (5.47, -1.81, -20),   # from LT90 (16, -30)
    "XR20": (-13.68, 1.69, -20),  # from XR90 (-40, 28)   undoes RT20
    "XL20": (-11.97, -1.21, 20),  # from XL90 (-35, -20)  undoes LT20
}
