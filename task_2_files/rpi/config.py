"""Map input stays in cells; planner distances are centimetres.
"""
"""
Flow 
1. If align_enabled is false ( ultrasonic sensor ) is off, view_sensor_dist + sensor_forward_cm  = Dist to obstacle from center of robot
2. If align_enabled is true ( ultrasonic sensor ) is on, view_sensor_dist + sensor_forward_cm = Dist to obstacle from center of robot
But, if view sensor_dist is more than +-2 cm off, then align_target_cm will be triggered where it will make adjustment such that it is within 18-22cm
from the obstacle, with constrains like the maximum it can move being 10 cm in total, and min clearance of 5cm from any obstacle at all sides being obeyed
"""
# Settings corresponding to the ultrasonic sensor is - align_enabled, align_target_cm, align_max_travel_cm, sensor_forward

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
    "RT90": (23, 42, 1),  "LT90": (19, -32, -1),
    "XL90": (-18, -31, 1), "XR90": (-39, 28, -1),
}
# v8: 45-degree turns, (forward cm, right cm, clockwise EIGHTH-turns: +1 = 45 deg clockwise).
# Same sign convention as TURN_MODELS above: negative forward = backwards,
# negative right = to the left. The STM already takes LT45 / RT45 / XL45 / XR45
# (it reads the angle after LT / RT / XL / XR).
# PLACEHOLDERS, worked out from your 90-degree TURN_MODELS so that two 45s in a
# row end exactly where one 90 ends (e.g. RT45 then RT45 = RT90 = 20 fwd, 35 right).
# Measure the real turns (10 runs each) and replace these numbers. The planner
# rounds each turn's end point to whole cm and adds that rounding to the
# clearance it checks during the turn.
# (forward cm, right cm, clockwise quarter-turns): modal measured endpoints.
TURN45_MODELS = {
    "RT45": (18, 9, 1),    # 2 x RT45 = RT90 (20, 35)
    "LT45": (16, -11, -1),  # edited
    "XL45": (-23, 4, 1),   # edited
    "XR45": (-23, 5, -1),   # 2 x XR45 = XR90 (-40, 28)
}
# Remove the four 45s from ALL_MOVES to plan with 90-degree turns only (v7 routes).
ALL_MOVES = ("FW", "BW", "LT90", "RT90", "XR90", "XL90")
STRAIGHT_STEP_CM = 1
MAX_STRAIGHT_COMMAND_CM = 100  # long drives split for STM timeout
STRAIGHT_COST_PER_CM = {"FW": 0.10, "BW": 0.15}
# v8: each 45-degree turn costs half its 90-degree turn.
TURN_COST = {"LT90": 4.0, "RT90": 4.0, "XL90": 20.0, "XR90": 10.0, "LT45": 4.0, "RT45": 4.0, "XL45": 4.0, "XR45": 4.0}
# v8 collision check while driving / turning:
#   "polygon" = the real rotated 19 x 24 cm body (needed for diagonals to fit)
#   "box"     = v7's axis-aligned boxes (more conservative mid-turn and on diagonals)
# Straight moves facing N / E / S / W are identical either way.
COLLISION_SHAPE = "polygon"
TURN_SAMPLE_DEGREES = 3.0
MAX_SEARCH_EXPANSIONS = 180000  # v8: not used by the scipy search (kept so old tools load)
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

# Pi terminal: False = short layout per obstacle (Pose / Sensor / Scan / Fallback /
# Evaluating / Logic / Result); the full printout goes to photos/Trial_<t>_detail.txt.
# True = the full printout on screen as before.
TERMINAL_DETAIL = False

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
SNAP_NEAR_MIN_SENSOR_CM = min(VIEW_SENSOR_DISTANCES_CM)                             # nearest retry photo (sensor -> face, cm)
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
# v8: the angled photo turn is 10 degrees and it is used ONLY by this CASE C
# fallback, never by the route planner (it is not in ALL_MOVES).
ANGLE_TURN_DEG = 10
# CASE C (IDEAL found no image):
#   True  -> FAR photo; if FAR has no image >= RECOGNISED_CONF, one angled RT10/LT10 photo (max 3).
#            Report the highest-confidence image over all the photos if >= RECOGNISED_CONF, else nothing.
#   False -> FAR photo only (max 2). Report FAR's highest-confidence image if >= RECOGNISED_CONF,
#            else nothing. No 10-degree turns at all.
CASE_C_ANGLED_FALLBACK = False
BODY_CENTRE_BAND = 0.15   # no "centre": anything left of the middle = LEFT, right = RIGHT (like v4)
BODY_MIN_AREA_PX = 800    # same value the old v4 correction used

"""
Alternative
BODY_CENTRE_BAND = 0.00   # dark body centre within +-15 % of frame width of the middle = "centred"
BODY_MIN_AREA_PX = 800    # smaller dark regions are ignored (shadows, specks)
"""

# v8 fallback turns as (forward cm, right cm, clockwise degrees) from the pose the turn starts at.
# Only the two FORWARD turns are entered here. The undo (XR10 after RT10, XL10 after
# LT10) drives the SAME turning circle in reverse, so it retraces the turn exactly and
# the robot ends back on the spot where the turn started. The planner works the undo
# out from these entries - there is nothing to enter for XR10 / XL10.
# PLACEHOLDERS: the 90-degree TURN_MODELS cut at 10 degrees with the planner's ellipse
# (forward = a*sin 10, right = r*(1 - cos 10)). Replace with your measured LT10 / RT10.
SMALL_TURN_MODELS = {
    "RT10": (3, 2, 10),     # from RT90 (20, 35)
    "LT10": (3, -2, -10),   # from LT90 (16, -30)
}

# ---------------------------------------------------------------------------
# v8.1 winning-photo popup (rpi_client.py -> image_server.py on the laptop)
# ---------------------------------------------------------------------------
IMAGE_PORT = 5001          # laptop port for the popup (allow it in the Windows firewall)
STREAM_TIMEOUT_S = 8.0     # per send attempt (runs in the background, the robot never waits)
STREAM_RETRIES = 5         # attempts per photo before it is parked for the end-of-run re-send
STREAM_FLUSH_S = 30.0      # end of run: time allowed to deliver / re-send the rest


# ---------------------------------------------------------------------------
# Task 2 (fastest car) - used as defaults by task2.run_task2()
# Everything marked MEASURE is a placeholder until you measure it on your robot.
# The turn models above (TURN45_MODELS, SMALL_TURN_MODELS) drive every Task 2
# move - measure LT45 / RT45 / RT10 / LT10 (10 runs each) before trusting a run.
# ---------------------------------------------------------------------------
# Task 2 body clearance limits (cm); preferred clearance is used when feasible.
T2_MIN_CLEARANCE_CM = 5.0
T2_CARPARK_CLEARANCE_CM = 5.0
T2_TARGET_CLEARANCE_CM = 10.0
T2_AC_OBS1_CM = 40.0             # ultrasonic -> obstacle 1 face when reading arrow 1
T2_IR_GAP_OBS1_CM = 10.0         # target side-IR reading while passing obstacle 1
# Robot centre at the start -> carpark front plane. From the rules (inside depth 50 cm):
# start with the robot's REAR against the back wall -> 50 - ROBOT_LENGTH_CM/2 = 38.
# Set a few cm LOW on purpose: underestimating by up to 20 cm is safe (the ultrasonic
# corrects it), overestimating by 20 cm is not.
T2_CARPARK_HALF_CM = 20.0        # half the carpark's INSIDE width (40 cm inside, arms 10 cm each)
T2_CARPARK_DEPTH_CM = 50.0       # carpark inside depth MINIMUM; the real one is measured when parking
T2_CP_FRONT_FROM_START_CM = T2_CARPARK_DEPTH_CM - ROBOT_LENGTH_CM / 2 - 5.0   # = 33
T2_IR_FWD_CM = 4.5               # MEASURE: side IRs' forward offset from robot centre (+ = ahead)
T2_IR_INSET_CM = 1.5            # MEASURE: how far each IR's face sits inside the robot's side
T2_IR_LAG_CM = 3.0               # MEASURE: how far past an edge the IR "spike" registers
T2_U2_FAR_END_CM = 18.5          # obstacle 2 far end assumed when the ultrasonic missed it (short)
T2_AC_STOP_CARPARK_CM = 70.0     # stop distance in front of the carpark arm before turning in
T2_CARPARK_ARM_CM = 10.0         # MEASURE: thickness of the carpark arm's front edge