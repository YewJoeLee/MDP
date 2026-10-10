"""
config.py
---------
Every number the project uses lives here, and nowhere else.
planner.py reads everything from this file - change a value here and the
planner follows. Nothing in planner.py should be a hardcoded number.

Grid coordinates: (0,0) is the BOTTOM-LEFT cell. x grows RIGHT, y grows UP.
"N" (north) means +y. "E" (east) means +x.
"""

# --- the arena -------------------------------------------------------
GRID = 20               # 20 x 20 cells
CELL_CM = 10            # each cell is 10 cm, so the arena is 200 x 200 cm
START_ZONE_SIZE = 4     # start zone is cells (0,0) to (3,3) = 40 x 40 cm

# --- compass ---------------------------------------------------------
# How one step "forward" changes (x, y) for each facing direction.
# Left / right / opposite are worked out from this in planner.py.
DIRECTION_STEP = {
    "N": (0, 1),
    "E": (1, 0),
    "S": (0, -1),
    "W": (-1, 0),
}

# --- the robot -------------------------------------------------------
# The real car is 20 x 21 cm. We model it as a 3 x 3 block of cells
# (30 x 30 cm), which already builds in 5 cm of margin on every side.
ROBOT_SIZE = 3
ROBOT_REACH = 1         # body covers centre_x - 1 .. centre_x + 1
START_POSE = (1, 1, "N")

# Cells moved by ONE straight move (FW or BW).
STRAIGHT_CELLS = 1

# Each turn has its own radius on the real car, so each gets its own numbers.
# All values are in cells, measured from where the turn starts, relative to
# the way the car is facing. Always enter POSITIVE numbers.
#   AHEAD / BACK = how far it ends up forward / backward
#   SIDE         = how far it ends up to the left / right (as the name says)
FORWARD_RIGHT_TURN_AHEAD = 2
FORWARD_RIGHT_TURN_SIDE = 4
FORWARD_LEFT_TURN_AHEAD = 2
FORWARD_LEFT_TURN_SIDE = 4
REVERSE_RIGHT_TURN_BACK = 4
REVERSE_RIGHT_TURN_SIDE = 2
REVERSE_LEFT_TURN_BACK = 4
REVERSE_LEFT_TURN_SIDE = 2

# Points sampled along each turn's arc to check it is collision free.
TURN_SWEEP_FRACTIONS = [0.25, 0.5, 0.75]

# --- obstacles -------------------------------------------------------
# Extra empty cells demanded between the robot's body and any obstacle.
# 1 cell = 10 cm here, on top of the 5 cm the robot model already adds,
# so the real car keeps a 15 cm gap.
SAFETY_CELLS = 1

# How far in front of an image the robot stands to photograph it, in cells.
# Change to 3 for a closer photo; the align command follows automatically
# (4 -> AC25, 3 -> AC15 with the -15 offset below).
VIEW_DISTANCE = 4

# If the nominal viewing pose is blocked or outside the arena, the planner
# tries these instead, best first. Distances are in cells from the obstacle.
VIEW_DISTANCES = [VIEW_DISTANCE, VIEW_DISTANCE + 1, VIEW_DISTANCE - 1]

# Sideways offsets (cells) accepted if the robot cannot stand dead centre
# in front of the image.
VIEW_OFFSETS = [0, -1, 1]

# --- the six moves the robot can make --------------------------------
#   FW forward 1 cell     BW backward 1 cell
#   FL / FR  90 degree turn, driving forwards
#   BL / BR  90 degree turn, reversing
# Order matters only for tie-breaking between equal-cost routes.
ALL_MOVES = ["FW", "BW", "FR", "FL", "BR", "BL"]

# Cost = roughly how many SECONDS a move takes, not how far it goes.
# That is what makes B.3 "shortest TIME" instead of "shortest distance".
MOVE_COST = {
    "FW": 1.0,
    "BW": 1.5,      # reversing is slower and less accurate
    "FL": 4.0,      # a 90 degree turn takes real time
    "FR": 4.0,
    "BL": 5.0,      # reverse turns are the slowest
    "BR": 5.0,
}

# A* heuristic: estimated cost per cell of Manhattan distance remaining.
HEURISTIC_WEIGHT = 0.5

# --- commands sent to the STM32 --------------------------------------
# Straight moves become e.g. "FW" + distance in cm -> "FW30".
STRAIGHT_COMMANDS = ["FW", "BW"]

# Each turn primitive becomes one fixed hardware command.
TURN_COMMANDS = {
    "FL": "LT90",
    "FR": "RT90",
    "BL": "XL90",
    "BR": "XR90",
}

# The photo marker. "SNAP" + obstacle id -> "SNAP6".
SNAP_PREFIX = "SNAP"

# --- ultrasonic alignment before each photo ----------------------------
# Just before every SNAP the planner inserts an align command, e.g. "AC25".
# The STM drives forward / backward until the ultrasonic sensor reads that
# many cm, then replies DONE, and the Pi takes the photo.
#
# The target follows the viewing distance the planner actually chose for
# that obstacle (normally VIEW_DISTANCE, but it can fall back to one of
# VIEW_DISTANCES), so it is always right for the pose the robot is at:
#     target_cm = cells_away * ALIGN_CM_PER_CELL + ALIGN_OFFSET_CM
# With VIEW_DISTANCE = 4 and ALIGN_OFFSET_CM = -15 this gives AC25.
# Set VIEW_DISTANCE = 3 for AC15.
ALIGN_ENABLED = True
ALIGN_PREFIX = "AC"         # agreed with the STM team
ALIGN_SEND_DISTANCE = True  # True -> "AC25". False -> just "AC" (STM uses its own fixed target)
ALIGN_CM_PER_CELL = 10
ALIGN_OFFSET_CM = -20      # 4 cells * 10 - 15 = AC25 (sensor 25 cm from the image)

# Also align at sideways-offset viewing poses (robot not dead centre in
# front of the image). The sensor might see the edge of the obstacle there.
ALIGN_ON_OFFSET = True

# --- drawing ---------------------------------------------------------
PX = 36                 # pixels per cell
MARGIN_LEFT = 44        # room for the y axis numbers
MARGIN_TOP = 16
PANEL_WIDTH = 400

WINDOW_WIDTH = MARGIN_LEFT + (GRID * PX) + PANEL_WIDTH
WINDOW_HEIGHT = MARGIN_TOP + (GRID * PX) + 44

COLOUR_BACKGROUND = (24, 26, 32)
COLOUR_PANEL = (18, 20, 25)
COLOUR_GRID_LINE = (60, 64, 76)
COLOUR_START_ZONE = (40, 78, 62)
COLOUR_HALO = (70, 52, 52)
COLOUR_OBSTACLE = (215, 90, 70)
COLOUR_IMAGE_FACE = (255, 220, 80)
COLOUR_ROBOT = (70, 150, 235)
COLOUR_ROBOT_EDGE = (250, 250, 250)
COLOUR_TRAIL = (120, 200, 160)
COLOUR_TEXT = (232, 234, 240)
COLOUR_DIM_TEXT = (150, 155, 168)
COLOUR_GOOD = (120, 230, 150)

