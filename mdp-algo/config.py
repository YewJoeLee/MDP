"""
config.py
---------
Every number the project uses lives here, and nowhere else.

Grid coordinates: (0,0) is the BOTTOM-LEFT cell. x grows RIGHT, y grows UP.
"N" (north) means +y. "E" (east) means +x.
"""

# --- the arena -------------------------------------------------------
GRID = 20               # 20 x 20 cells
CELL_CM = 10            # each cell is 10 cm, so the arena is 200 x 200 cm
START_ZONE_SIZE = 4     # start zone is cells (0,0) to (3,3) = 40 x 40 cm

# --- the robot -------------------------------------------------------
# The real car is 20 x 21 cm. We model it as a 3 x 3 block of cells
# (30 x 30 cm), which already builds in 5 cm of margin on every side.
ROBOT_SIZE = 3
ROBOT_REACH = 1         # body covers centre_x - 1 .. centre_x + 1
START_POSE = (1, 1, "N")

# How many cells the robot slides forward AND sideways during one 90 degree
# turn. Comes from the car's ~25-30 cm turning radius. Re-measure with the
# real car and change this one number.
TURN_CELLS = 3

# --- obstacles -------------------------------------------------------
# Extra empty cells demanded between the robot's body and any obstacle.
# 1 cell = 10 cm here, on top of the 5 cm the robot model already adds,
# so the real car keeps a 15 cm gap - including through turns.
SAFETY_CELLS = 1

# How far in front of an image the robot stands to photograph it.
# 4 cells = 40 cm from the obstacle centre, which puts the camera about
# 25 cm from the image - inside the 20-50 cm recognition band.
VIEW_DISTANCE = 4

# --- the six moves the robot can make --------------------------------
#   FW forward 1 cell     BW backward 1 cell
#   FL / FR  90 degree turn, driving forwards
#   BL / BR  90 degree turn, reversing
ALL_MOVES = ["FW", "BW", "FL", "FR", "BL", "BR"]

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
