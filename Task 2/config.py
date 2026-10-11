"""
task2_config.py -- Dynamic Odometry Edition
=====================================================================
NTU MDP Task 2. The robot now tracks its own coordinate geometry dynamically.
"""

# =====================================================================
# 1. APPROACHING & BYPASSING OBSTACLE 1
# =====================================================================
OBST1_PHOTO_STOP_CM = 30
OBST1_MIN_SIDE_SHIFT_CM = 40  # Safe minimum shift. IR sensor will push out further if needed.
OBST1_WIDTH_CM = 10           # Front-to-back depth (Fixed by rules)
OBST1_DRIVE_PAST_CM = 20      # Extra forward buffer so the tail clears the 10cm width

# =====================================================================
# 2. APPROACHING & BYPASSING OBSTACLE 2
# =====================================================================
OBST2_PHOTO_STOP_CM = 30
OBST2_MIN_SIDE_SHIFT_CM = 40  # Start small (40cm). IR sensor dynamically clears unknown 30cm+ lengths.
OBST2_WIDTH_CM = 10           # Front-to-back depth (Fixed by rules)
OBST2_DRIVE_PAST_CM = 20      # Extra forward buffer so the tail clears the 10cm width

# =====================================================================
# 3. TURNING AROUND & RETURNING HOME
# =====================================================================
# Return lateral shifts are now calculated dynamically based on what the IR sensor tracked!
RETURN_DRIVE_PAST_OBST1_CM = 20  # Extra buffer to ensure tail clears Obs 1 on the way back

# =====================================================================
# 4. ARRIVING AT CARPARK
# =====================================================================
CARPARK_ALIGNMENT_STOP_CM = 30   # Distance short of carpark to use IR sensors
CARPARK_FINAL_PARK_CM = 20       # Distance to creep inside the carpark

# =====================================================================
# 5. CORRECTIONS, SAFETY & HARDWARE
# =====================================================================
GENERAL_MARGIN_CM = 10           # Buffer used for step-out corrections and back-offs
MAX_RETRIES = 5                  # Auto-correction limits (alignment, IR clearing, US back-offs)
FRONT_MIN_MM = 70                # Front US reading threshold to trigger a back-off
SIDE_MIN_MM = 500                 # Side IR reading threshold to trigger a step-out correction
ALIGN_TOLERANCE_MM = 15          # L/R disagreement above this = yawed, needs correction
ALIGN_MAX_DEG = 15               # Cap on any single correction turn
ALIGN_MULTIPLIER = 0.15          # Turn degrees per mm of IR disagreement (TUNE THIS)
US_OFFSET_CM = 12.0              # Distance from robot centre to front US sensor
MAX_STRAIGHT_CM = 100            # Longest single FW/BW before we split into chunks
CMD_DELAY_S = 0.2                # Delay between serial commands
ARROW_RETRIES = 5                # Max attempts to read the camera

# =====================================================================
# 6. ENVIRONMENT & COMMANDS
# =====================================================================
CAMERA_MODEL_PATH = "best_ncnn_model"
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
CMD_TURN_LEFT = "LT"
CMD_TURN_RIGHT = "RT"
CMD_FORWARD = "FW"
CMD_BACKWARD = "BW"
CMD_READ_US = "US"
CMD_APPROACH = "AC"
CMD_READ_IR = "IR"

