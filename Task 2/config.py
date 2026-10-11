"""
task2_config.py -- Phase-Specific Tuning Edition
=====================================================================
NTU MDP Task 2. Constants are broken down by phase and geometry.
"""

# =====================================================================
# 1. APPROACHING & BYPASSING OBSTACLE 1
# =====================================================================
CARPARK_TO_OBST1_CM = 100
OBST1_PHOTO_STOP_CM = 30
OBST1_SIDE_SHIFT_CM = 40      # Must clear half Obs 1 length (10) + half robot width
OBST1_WIDTH_CM = 10           # Front-to-back depth
OBST1_DRIVE_PAST_CM = 20      # Extra forward buffer so the tail clears the 10cm width

# =====================================================================
# 2. APPROACHING & BYPASSING OBSTACLE 2
# =====================================================================
OBST1_TO_OBST2_CM = 100       # Empty space between the two obstacles
OBST2_PHOTO_STOP_CM = 30
OBST2_SIDE_SHIFT_CM = 60      # Must clear half Obs 2 length (30+) + half robot width
OBST2_WIDTH_CM = 10           # Front-to-back depth
OBST2_DRIVE_PAST_CM = 20      # Extra forward buffer so the tail clears the 10cm width

# =====================================================================
# 3. TURNING AROUND & RETURNING HOME
# =====================================================================
FACE_CARPARK_DRIVE_CM = OBST2_SIDE_SHIFT_CM         # Drive in side lane before final 90-degree turn
RETURN_DRIVE_PAST_OBST1_CM = 20    # Extra buffer to ensure tail clears Obs 1 on the way back
RETURN_SHIFT_TO_CENTER_CM = FACE_CARPARK_DRIVE_CM     # Sideways drive back to center (usually matches FACE_CARPARK_DRIVE_CM)

# =====================================================================
# 4. ARRIVING AT CARPARK
# =====================================================================
CARPARK_ALIGNMENT_STOP_CM = 30     # Distance short of carpark to use IR sensors
CARPARK_FINAL_PARK_CM = 10         # Distance to creep inside the carpark

# =====================================================================
# 5. CORRECTIONS, SAFETY & HARDWARE
# =====================================================================
GENERAL_MARGIN_CM = 10           # Buffer used for step-out corrections and back-offs
MAX_RETRIES = 5                  # Auto-correction limits (alignment, IR clearing, US back-offs)
FRONT_MIN_MM = 70                # Front US reading threshold to trigger a back-off
SIDE_MIN_MM = 50                 # Side IR reading threshold to trigger a step-out correction
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

