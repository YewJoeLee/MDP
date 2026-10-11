# =====================================================================
# 1. COURSE LAYOUT (Set on the day)
# =====================================================================
CARPARK_TO_OBS1_CM = 100
OBS1_TO_OBS2_CM = 100
OBS1_LENGTH_CM = 10
OBS2_LENGTH_CM = 30

# =====================================================================
# 2. MOVEMENT SIZES & DISTANCES
# =====================================================================
PHOTO_STANDOFF_CM = 30           # Distance to stop in front of obstacle for photo
LANE_SHIFT_CM = 40               # Sideways distance to swing out into the bypass lane
CLEARANCE_MARGIN_CM = 20         # Safe buffer to drive PAST an obstacle before cutting back in
CARPARK_CHECK_STANDOFF_CM = 30   # Stop this far short of carpark to check IR alignment
CARPARK_FINAL_STOP_CM = 10       # Final resting distance inside carpark

# =====================================================================
# 3. CORRECTIONS & SAFETY (Unified & Reduced)
# =====================================================================
GENERAL_MARGIN_CM = 10           # Shared small buffer used for back-offs, approach gaps, and step-out corrections
MAX_RETRIES = 5                  # Max auto-correction attempts (used for alignment, clearing IR, backing off)
FRONT_MIN_MM = 70                # Front US reading threshold to trigger a back-off
SIDE_MIN_MM = 50                 # Side IR reading threshold to trigger a step-out correction
ALIGN_TOLERANCE_MM = 15          # L/R disagreement above this = yawed, needs correction
ALIGN_MAX_DEG = 15               # Cap on any single correction turn
ALIGN_MULTIPLIER = 0.15          # Turn degrees per mm of IR disagreement (TUNE THIS)

# =====================================================================
# 4. HARDWARE & CAMERA
# =====================================================================
US_OFFSET_CM = 12.0              # Distance from robot centre point to front US sensor
MAX_STRAIGHT_CM = 100            # Longest single FW/BW before we split into chunks
CMD_DELAY_S = 0.2                # Delay between serial commands
ARROW_RETRIES = 5                # Max attempts to read the camera

# =====================================================================
# 5. ENVIRONMENT & COMMANDS (Do not change)
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

