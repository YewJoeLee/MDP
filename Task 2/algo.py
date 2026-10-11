#!/usr/bin/env python3
"""
task2_algo.py -- Unified Config & Restored Comments
=====================================================================
NTU MDP -- Task 2 "Fastest Car" controller.

WHAT STILL NEEDS THE FIRMWARE PATCH
---------------------------------------------------------------------
Same as before -- see main_c_patch.md. Summary: `IR` is a new STM32
command (side IR sensors aren't wired into any command today), and
`Drive_Distance()` gets a periodic front-ultrasonic check so a long
`FW` can't drive blind into something closer than expected -- nothing
on this side of the serial link can react mid-command, only the
firmware can.
"""

import sys
import time
import serial
import config as cfg

# =====================================================================
# 1. LOW-LEVEL SERIAL TRANSPORT
# =====================================================================

def open_serial():
    ser = serial.Serial(cfg.SERIAL_PORT, cfg.SERIAL_BAUD, timeout=1)
    time.sleep(2.0)   # let the STM32's USB-serial re-enumerate after opening
    return ser


def send_command(ser, command, timeout_s=20):
    """Send one line, block for the STM32's single reply line. Returns
    the raw reply string, or None on timeout."""
    ser.write((command + "\n").encode("utf-8"))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            return line
    return None


def fail_run(reason):
    """The only place a genuinely unrecoverable problem stops the run,
    so the terminal always shows WHY."""
    print(f"\n[TASK2] ABORT: {reason}")
    raise RuntimeError(reason)


# =====================================================================
# 2. SENSOR READS
# =====================================================================

def read_front_distance_mm(ser):
    """Reply shape from main.c: 'US <mm> mm'."""
    reply = send_command(ser, cfg.CMD_READ_US)
    if reply is None:
        fail_run("No reply to US -- check the UART link.")
    try:
        return int(reply.split()[1])
    except (IndexError, ValueError):
        fail_run(f"Could not parse ultrasonic reply: {reply!r}")


def read_side_ir_mm(ser):
    """Reply shape from the NEW 'IR' command (main_c_patch.md):
    'IR L:<mm> R:<mm>'. Returns (left_mm, right_mm)."""
    reply = send_command(ser, cfg.CMD_READ_IR)
    if reply is None:
        fail_run("No reply to IR -- firmware patch not applied yet?")
    try:
        left_token, right_token = reply.split()[1:3]
        return int(left_token.split(":")[1]), int(right_token.split(":")[1])
    except Exception:
        fail_run(f"Could not parse IR reply: {reply!r}")


# =====================================================================
# 3. PRIMITIVE MOTION
# =====================================================================

def drive_straight_cm(ser, distance_cm, forward=True):
    """One straight drive, split into <=MAX_STRAIGHT_CM chunks.
    Stops the run on the first ERR/timeout."""
    distance_cm = int(round(distance_cm))
    if distance_cm <= 0:
        return
    prefix = cfg.CMD_FORWARD if forward else cfg.CMD_BACKWARD
    remaining = distance_cm
    
    while remaining > 0:
        step = min(remaining, cfg.MAX_STRAIGHT_CM)
        command = f"{prefix}{step}"
        reply = send_command(ser, command)
        print(f"    {command:8s} -> {reply}")
        if reply is None or reply.upper().startswith("ERR"):
            fail_run(f"{command} failed ({reply!r}).")
        remaining -= step
        time.sleep(cfg.CMD_DELAY_S)


def turn(ser, degrees, side):
    """One turn, steering forward (LT/RT). side: 'LEFT' or 'RIGHT'."""
    if side not in ("LEFT", "RIGHT"):
        raise ValueError(f"side must be 'LEFT' or 'RIGHT', got {side!r}")
    prefix = cfg.CMD_TURN_LEFT if side == "LEFT" else cfg.CMD_TURN_RIGHT
    command = f"{prefix}{int(round(degrees))}"
    reply = send_command(ser, command)
    print(f"    {command:8s} -> {reply}")
    if reply is None or reply.upper().startswith("ERR"):
        fail_run(f"{command} failed ({reply!r}).")
    time.sleep(cfg.CMD_DELAY_S)


def opposite(side):
    return "LEFT" if side == "RIGHT" else "RIGHT"


# =====================================================================
# 4. SENSOR-GUIDED APPROACH (fixes bug #1: back off, don't abort)
# =====================================================================

def drive_until_safe_then_finish_approach(ser, coarse_distance_cm, target_standoff_cm):
    """
    Drive `coarse_distance_cm` forward open-loop, then hand the last bit
    to the STM's AC closed-loop ultrasonic approach so the final
    stopping distance is sensor-checked.

    If the ultrasonic ever reads closer than FRONT_MIN_MM after
    the coarse leg, back off and re-check, up to MAX_RETRIES times, 
    before giving up.
    """
    drive_straight_cm(ser, coarse_distance_cm)

    for attempt in range(cfg.MAX_RETRIES + 1):
        reading_mm = read_front_distance_mm(ser)
        print(f"    [US] reading={reading_mm} mm (floor {cfg.FRONT_MIN_MM} mm)")
        if reading_mm >= cfg.FRONT_MIN_MM:
            break
        if attempt == cfg.MAX_RETRIES:
            fail_run(f"Still only {reading_mm} mm clear after retries -- likely actually blocked.")
        print(f"    [US] too close -- backing off {cfg.GENERAL_MARGIN_CM} cm and re-checking.")
        drive_straight_cm(ser, cfg.GENERAL_MARGIN_CM, forward=False)

    # Final bit via the STM's own closed-loop ultrasonic approach.
    sensor_target_mm = int((target_standoff_cm - cfg.US_OFFSET_CM) * 10)
    command = f"{cfg.CMD_APPROACH}{round(sensor_target_mm / 10)}"
    reply = send_command(ser, command)
    print(f"    {command:8s} -> {reply}")
    if reply is None or reply.upper().startswith("ERR"):
        fail_run(f"{command} failed ({reply!r}).")


def approach_obstacle_for_photo(ser, total_distance_cm):
    """Drive from wherever the robot is now to PHOTO_STANDOFF_CM short of
    the obstacle's face. This is the ONLY approach call for this leg --
    nothing after it re-closes any gap."""
    coarse_cm = max(0, total_distance_cm - cfg.PHOTO_STANDOFF_CM - cfg.GENERAL_MARGIN_CM)
    drive_until_safe_then_finish_approach(ser, coarse_cm, cfg.PHOTO_STANDOFF_CM)


# =====================================================================
# 5. OBSTACLE BYPASS -- two separate functions (fixes bug #3)
# =====================================================================
# Both start from wherever the robot is right now (the photo standoff). 
# The "pass" leg's length covers: the remaining standoff gap, the
# obstacle's own length, and a small exit margin.

def _pass_distance_cm(obstacle_length_cm):
    return cfg.PHOTO_STANDOFF_CM + obstacle_length_cm + cfg.CLEARANCE_MARGIN_CM


def _check_ir_clear_after_stepout(ser, out_side):
    """
    After swinging out sideways, read the side IR facing the obstacle.
    If the planned offset isn't actually clearing the obstacle yet, 
    do NOT abort. Instead, drive forward by GENERAL_MARGIN_CM (which 
    pushes the robot further OUTWARD into the bypass lane laterally) 
    and check again until clear.
    """
    for attempt in range(cfg.MAX_RETRIES + 1):
        left_mm, right_mm = read_side_ir_mm(ser)
        facing_mm = left_mm if out_side == "RIGHT" else right_mm
        print(f"    [IR] L={left_mm} R={right_mm} mm (floor {cfg.SIDE_MIN_MM} mm)")
        
        if facing_mm >= cfg.SIDE_MIN_MM:
            return  # We have successfully cleared the obstacle!
            
        if attempt == cfg.MAX_RETRIES:
            fail_run(f"Side IR still obstructed after {cfg.MAX_RETRIES} step-out corrections.")
            
        print(f"    [IR] Not clear. Driving {cfg.GENERAL_MARGIN_CM} cm further out.")
        drive_straight_cm(ser, cfg.GENERAL_MARGIN_CM)


def bypass_obstacle_1(ser, side, obstacle_length_cm):
    """
    Full lane-change around obstacle 1. Nets to ZERO heading change:
    turn, step out, turn, pass, turn, step back, turn -- ends facing
    forward again, ready to drive straight toward obstacle 2.
    """
    out_side, back_side = side, opposite(side)
    pass_cm = _pass_distance_cm(obstacle_length_cm)

    turn(ser, 90, out_side)
    drive_straight_cm(ser, cfg.LANE_SHIFT_CM)
    _check_ir_clear_after_stepout(ser, out_side)

    turn(ser, 90, back_side)
    drive_straight_cm(ser, pass_cm)
    turn(ser, 90, back_side)
    drive_straight_cm(ser, cfg.LANE_SHIFT_CM)
    turn(ser, 90, out_side)       # <-- the turn obstacle 2's bypass omits


def bypass_obstacle_2(ser, side, obstacle_length_cm):
    """
    Same shape as bypass_obstacle_1, but WITHOUT the final turn. Nets to
    a 90-degree heading change instead of zero, so the robot is already 
    a quarter of the way through the 180-degree flip it needs to face 
    the carpark. Returns `back_side`.
    """
    out_side, back_side = side, opposite(side)
    pass_cm = _pass_distance_cm(obstacle_length_cm)

    turn(ser, 90, out_side)
    drive_straight_cm(ser, cfg.LANE_SHIFT_CM)
    _check_ir_clear_after_stepout(ser, out_side)

    turn(ser, 90, back_side)
    drive_straight_cm(ser, pass_cm)
    turn(ser, 90, back_side)
    drive_straight_cm(ser, cfg.LANE_SHIFT_CM)
    # NOTE: no final turn(ser, 90, out_side) here -- see docstring.
    
    return back_side


# =====================================================================
# 6. RETURN TO CARPARK
# =====================================================================

def continue_curve_toward_carpark(ser, continuing_direction):
    """
    After bypassing obstacle 2, move past it completely and then face 
    the carpark, so the next leg can be a straight drive back.
    """
    drive_straight_cm(ser, cfg.CLEARANCE_MARGIN_CM)
    turn(ser, 90, continuing_direction)


def drive_back_to_carpark_standoff(ser, continuing_direction):
    """
    Drive back along the offset lane until safely past Obstacle 1,
    step back into the center lane (X=0), and then complete the 
    approach to the carpark standoff.
    """
    # 1. Drive straight back along offset lane. Added extra clearance margin 
    # for Obs 1 so the tail doesn't clip it during the final turn back to center.
    clearance_drive_cm = (
        cfg.OBS2_LENGTH_CM 
        + cfg.CLEARANCE_MARGIN_CM 
        + cfg.OBS1_TO_OBS2_CM 
        + cfg.OBS1_LENGTH_CM 
        + cfg.CLEARANCE_MARGIN_CM 
    )

    print(f"    [RETURN] Driving {clearance_drive_cm:.0f} cm along offset lane to clear Obstacles.")
    drive_straight_cm(ser, clearance_drive_cm)

    # 2. Step back to the center line (X=0)
    print("    [RETURN] Shifting back to center lane.")
    turn(ser, 90, continuing_direction)
    drive_straight_cm(ser, cfg.LANE_SHIFT_CM) 
    turn(ser, 90, opposite(continuing_direction))

    # 3. Drive the rest of the way to the carpark standoff
    remaining_cm = (
        cfg.CARPARK_TO_OBS1_CM 
        - cfg.CLEARANCE_MARGIN_CM 
        - cfg.CARPARK_CHECK_STANDOFF_CM
    )
    print(f"    [RETURN] Approaching carpark standoff ({remaining_cm:.0f} cm coarse drive).")
    
    coarse_cm = max(0, remaining_cm - cfg.GENERAL_MARGIN_CM)
    drive_until_safe_then_finish_approach(ser, coarse_cm, cfg.CARPARK_CHECK_STANDOFF_CM)


# =====================================================================
# 7. CARPARK RE-ENTRY: CHECK ALIGNMENT, CORRECT, CREEP IN
# =====================================================================

def align_and_enter_carpark(ser):
    """
    Robot is short of the carpark mouth, roughly facing it, but almost 
    certainly not square after open-loop driving. Read both side IR 
    sensors; if they disagree by more than ALIGN_TOLERANCE_MM, the nose
    has drifted off-centre -- turn a small, capped amount. Then creep 
    the remaining distance in.
    """
    for attempt in range(1, cfg.MAX_RETRIES + 1):
        left_mm, right_mm = read_side_ir_mm(ser)
        difference_mm = left_mm - right_mm
        print(f"    [ALIGN] attempt {attempt}: L={left_mm} R={right_mm} diff={difference_mm:+d} mm")

        if abs(difference_mm) <= cfg.ALIGN_TOLERANCE_MM:
            print("    [ALIGN] within tolerance.")
            break

        # Left reads farther than right -> nose has drifted toward the
        # right wall -> turn left to re-centre (and vice versa).
        correction_side = "LEFT" if difference_mm > 0 else "RIGHT"
        correction_deg = min(cfg.ALIGN_MAX_DEG, abs(difference_mm) * cfg.ALIGN_MULTIPLIER)
        print(f"    [ALIGN] correcting {correction_deg:.1f} deg {correction_side}.")
        turn(ser, correction_deg, correction_side)
    else:
        print(f"    [ALIGN] proceeding outside tolerance after {cfg.MAX_RETRIES} attempts.")

    coarse_cm = max(0, cfg.CARPARK_CHECK_STANDOFF_CM - cfg.CARPARK_FINAL_STOP_CM - cfg.GENERAL_MARGIN_CM)
    drive_until_safe_then_finish_approach(ser, coarse_cm, cfg.CARPARK_FINAL_STOP_CM)
    print("    [ALIGN] run complete.")


# =====================================================================
# 8. ARROW READING (camera + YOLO)
# =====================================================================

_picam2 = None
_yolo_model = None

def start_camera():
    global _picam2, _yolo_model
    from picamera2 import Picamera2
    from ultralytics import YOLO

    _picam2 = Picamera2()
    _picam2.preview_configuration.main.size = (416, 416)
    _picam2.preview_configuration.main.format = "BGR888"
    _picam2.preview_configuration.align()
    _picam2.configure("preview")
    _picam2.start()
    _picam2.capture_array()
    _yolo_model = YOLO(cfg.CAMERA_MODEL_PATH)
    print("[CAMERA] Pi camera and YOLO model ready.")


def stop_camera():
    global _picam2
    if _picam2 is not None:
        try:
            _picam2.stop()
        except Exception:
            pass
        _picam2 = None


def read_arrow_direction(obstacle_label):
    if _picam2 is None or _yolo_model is None:
        raise RuntimeError("Camera not started -- call start_camera() first.")

    arrow_classes = {"Left Arrow": "LEFT", "Right Arrow": "RIGHT"}

    for attempt in range(1, cfg.ARROW_RETRIES + 1):
        frame = _picam2.capture_array()
        detections = _yolo_model.predict(frame, imgsz=416, verbose=False)

        best_direction, best_confidence = None, 0.0
        for detection_result in detections:
            for box in detection_result.boxes:
                class_name = detection_result.names[int(box.cls[0])]
                confidence = float(box.conf[0])
                direction = arrow_classes.get(class_name)
                if direction is not None and confidence > best_confidence:
                    best_direction, best_confidence = direction, confidence

        print(f"[ARROW] {obstacle_label} attempt {attempt}: best={best_direction} conf={best_confidence:.2f}")
        if best_direction is not None and best_confidence >= 0.50:
            return best_direction
        time.sleep(cfg.CMD_DELAY_S)

    fail_run(f"Could not read a confident arrow for {obstacle_label} after {cfg.ARROW_RETRIES} attempts.")


# =====================================================================
# 9. FULL RUN
# =====================================================================

def run_task2(dry_run=False):
    start_camera()
    ser = None if dry_run else open_serial()

    try:
        print("\n[TASK2] Leg 1 -- approach obstacle 1's photo standoff.")
        approach_obstacle_for_photo(ser, cfg.CARPARK_TO_OBS1_CM)
        obstacle1_side = read_arrow_direction("Obstacle 1")
        print(f"[TASK2] Obstacle 1 arrow: {obstacle1_side}")

        print("\n[TASK2] Leg 2 -- bypass obstacle 1 (ends facing forward).")
        bypass_obstacle_1(ser, obstacle1_side, cfg.OBS1_LENGTH_CM)

        print("\n[TASK2] Leg 3 -- approach obstacle 2's photo standoff.")
        remaining_to_obstacle2_cm = cfg.OBS1_TO_OBS2_CM - cfg.CLEARANCE_MARGIN_CM
        approach_obstacle_for_photo(ser, remaining_to_obstacle2_cm)
        obstacle2_side = read_arrow_direction("Obstacle 2")
        print(f"[TASK2] Obstacle 2 arrow: {obstacle2_side}")

        print("\n[TASK2] Leg 4 -- bypass obstacle 2 (continues curving, no final turn).")
        continuing_direction = bypass_obstacle_2(ser, obstacle2_side, cfg.OBS2_LENGTH_CM)

        print("\n[TASK2] Leg 5 -- complete the curve to face the carpark.")
        continue_curve_toward_carpark(ser, continuing_direction)

        print("\n[TASK2] Leg 6 -- drive back to the carpark standoff.")
        drive_back_to_carpark_standoff(ser, continuing_direction)

        print("\n[TASK2] Leg 7 -- align to the carpark mouth and creep in.")
        align_and_enter_carpark(ser)
        
        print("\n[TASK2] Run complete.")

    finally:
        stop_camera()
        if ser is not None:
            ser.close()


def main():
    dry_run = "--dry-run" in sys.argv
    if dry_run:
        print("Dry run: no serial port opened; sensor-dependent calls will "
              "raise unless you stub send_command yourself.")
    run_task2(dry_run=dry_run)


if __name__ == "__main__":
    main()

