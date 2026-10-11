#!/usr/bin/env python3
"""
task2_algo.py -- Dynamic Odometry Edition
=====================================================================
NTU MDP -- Task 2 "Fastest Car" controller.
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
    time.sleep(2.0)
    return ser


def send_command(ser, command, timeout_s=20):
    ser.write((command + "\n").encode("utf-8"))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            return line
    return None


def fail_run(reason):
    print(f"\n[TASK2] ABORT: {reason}")
    raise RuntimeError(reason)


# =====================================================================
# 2. SENSOR READS
# =====================================================================

def read_front_distance_mm(ser):
    reply = send_command(ser, cfg.CMD_READ_US)
    if reply is None:
        fail_run("No reply to US -- check the UART link.")
    try:
        return int(reply.split()[1])
    except (IndexError, ValueError):
        fail_run(f"Could not parse ultrasonic reply: {reply!r}")


def read_side_ir_mm(ser):
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
# 4. SENSOR-GUIDED APPROACH (DYNAMIC ODOMETRY)
# =====================================================================

def drive_until_safe_then_finish_approach(ser, coarse_distance_cm, target_standoff_cm):
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

    sensor_target_mm = int((target_standoff_cm - cfg.US_OFFSET_CM) * 10)
    command = f"{cfg.CMD_APPROACH}{round(sensor_target_mm / 10)}"
    reply = send_command(ser, command)
    print(f"    {command:8s} -> {reply}")
    if reply is None or reply.upper().startswith("ERR"):
        fail_run(f"{command} failed ({reply!r}).")


def approach_obstacle_for_photo(ser, photo_stop_cm):
    """
    Reads the US sensor to dynamically determine the distance to the obstacle.
    Returns the exact Y-axis distance driven so the main loop can track it.
    """
    current_distance_mm = read_front_distance_mm(ser)
    current_distance_cm = current_distance_mm / 10.0
    
    distance_to_drive_cm = current_distance_cm - photo_stop_cm
    
    if distance_to_drive_cm > 0:
        coarse_cm = max(0, distance_to_drive_cm - cfg.GENERAL_MARGIN_CM)
        drive_until_safe_then_finish_approach(ser, coarse_cm, photo_stop_cm)
        return distance_to_drive_cm
    else:
        print("    [US] Already at or closer than photo standoff!")
        return 0


# =====================================================================
# 5. OBSTACLE BYPASS (DYNAMIC X-AXIS CLEARANCE)
# =====================================================================

def _check_ir_clear_after_stepout(ser, out_side):
    """
    Returns the extra X-axis distance driven to clear the obstacle.
    """
    extra_shift_cm = 0
    for attempt in range(cfg.MAX_RETRIES + 1):
        left_mm, right_mm = read_side_ir_mm(ser)
        facing_mm = left_mm if out_side == "RIGHT" else right_mm
        print(f"    [IR] L={left_mm} R={right_mm} mm (floor {cfg.SIDE_MIN_MM} mm)")
        
        if facing_mm >= cfg.SIDE_MIN_MM:
            return extra_shift_cm
            
        if attempt == cfg.MAX_RETRIES:
            fail_run(f"Side IR still obstructed after {cfg.MAX_RETRIES} step-out corrections.")
            
        print(f"    [IR] Not clear. Driving {cfg.GENERAL_MARGIN_CM} cm further out.")
        drive_straight_cm(ser, cfg.GENERAL_MARGIN_CM)
        extra_shift_cm += cfg.GENERAL_MARGIN_CM
        
    return extra_shift_cm


def bypass_obstacle_1(ser, side):
    out_side, back_side = side, opposite(side)
    pass_cm = cfg.OBST1_PHOTO_STOP_CM + cfg.OBST1_WIDTH_CM + cfg.OBST1_DRIVE_PAST_CM

    turn(ser, 90, out_side)
    drive_straight_cm(ser, cfg.OBST1_MIN_SIDE_SHIFT_CM)
    extra_shift = _check_ir_clear_after_stepout(ser, out_side)
    total_shift = cfg.OBST1_MIN_SIDE_SHIFT_CM + extra_shift

    turn(ser, 90, back_side)
    drive_straight_cm(ser, pass_cm)
    turn(ser, 90, back_side)
    drive_straight_cm(ser, total_shift) # Return to exact center using dynamic shift tracker
    turn(ser, 90, out_side)
    
    return pass_cm


def bypass_obstacle_2(ser, side):
    out_side, back_side = side, opposite(side)
    pass_cm = cfg.OBST2_PHOTO_STOP_CM + cfg.OBST2_WIDTH_CM + cfg.OBST2_DRIVE_PAST_CM

    turn(ser, 90, out_side)
    drive_straight_cm(ser, cfg.OBST2_MIN_SIDE_SHIFT_CM)
    extra_shift = _check_ir_clear_after_stepout(ser, out_side)
    total_shift = cfg.OBST2_MIN_SIDE_SHIFT_CM + extra_shift

    turn(ser, 90, back_side)
    drive_straight_cm(ser, pass_cm)
    turn(ser, 90, back_side)
    drive_straight_cm(ser, total_shift) 
    
    return pass_cm, back_side, total_shift


# =====================================================================
# 6. RETURN TO CARPARK (USING ODOMETRY)
# =====================================================================

def continue_curve_toward_carpark(ser, continuing_direction, crossover_cm):
    """
    Uses the exact tracked crossover_cm to mirror the obstacle bypass 
    on the opposite side of the track.
    """
    drive_straight_cm(ser, crossover_cm)
    turn(ser, 90, continuing_direction)


def drive_back_to_carpark_standoff(ser, continuing_direction, total_y_driven, obs1_front_y, crossover_cm):
    """
    Calculates the exact backward drive needed based on tracked Y odometry.
    """
    # 1. We must drive backward until our rear bumper clears Obstacle 1.
    safe_cut_in_y = obs1_front_y - cfg.RETURN_DRIVE_PAST_OBST1_CM
    clearance_drive_cm = total_y_driven - safe_cut_in_y

    print(f"    [RETURN] Driving {clearance_drive_cm:.0f} cm along offset lane to clear Obstacles.")
    drive_straight_cm(ser, clearance_drive_cm)

    # 2. Step back to the center line using the dynamically tracked X-axis shift
    print("    [RETURN] Shifting back to center lane.")
    turn(ser, 90, continuing_direction)
    drive_straight_cm(ser, crossover_cm) 
    turn(ser, 90, opposite(continuing_direction))

    # 3. Complete the approach using the remaining tracked Y coordinates
    remaining_cm = safe_cut_in_y - cfg.CARPARK_ALIGNMENT_STOP_CM
    print(f"    [RETURN] Approaching carpark standoff ({remaining_cm:.0f} cm coarse drive).")
    
    if remaining_cm > 0:
        coarse_cm = max(0, remaining_cm - cfg.GENERAL_MARGIN_CM)
        drive_until_safe_then_finish_approach(ser, coarse_cm, cfg.CARPARK_ALIGNMENT_STOP_CM)


# =====================================================================
# 7. CARPARK RE-ENTRY: CHECK ALIGNMENT
# =====================================================================

def align_and_enter_carpark(ser):
    for attempt in range(1, cfg.MAX_RETRIES + 1):
        left_mm, right_mm = read_side_ir_mm(ser)
        difference_mm = left_mm - right_mm
        print(f"    [ALIGN] attempt {attempt}: L={left_mm} R={right_mm} diff={difference_mm:+d} mm")

        if abs(difference_mm) <= cfg.ALIGN_TOLERANCE_MM:
            print("    [ALIGN] within tolerance.")
            break

        correction_side = "LEFT" if difference_mm > 0 else "RIGHT"
        correction_deg = min(cfg.ALIGN_MAX_DEG, abs(difference_mm) * cfg.ALIGN_MULTIPLIER)
        print(f"    [ALIGN] correcting {correction_deg:.1f} deg {correction_side}.")
        turn(ser, correction_deg, correction_side)
    else:
        print(f"    [ALIGN] proceeding outside tolerance after {cfg.MAX_RETRIES} attempts.")

    coarse_cm = max(0, cfg.CARPARK_ALIGNMENT_STOP_CM - cfg.CARPARK_FINAL_PARK_CM - cfg.GENERAL_MARGIN_CM)
    drive_until_safe_then_finish_approach(ser, coarse_cm, cfg.CARPARK_FINAL_PARK_CM)
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
        raise RuntimeError("Camera not started.")

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
# 9. FULL RUN WITH ODOMETRY TRACKING
# =====================================================================

def run_task2(dry_run=False):
    start_camera()
    ser = None if dry_run else open_serial()
    
    # Global Y-axis Odometry Tracker
    total_y_driven = 0

    try:
        print("\n[TASK2] Leg 1 -- approach obstacle 1's photo standoff.")
        y_leg1 = approach_obstacle_for_photo(ser, cfg.OBST1_PHOTO_STOP_CM)
        total_y_driven += y_leg1
        obs1_front_y = total_y_driven + cfg.OBST1_PHOTO_STOP_CM # Save exact Y coordinate of Obs 1
        obstacle1_side = read_arrow_direction("Obstacle 1")
        print(f"[TASK2] Obstacle 1 arrow: {obstacle1_side}")

        print("\n[TASK2] Leg 2 -- bypass obstacle 1 (ends facing forward).")
        y_leg2 = bypass_obstacle_1(ser, obstacle1_side)
        total_y_driven += y_leg2

        print("\n[TASK2] Leg 3 -- approach obstacle 2's photo standoff.")
        y_leg3 = approach_obstacle_for_photo(ser, cfg.OBST2_PHOTO_STOP_CM)
        total_y_driven += y_leg3
        obstacle2_side = read_arrow_direction("Obstacle 2")
        print(f"[TASK2] Obstacle 2 arrow: {obstacle2_side}")

        print("\n[TASK2] Leg 4 -- bypass obstacle 2 (re-centers, facing sideways).")
        # Tracks the dynamic X-axis shift required to clear the unknown length of Obstacle 2
        y_leg4, continuing_direction, dynamic_x_shift = bypass_obstacle_2(ser, obstacle2_side)
        total_y_driven += y_leg4

        print("\n[TASK2] Leg 5 -- complete the curve to face the carpark.")
        continue_curve_toward_carpark(ser, continuing_direction, dynamic_x_shift)

        print("\n[TASK2] Leg 6 -- drive back to the carpark standoff.")
        drive_back_to_carpark_standoff(ser, continuing_direction, total_y_driven, obs1_front_y, dynamic_x_shift)

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

