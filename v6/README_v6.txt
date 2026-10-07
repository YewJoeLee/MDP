v6 - photo rules + 30 cm viewing preference
============================================

Copy this whole folder to the laptop (algo_server.py side) AND to the Pi.
Both sides read the same config.py.

Laptop :  python algo_server.py
Pi     :  python3 rpi_client_android.py <laptop ip>
Camera :  python3 test_camera.py --loop      (prints every box, the count and the v5 case)
Replay :  python3 rpi_stm_conn.py -f cmds/latest.txt   (uses cmds/latest.txt.targets.json)

"count" = boxes whose class has a numeric image ID ("Bounding box" is NOT counted).

  Snap 1 IDEAL at the planned photo pose
  CASE A  count 1, conf >= RECOGNISED_CONF (0.50)  -> report it                      (1 photo)
  CASE B  count 1, conf <  0.50                    -> BW to FAR, snap, FW to NEAR,
                                                      snap, back to the photo pose.
                                                      Rank EVERY box from all photos by
                                                      confidence; report the highest.  (max 3 photos)
  CASE C  count 0                                  -> report nothing (MSG to Android)  (1 photo)
  CASE D  count 2+                                 -> IDEAL boxes held, FW to NEAR, snap,
                                                      back. Rank every box from IDEAL +
                                                      NEAR by confidence; report the top.  (max 2 photos)

Photos: photos/Trial_<run>_Obstacle_<id>_Snap<n>_Case<X>_<cm>cm[_short]_<IDEAL|FAR|NEAR>.jpg
        (position tag always last; _short = clearance stopped the FAR/NEAR move early)
Log:    photos/Trial_<run>_diagnosis.txt  - every photo, every box, why it was taken,
        how the planner chose each FAR/NEAR distance (what was blocked and by what),
        and the CASE B ranking.

Viewing distance preference (planner, config.py):
  VIEW_PREFERRED_SENSOR_CM = 30   costs 0, so 30 cm is tried first
  VIEW_FURTHER_COST_PER_CM = 1.0  per cm further away (robot further back)
  VIEW_NEARER_COST_PER_CM  = 2.0  per cm closer (robot further forward)
  Added on top of the lateral penalty (LATERAL_COST_PER_CM), which is unchanged.
  Set VIEW_PREFERRED_SENSOR_CM = None to make every distance cost the same again.
  The Pi prints the chosen distance, which side of 30 it is and the penalty paid.

FAR / NEAR distances come from the planner (planner.snap_retry_bounds), cut short
so MIN_CLEARANCE_CM and the arena wall are kept. Settings in config.py:
  SNAP_FAR_MAX_SENSOR_CM   furthest extra photo, sensor -> face (default: max view distance)
  SNAP_NEAR_MIN_SENSOR_CM  nearest extra photo, sensor -> face (default 12)
  SNAP_RETRY_MIN_MOVE_CM   a bound with less room than this is skipped (default 2)
  MODEL_MIN_CONF           YOLO drops boxes below this (0.25)
  RECOGNISED_CONF          CASE A threshold (0.50)

Function names are unchanged (trigger_camera, check_obstacle_side, photo_cycle,
run_commands, correct_snap_angle, ...). New: planner.snap_retry_bounds,
planner.sensor_to_face_cm, rpi_client_android.read_detections / report_result,
rpi_stm_conn.vote_across_snaps / print_snap / print_photo_header / print_photo_summary.
No angle correction runs in v5 (correct_snap_angle and check_obstacle_side are kept
but not called). simulator.py still runs; it does not simulate the FAR / NEAR photos.
