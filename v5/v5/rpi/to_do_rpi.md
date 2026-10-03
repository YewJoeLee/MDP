# RPi team: V5
## Deploy
Keep v5/rpi and v5/algo as sibling folders on the Pi. RPi imports the single calibrated config.py from ../algo; do not deploy rpi alone. Run from v5/rpi:

    python3 rpi_client_android.py <laptop-IP>

Use your existing pyserial, Picamera2, OpenCV and Ultralytics environment. Place/link your existing best_ncnn_model in the working directory; model weights were not supplied in v4 and are not included here. Bluetooth, serial paths, confidence threshold and camera settings remain unchanged.

## New execution
The client uses run_plan_v5 with server segment metadata. It settles 0.5 seconds, tries up to three frames 0.2 seconds apart, stops immediately on recognition, and otherwise executes the single checked BW backup if supplied. It then settles and tries up to three more frames. Exhaustion sends "no image found" once. Successful detection uses the existing TARGET message.
No angled moves, AC/RA, undo/nudge/replay or repeated backups run in this flow. Legacy helper functions and constants remain for reference. Plain manual FW/BW/turn commands still work through rpi_stm_conn.py; flat SNAP command files are rejected because they lack clearance and replanning metadata.

## Validate on robot
- First-frame and third-frame success: no backup.
- Three misses with valid backup: one BW move, then at most three more frames.
- Three misses without backup: no movement and one final miss message.
- Backup succeeds or fails recognition: next leg is requested from the backup pose, not the original goal.
- Replan network failure: stop; never resume old commands. Every original obstacle stays in the request map even after its photo is complete.
- STM STALL/TIMEOUT/IMUFAIL/HEADINGERR/ABORT: halt. Verify stale DONE responses do not complete a different command.
- STOP is checked between motions and captures, not as a physical emergency-stop command during a blocking STM move.
- Verify separate frames are fresh after settling and all attempts save distinct filenames (1-6). Existing highest-confidence detection selection and threshold are unchanged; check for neighbouring-target misassociation.

The new frame/settle/backup policy is in ../algo/recovery_config.py. These are new defaults, not measured calibration values. Pose tracking is nominal dead reckoning, not measured localization.

Do not run the full arena route until the STM team has calibrated the NEW controller and algo/config.py has been updated with those measurements. The file in this ZIP intentionally keeps your existing baseline.

## Terminal flow logging update
Run `python3 -u rpi_client_android.py <laptop-IP>` for immediate terminal output.
Each obstacle prints its planned pose and available backup. Every movement prints the command BEFORE sending and the full STM response AFTER completion. Captures are labelled PRIMARY or BACKUP, with per-pose frame count and saved Snap attempt number. Detection, confidence, image path and write errors remain visible.
Explicit labels cover SETTLE, RETRY, BACKUP, BACKUP SKIPPED, NO IMAGE FOUND, REPLAN, REPLAN OK, UNREACHABLE, NEW LEG, STOP and RUN COMPLETE. Replanning states the new nominal start, remaining targets and retained physical obstacle count. No movement/recovery behavior changed in this logging update.
