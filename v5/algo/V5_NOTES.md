# V5
Four team folders: android, rpi, stm, algo. Start with each to_do_<team>.md.

## Requested image recovery
At the planned pose: settle, take up to 3 frames, stop early on recognition.
If unsuccessful: move to one preplanned collision-checked reverse backup within the existing camera range, settle and take up to 3 more frames.
If no backup or still unsuccessful: report "no image found" and continue.
After backup: replan remaining motion from its nominal endpoint. Keep remaining order if feasible; otherwise use the existing solver. All physical obstacles remain collision obstacles.
No angled or lateral-pose expansion, AC/RA, undo/nudge/replay or repeated backup runs in the live V5 policy.

## STM scope expanded at your request
V5 now adds a yaw-rate feedback turn profile with acceleration and final slowdown, turn-lock retention while braking, bounded straight heading correction, and stronger faults/timeouts. Inspiration: Group 10 motion control; Armaan's robot-specific movement models.
All original numerical STM #defines remain. New settings are in stm/v5_motion_control.h and are explicit UNCALIBRATED starting gains. Existing configured values are the hardware baseline. See stm/v4_to_v5.patch.
IMPORTANT: motion behavior changed, so remeasure x/y/heading distributions and swept paths before trusting the old algo endpoints. config.py is deliberately byte-for-byte unchanged, not silently recalibrated.

## Other necessary corrections
- RPi uses algo/config.py TURN_MODELS instead of its stale duplicated measurements.
- DONE must match the command; STALL/TIMEOUT/IMUFAIL/HEADINGERR/ABORT stops execution.
- Long camera/replan pauses do not silently erase STM heading error.
- Simulator execution follows the new recovery engine. Its legacy angle/AC controls and strategy labels remain but are inactive; its error model is only a simulation assumption.

## Layout/deployment
Original algo_rpi_code files split by responsibility. Deploy rpi and algo as sibling folders on the Pi; run the server from algo on the laptop. Original .bak configuration and test map retained. Android project files byte-for-byte unchanged, with only to_do_android.md added. Brief PDFs remain in original v4; not duplicated into this code package. Existing camera model weights and the complete STM board project were not in v4 and are not included.

## Tests
14 Python tests passed, including early success, retry cap, no backup, stale-leg replacement, failed motion, reply parsing, backup clearance/range, original calibration/Android hashes and Python syntax.
A two-obstacle planner run and a replan from a backup endpoint succeeded; replanned edges checked against the full map. Headless simulator exercised six misses and one backup.
C control helpers compile with host GCC -Wall -Wextra -Werror and pass bounds/feedback tests. The complete STM firmware was not compiled (board project/headers unavailable), Android was not rebuilt, and camera/Bluetooth/robot behavior has not been physically tested.

Terminal logging update: explicit PRIMARY/BACKUP captures, move send/STM reply, skipped backup, final misses, next-leg replanning and interruption messages. Three extra logging tests verify the backup/replan, no-backup/miss and failed-movement flows.

## Compensation sequencing fix
Car_Turn_Square now stops immediately on any failed sub-move. Failed XL/XR forward compensation prevents the turn. A failed LT/RT turn prevents reverse compensation. Compensation TIMEOUT, STALL and IMUFAIL propagate into the final turn reply; unexpected failures report ABORT. Failed moves brake, and early-exit diagnostics no longer inherit the previous turn's values. All four UART turn handlers use the same status helper. No motion settings changed.

Run `python test_turn_compensation.py` in stm with host GCC installed. It extracts the actual wrapper/helpers and checks 40 success/failure/zero-compensation/non-90-degree cases using stubbed hardware. The configured compensation distances remain zero; tests explicitly enable nonzero compensation to cover the latent bug. All 14 Python recovery/logging tests also pass. Full board firmware build and hardware validation are still required.
