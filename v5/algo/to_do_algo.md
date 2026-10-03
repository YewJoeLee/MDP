# Algorithm team: V5
## Deploy
On laptop, from v5/algo:

    python3 algo_server.py

Server remains on port 5000. Use this V5 server with the V5 client. Keep identical algo folders on laptop and Pi. Existing config.py is byte-for-byte unchanged, including current map/turn models, view distances, margins and flags. IMPORTANT: the new STM control behavior requires fresh movement measurements; these retained values are the OLD baseline until the STM team recalibrates.

## Implemented
- Existing primary pose/order selection is preserved; no lateral or angled pose expansion.
- Each segment includes backup=null or one checked straight BW pose.
- Try up to 5 cm of reverse, down to 1 cm; select the farthest valid candidate within the ORIGINAL 15-20 cm sensor-to-face viewing interval. For example, 15 cm can back up 5; 18 cm can back up 2; 20 cm has no backup. Primary route selection is not changed to force a backup.
- The entire reverse strip is checked against the full robot footprint, arena bounds and all physical obstacles using existing clearance rules. Existing POSITION_MARGIN_CM=0 is preserved; it is not an uncertainty guarantee.
- Following backup, plan the remaining route from its nominal endpoint. Retain the prior order if the current leg-selection model can execute it; otherwise use the existing most-targets/lowest-cost solver. Completed/failed-photo targets are excluded from new goals but remain physical obstacles.
- Camera fails do not trigger another visit in this version. Route-search limits and calibration assumptions remain as in v4.
- ALIGN_ENABLED is currently false. V5 image execution bypasses legacy AC/RA; enabling it is outside this tested recovery policy.

## Tests and tasks
    python3 test_v5_recovery.py

No serial/camera hardware is opened by these tests. Tests also verify the baseline hashes for config.py and original Android files. The simulator now executes run_plan_v5; old angle/AC retry controls and strategy labels are retained but inactive, and its camera/motion error model remains only an assumption.

Validate backup clearance and camera range physically, especially 1-2 cm moves and narrow margins. Do not widen the calibrated camera range merely to create more backups. If the robot frequently chooses the farthest primary pose, stationary-only retries are expected. Any future expanded distance range needs camera validation first.

See V5_NOTES.md for scope and package-wide validation.

## Handoff after STM changes
Get mean/median turn endpoints AND spread, full swept-path measurements and durations for all four turn directions. Update TURN_MODELS and timing only from these new measurements; preserve a copy of the old calibration. Determine an uncertainty margin from tests instead of assuming the retained POSITION_MARGIN_CM=0 covers errors.
