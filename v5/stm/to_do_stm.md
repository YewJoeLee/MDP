# STM team: V5 repeatability work
## What to integrate
- latest_stm.c: updated firmware source based on your v4, preserving all original numeric #defines, pin mappings and timer setup.
- v5_motion_control.h: NEW motion-controller settings and pure control helpers. Add it to the STM project's include path.
- v4_to_v5.patch: exact firmware-source changes for review.
- test_motion_control.c: host-only helper tests. Do NOT add this test's main() to the STM firmware build.

The original full CubeIDE project/headers were not supplied. Integrate the C source and header in your existing project, build there, and flash. Check that CubeMX regeneration preserves the added include/control changes.

## Implemented changes
Inspired by Group 10's feedback/speed profiling and heading hold, while retaining your hardware-specific settings and Armaan-style separate motion models:
1. Turns use a ramped yaw-rate target, slowing near the target heading. PI yaw-rate feedback scales both wheel PWMs together, retaining each turn's configured inner/outer PWM ratio and maximum PWM percentage. This is yaw-rate control, NOT separate encoder speed PI for each wheel.
2. Feed-forward references 135/57/62/52 deg/s come from v4 comments. They are approximate historical values, not new measurements.
3. Steering stays at that turn's lock during braking; it no longer automatically swings left during right-turn braking. Recentering follows the existing stop wait.
4. Straights apply bounded, slew-limited servo correction around your existing SERVO_CENTER, using accumulated heading error. Reverse correction changes sign. Existing encoder distance/synchronization logic remains.
5. Straight setup approaches centre from 40 us below instead of sweeping all the way left. Verify backlash behavior mechanically.
6. Turn stopping uses gyro scale consistently in the enabled controller. Existing COAST_K values remain starting points but require new measurements with the slower approach and changed steering behavior.
7. No automatic heading reset on a 15-second camera/replan pause. Use BI after deliberate repositioning.
8. Wall-clock timeouts, early gyro-fault termination, explicit TIMEOUT/IMUFAIL/HEADINGERR status. The Pi stops on these outcomes; it does not guess a new pose after an incomplete move.

## New values: starting points, not calibrated
All new values are together in v5_motion_control.h:
- Max/min turn rate: 45/18 deg/s; acceleration/deceleration: 90 deg/s^2.
- Yaw-rate PI gains: 12 PWM/(deg/s), 20 PWM/deg; anti-windup and configured PWM limits.
- Final heading tolerance: 3 degrees. Outside tolerance returns HEADINGERR; no automatic corrective wiggle.
- Straight heading gain: 3 us/degree; limit +/-80 us; slew 400 us/s.
- V5_TURN_CONTROL_ENABLED and V5_HEADING_HOLD_ENABLED allow one subsystem at a time during bench comparison. Existing STEER_TEST values/commands remain.
- Turn watchdog is the larger of original TURN_TIMEOUT_MS and angle/minimum-rate + 2 seconds. The Pi retains its 20-second reply timeout, sufficient for the planner's 90-degree commands. Very large manual angles may exceed it.

## Calibration order -- do this before using old planner endpoints
1. Build and inspect PWM/servo direction on the bench. Confirm a positive left-heading error steers right going forward and reverses that correction going backward. Verify commanded pulses do not strain the servo.
2. Keep robot still, run BI; verify clean gyro calibration, no ERR/SAT counts. Check STOP/fault outcomes with wheels raised first.
3. Test FW/BW at several distances. Tune heading gain/slew only if needed; measure physical heading AND lateral drift. Gyro/encoders alone cannot establish true x/y.
4. Repeat each LT90, RT90, XL90, XR90 at least 10 times on the task floor. Record battery condition, actual x/y displacement, actual final angle, IMU angle, braking rate and failure status. Repeat in motion sequences, not only isolated turns.
5. Tune the NEW rate gains/profile first. Then refit the existing COAST_K per turn. Redo this if steering locks, speed, braking or inner/outer ratio changes.
6. Validate short BW1..BW5 backups. If one-centimetre moves are unreliable, coordinate with algo to restrict backup choices to tested distances.
7. Give algo NEW turn endpoints, representative swept paths, repeatability spread and durations. The unchanged config.py retains old calibrated values for comparison; it is NOT validated against this new controller.
8. Only after these measurements run multi-obstacle trials. Separate a repeatable offset (calibration issue) from broad random spread (control/mechanical issue).

## Limits and verification
Host helper tests passed (bounds, saturation recovery, heading direction/slew and simple mocked rate response). These do not model the real tyres, backlash, floor or battery. Full firmware has not been built/flashed here. No claim of achieved physical accuracy is made.

Legacy functions/settings remain. The new controls can be disabled for A/B testing, but reliability/status and timeout changes remain. Keep a copy of v4 for a full baseline comparison.

With V5_TURN_CONTROL_ENABLED=1, stopping always uses the configured speed-adaptive COAST_K model. The legacy KA fixed-offset switch affects only the disabled-profile baseline, because old high-speed fixed offsets do not suit the new slow approach.

## Compensation sequencing fix
Car_Turn_Square now stops immediately on any failed sub-move. Failed XL/XR forward compensation prevents the turn. A failed LT/RT turn prevents reverse compensation. Compensation TIMEOUT, STALL and IMUFAIL propagate into the final turn reply; unexpected failures report ABORT. Failed moves brake, and early-exit diagnostics no longer inherit the previous turn's values. All four UART turn handlers use the same status helper. No motion settings changed.

Run `python test_turn_compensation.py` in stm with host GCC installed. It extracts the actual wrapper/helpers and checks 40 success/failure/zero-compensation/non-90-degree cases using stubbed hardware. The configured compensation distances remain zero; tests explicitly enable nonzero compensation to cover the latent bug. All 14 Python recovery/logging tests also pass. Full board firmware build and hardware validation are still required.
