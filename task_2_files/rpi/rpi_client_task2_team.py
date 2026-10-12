#!/usr/bin/env python3
"""Task 2 on the RPi with the TEAM's algorithm (task2_team.py).

Same robot code as rpi_client_task2.py - it is imported, not copied:
    Stm (send / read_us / read_ir / read_heading / drive_edge=IE / bias / stop),
    Camera (Picamera2 + YOLO snap), Out (screen + photos/task2_run_<n>.log),
    SimRobot, make_snap, find_port, next_run ...
Only the route is different: task2_team.run_task2() instead of task2.run_task2().

Usage (same switches as rpi_client_task2.py, plus --stage):
    python3 rpi_client_task2_team.py                   # BI, press Enter, full run
    python3 rpi_client_task2_team.py --go              # no Enter prompt
    python3 rpi_client_task2_team.py --stage obs1      # team stages: obs1 | detect2 |
                                                       #   o2turn | obs2 | full
    python3 rpi_client_task2_team.py --no-bi / --no-gyro / --no-ie / --port /dev/ttyACM1
    python3 rpi_client_task2_team.py --sim             # laptop: arena from show_path.py
    python3 rpi_client_task2_team.py --sim --a1 L --a2 R --g1 60 --g2 100 --length 40

Ctrl+C at any time sends STOP to the STM.
"""
import argparse
import os
import re
import sys
import time
import traceback
from datetime import datetime

import rpi_client_task2 as RC          # chdir to this folder + your Stm / Camera / Out
import config as C
import task2_team as T

STEPS = [
    ("approach_obs1", "0", "Approach obstacle 1",
     f"Ultrasonic approach to {T.O1_CAPTURE_US_CM:g} cm; arrow = two photos in a row >= {T.ARROW_CONFIRM_CONF:g}."),
    ("bypass_obs1", "1", "O1 side-lane exit",
     f"45 out, {T.OBS1_OUT_DIAGONAL_CM} cm, 45 back, then forward until the side IR is past O1. Stay in the lane."),
    ("aim_obs2", "2", "O2 coarse centring",
     "Photos at 0/15/25 deg inward, aim at O2's centre, diagonal US approach to 35 cm, undo the heading."),
    ("loop_obs2", "3", "Loop obstacle 2",
     "90 out, slide until the inside IR passes O2's end, 90 in, pass O2, cross behind it, back down the other side."),
    ("return_home", "4", "Home past obstacle 1",
     "Shift to the home lane, inside IR passes O1, S-curve onto the carpark centre line."),
    ("park", "5", "Park",
     f"Both side IRs find the carpark walls, then pull in to {T.CARPARK_WALL_STOP_CM:g} cm off the back wall."),
]

f1 = RC.f1


def summary(fname, r):
    g = r.get
    if fname == "approach_obs1":
        return (f"US {f1(g('us'))} cm · arrow 1 = {g('arrow')} ({g('arrow_how')}, conf {f1(g('conf'))})"
                f" · drove {f1(g('D1'))} cm")
    if fname == "bypass_obs1":
        return (f"diagonal {g('diag')} cm · side forward {g('side_fw')} cm · O1 clearance {f1(g('clear'))} cm"
                f" · lane y {f1(g('lane_y'))} cm")
    if fname == "aim_obs2":
        return (f"arrow 2 = {g('arrow')} · centre bearing {g('bearing')} deg · aimed US {f1(g('us'))} cm"
                f" · O2 face x {f1(g('x2'))} · y {f1(g('y'))} cm")
    if fname == "loop_obs2":
        if g("stage") == "o2turn":
            return "stopped after the first 90-degree turn (team o2turn stage)"
        return (f"O2 length ~{f1(g('length'))} cm · end at y {f1(g('y_edge'))} · "
                f"return corridor y {f1(g('y_return'))} cm")
    if fname == "return_home":
        return (f"home lane {f1(g('lane'))} cm · IR beside O1 {f1(g('ir_obs1'))} cm · "
                f"at x {f1(g('x'))} y {f1(g('y'))}")
    if fname == "park":
        lr = g("ir_lr") or (None, None)
        return (f"back wall {f1(g('back_cm'))} cm · IR L/R {f1(lr[0])}/{f1(lr[1])} · "
                f"pulled in {g('pulled')} cm")
    return ""


def wrap_steps(state):
    for fname, num, title, purpose in STEPS:
        orig = getattr(T, fname)

        def wrapped(*a, _f=fname, _n=num, _t=title, _p=purpose, _o=orig, **k):
            state["step"] = _n
            RC.OUT.say(f"\n== STEP {_n} · {_t} " + "=" * max(4, 48 - len(_t) - len(_n)))
            RC.OUT.say(f"   {_p}")
            t0 = time.time()
            r = _o(*a, **k)
            RC.OUT.flush_cmds()
            for line in r.get("log", []):
                s = str(line)
                if re.match(r"^(FW|BW|LT|RT)\d+$", s):
                    continue                              # already printed as commands
                RC.OUT.say(f"   why: {s}")
            status = r.get("status", "OK")
            RC.OUT.say(f"   {'OK ' if status == 'OK' else '!! '}{summary(_f, r) if status == 'OK' else ''}")
            if status != "OK":
                RC.OUT.say(f"   !! status {status}")
            RC.OUT.say(f"   ({time.time() - t0:.1f} s)")
            state["times"][_n] = time.time() - t0
            return r
        setattr(T, fname, wrapped)


class TeamSimRobot(RC.SimRobot):
    """RC.SimRobot on sim_team.TeamWorld (LT/RT of any angle)."""

    def __init__(self, a):
        import sim_team
        self.w = sim_team.TeamWorld(a.g1, a.g2, a.length, a.a1, a.a2, wall=a.wall,
                                    ir_fwd=getattr(C, "T2_IR_FWD_CM", 0.0),
                                    s=getattr(C, "T2_IR_INSET_CM", 0.0))
        self.n = 0


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=RC.PORT)
    ap.add_argument("--go", action="store_true", help="start without waiting for Enter")
    ap.add_argument("--no-bi", action="store_true", help="skip the gyro bias calibration")
    ap.add_argument("--no-gyro", action="store_true", help="track heading from the turn models only")
    ap.add_argument("--no-ie", action="store_true", help="old firmware: FW3 + IR steps instead of IE")
    ap.add_argument("--stage", choices=T.STAGES, default="full",
                    help="stop after a team test stage (obs1, detect2, o2turn, obs2) or run it all")
    ap.add_argument("--sim", action="store_true", help="simulated arena, no robot")
    for k in ("g1", "g2", "length", "wall"):
        ap.add_argument(f"--{k}", type=float)
    ap.add_argument("--a1")
    ap.add_argument("--a2")
    a = ap.parse_args()
    arena = ("g1", "g2", "length", "a1", "a2", "wall")
    if not a.sim:
        given = [f"--{k}" for k in arena if getattr(a, k) is not None]
        if given:
            sys.exit(f"{' '.join(given)} only work with --sim: the real run must not be told the layout.")
        return a
    import show_path as SP
    defaults = {"g1": SP.G1_CM, "g2": SP.G2_CM, "length": SP.LENGTH_CM,
                "a1": SP.ARROW_1, "a2": SP.ARROW_2, "wall": SP.WALL_CM}
    for k in arena:
        if getattr(a, k) is None:
            setattr(a, k, defaults[k])
    a.a1 = "LEFT" if str(a.a1).upper().startswith("L") else "RIGHT"
    a.a2 = "LEFT" if str(a.a2).upper().startswith("L") else "RIGHT"
    return a


def main():
    a = parse_args()
    os.makedirs(RC.PHOTO_DIR, exist_ok=True)
    run = RC.next_run()
    log_path = os.path.join(RC.PHOTO_DIR, f"task2_run_{run}.log")
    RC.OUT = RC.Out(log_path)                     # Stm / make_snap print through RC.OUT
    RC.OUT.logf(f"--- Task 2 TEAM run {run}  {datetime.now():%Y-%m-%d %H:%M:%S}  "
                f"{'SIM' if a.sim else a.port}  stage {a.stage}")

    print(f"TASK 2 (team algorithm) · run {run} · {'SIMULATION' if a.sim else 'robot on ' + a.port}"
          f" · stage {a.stage}")
    print(f"   O1 photo stop {T.O1_CAPTURE_US_CM:g} cm · O1 diagonal {T.OBS1_OUT_DIAGONAL_CM} cm · "
          f"O2 sweep {list(T.O2_SWEEP_ANGLES_DEG)} deg · O2 stop {T.O2_CENTER_US_CM:g} cm · "
          f"clearance aim {T.NEED_PREF:g} cm (never under {T.NEED_MIN:g})")
    print(f"   IR legs: {'FW3 + IR steps (--no-ie)' if a.no_ie else 'one IE command each'}")
    print(f"   log: {log_path}")

    robot = cam = None
    try:
        if a.sim:
            robot = TeamSimRobot(a)
            print(f"   sim arena: G1 {a.g1:g} · G2 {a.g2:g} · L {a.length:g} · wall {a.wall:g} · "
                  f"arrows {a.a1}/{a.a2}")
        else:
            port = RC.find_port(a.port)
            try:
                robot = RC.Stm(port, RC.BAUD)
            except Exception as e:
                print(f"!! cannot open {port}: {e}  (STM plugged in? another script using it?)")
                return 1
            print(f"   STM on {port}")
            print("   camera + YOLO loading ...")
            try:
                cam = RC.Camera(run)
            except Exception as e:
                RC.OUT.logf(traceback.format_exc())
                print(f"!! camera / model failed: {e}")
                return 1
            print(f"   model: {cam.model_path}")
            robot.snap = cam.snap
            if not a.no_bi:
                print("   BI: gyro calibration - keep the robot still ...")
                ok, line = robot.bias()
                print(f"   {line}")
                if not ok:
                    print("   ! gyro bias still not clean - carrying on with the STM's start-up bias")
            us = robot.read_us()
            RC.OUT.flush_us()
            ir = robot.read_ir()
            print("   US {} cm · IR L/R {}".format(f1(us), " / ".join(f1(v) for v in ir)))

        if not a.go and sys.stdin is not None and sys.stdin.isatty():
            try:
                input("\nRobot in the carpark, facing obstacle 1. Enter = START (Ctrl+C = STOP) ")
            except EOFError:
                pass
        else:
            print("\n   starting now (--go or no keyboard)")

        state = {"step": "0", "times": {}}
        wrap_steps(state)
        heading = None if (a.sim or a.no_gyro) else robot.read_heading
        kw = {}
        if a.sim:
            kw["cp_front_from_start"] = C.T2_CARPARK_DEPTH_CM - 18     # sim start pose
        t0 = time.time()
        out = T.run_task2(robot.send, robot.read_us, robot.read_ir_left, robot.read_ir_right,
                          RC.make_snap(robot, state), read_heading=heading,
                          drive_edge=None if a.no_ie else robot.drive_edge, stage=a.stage, **kw)
        total = time.time() - t0

        RC.OUT.say("\n" + "=" * 60)
        arrows = out.get("arrows") or (out.get("start", {}).get("arrow"), None)
        RC.OUT.say(f"RESULT  {out['status']}  ·  stage {a.stage}  ·  {total:.1f} s  ·  "
                   f"arrows {arrows[0]} / {arrows[1]}")
        RC.OUT.say("   step times: " + "  ".join(f"{k}:{v:.1f}s" for k, v in state["times"].items()))
        if a.sim:
            p, mc = robot.w.p, robot.w.minclear
            RC.OUT.say(f"   sim: final pose ({p.x:.1f}, {p.y:.1f}, {p.h:.0f} deg) · closest {mc[0]:.1f} cm "
                       f"to {mc[1]} during {mc[2]}")
        return 0 if out["status"] == "OK" else 2

    except RC.MoveFailed as e:
        RC.OUT.say(f"\n!! STOPPED: {e}")
        robot.stop()
        return 3
    except KeyboardInterrupt:
        RC.OUT.say("\n!! Ctrl+C - STOP sent")
        if robot:
            robot.stop()
        return 130
    except Exception as e:
        RC.OUT.say(f"\n!! CRASHED: {type(e).__name__}: {e}  - STOP sent (traceback below and in the log)")
        RC.OUT.logf(traceback.format_exc())
        traceback.print_exc()
        if robot:
            robot.stop()
        return 4
    finally:
        if cam:
            cam.close()
        if robot:
            robot.close()
        RC.OUT.close()


if __name__ == "__main__":
    sys.exit(main())
