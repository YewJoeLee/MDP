"""Trig plan calculator - same maths as T2_LaneShift() in main.c.

Give it arrow 2's direction and obstacle 2's measured length, and it prints the path
from the exit 90 (LT90 for arrow R, RT90 for arrow L) up to obstacle 1's edge.

Usage:
    python3 trig_plan.py R 44.9              # arrow 2 RIGHT, TxV 44.9 cm (+ speed add)
    python3 trig_plan.py R 44.9 --beside     # OLL said START:BESIDE -> also + offset
    python3 trig_plan.py L 44.9              # arrow 2 LEFT  (exit RT90, right IR)
    python3 trig_plan.py R 62.9 --final      # length already includes the +10 +8
    python3 trig_plan.py R 44.9 --drift 2.5 --ahead 80
        --drift  cm the robot went past the edge after the IR saw it (default 1.7, latest OLL)
        --ahead  cm from the robot (after the exit 90) to obstacle 1's near edge, if known:
                 also prints how far the HOME leg drives straight
KEEP THE CONSTANTS BELOW THE SAME AS main.c
"""
import argparse
import math

# ---- copied from main.c ----
LEN_SPEED_ADD_CM = 0       # T2_LEN_SPEED_ADD_CM  (PE)
LEN_OFFSET_CM = 8          # T2_LEN_OFFSET_CM     (PO)
FW_AFTER_OLL2_CM = 8       # T2_FW_AFTER_OLL2_CM
AFTER_LOOP_FW_CM = 0       # T2_AFTER_LOOP_FW_CM
EXIT_RT90_FWD_CM = 23.0    # T2_EXIT_RT90_FWD_CM  (arrow 2 LEFT exit)
EXIT_LT90_FWD_CM = 19.0    # T2_EXIT_LT90_FWD_CM  (arrow 2 RIGHT exit)
IR_FWD_CM = 4.5            # T2_IR_FWD_CM
X_TRIM_CM = 0.0            # T2_X_TRIM_CM
OBS1_HALF_W_CM = 5.0       # T2_OBS1_HALF_W_CM
IR_SIDE_CM = 8.0           # T2_IR_SIDE_CM
AIM_CM = 15.0              # T2_OBS1_IR_GAP_CM
GAP_MIN_CM, GAP_MAX_CM = 10.0, 20.0
GAP_MARGIN_CM = 2.0        # T2_GAP_MARGIN_CM
BUDGET_CM = 50.0           # T2_SHIFT_BUDGET_CM
SHIFT_MAX_FW_CM = 150      # T2_SHIFT_MAX_FW_CM
HOME_NEAR_CM = 20          # T2_HOME_NEAR_MM / 10
TURN = {"LT": (15.0, 17.0),   # T2_LT45_SIDE_CM, T2_LT45_FWD_CM
        "RT": (17.0, 21.0)}   # T2_RT45_SIDE_CM, T2_RT45_FWD_CM
# ----------------------------
S45 = math.sin(math.radians(45))


def plan(X, toward, pre_fw):
    """toward = 'LT' or 'RT', the turn that heads to the centre line. Mirrors T2_LaneShift."""
    other = "RT" if toward == "LT" else "LT"
    gap0 = X - OBS1_HALF_W_CM - IR_SIDE_CM
    moving_in = gap0 > AIM_CM
    tin = toward if moving_in else other
    tback = other if tin == toward else toward
    sg = -1.0 if moving_in else 1.0
    in_side, in_fwd = TURN[tin]
    back_side, back_fwd = TURN[tback]
    t_lat = in_side + (back_fwd - back_side) * S45
    t_fwd = in_fwd + (back_fwd + back_side) * S45
    edge = GAP_MAX_CM - GAP_MARGIN_CM if moving_in else GAP_MIN_CM + GAP_MARGIN_CM

    cands = [("nothing", -1, gap0, pre_fw), ("turns only", 0, gap0 + sg * t_lat, pre_fw + t_fwd)]
    for name, aim in (("aim %.0f" % AIM_CM, AIM_CM), ("aim %.0f (edge)" % edge, edge)):
        dd = (abs(gap0 - aim) - t_lat) / S45
        d = min(int(dd + 0.5) if dd > 0 else 0, SHIFT_MAX_FW_CM)
        cands.append((name, d, gap0 + sg * (t_lat + d * S45), pre_fw + t_fwd + d * S45))

    inwin = lambda g: GAP_MIN_CM - 0.05 <= g <= GAP_MAX_CM + 0.05
    best, fit = None, "OK"
    if GAP_MIN_CM <= gap0 <= GAP_MAX_CM:
        best = 0
    if best is None:
        for i in range(1, 4):
            if inwin(cands[i][2]) and cands[i][3] <= BUDGET_CM + 0.05 and (
                    best is None or abs(cands[i][2] - AIM_CM) < abs(cands[best][2] - AIM_CM) - 0.05):
                best = i
    if best is None:
        for i in range(1, 4):
            if inwin(cands[i][2]) and (best is None or cands[i][3] < cands[best][3]):
                best = i
        if best is not None:
            fit = "OVER"
    if best is None:
        bo = 1e9
        for i in range(4):
            g, f = cands[i][2], cands[i][3]
            o = GAP_MIN_CM - g if g < GAP_MIN_CM else g - GAP_MAX_CM
            if o < bo - 0.05 or (abs(o - bo) <= 0.05 and f < cands[best][3]):
                bo, best = o, i
        fit = "OUTRANGE"
    return gap0, tin, tback, t_lat, t_fwd, cands, best, fit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arrow", choices=["L", "R", "l", "r"], help="arrow 2 direction")
    ap.add_argument("length", type=float, help="obstacle 2 length, cm = ENC_MM / 10 of the OLL after the 180 (main.c uses the encoder since 14 Oct)")
    ap.add_argument("--final", action="store_true", help="length already includes speed add + offset")
    ap.add_argument("--beside", action="store_true", help="the OLL after the 180 said START:BESIDE (adds the offset)")
    ap.add_argument("--drift", type=float, default=1.7, help="cm past the edge after the IR saw it")
    ap.add_argument("--fw2", type=float, default=FW_AFTER_OLL2_CM, help="FW after the 2nd OLL, cm")
    ap.add_argument("--ahead", type=float, help="cm from the robot after the exit 90 to obstacle 1's edge")
    a = ap.parse_args()

    arrow = a.arrow.upper()
    exit_turn = "LT90" if arrow == "R" else "RT90"
    exit_fwd = EXIT_LT90_FWD_CM if arrow == "R" else EXIT_RT90_FWD_CM
    toward = "LT" if arrow == "R" else "RT"          # -s2: the turn toward the centre line
    ir = "LEFT" if arrow == "R" else "RIGHT"
    ofs = LEN_OFFSET_CM if a.beside else 0
    L = a.length if a.final else a.length + LEN_SPEED_ADD_CM + ofs
    X = L / 2 + a.drift + a.fw2 + exit_fwd - IR_FWD_CM + X_TRIM_CM
    gap0, tin, tback, t_lat, t_fwd, cands, best, fit = plan(X, toward, AFTER_LOOP_FW_CM)
    name, d, g, f = cands[best]

    print(f"Arrow 2 {arrow}: exit {exit_turn}, {ir} IR watches obstacle 1")
    if a.final:
        print(f"Obstacle 2 length      L = {L:.1f} cm")
    else:
        print(f"Obstacle 2 length      L = {a.length:.1f} + {LEN_SPEED_ADD_CM} + {ofs} = {L:.1f} cm")
    print(f"Robot out from centre  X = {L/2:.2f} (L/2) + {a.drift:.1f} (drift) + {a.fw2:.0f} (FW) + "
          f"{exit_fwd:.0f} ({exit_turn}) - {IR_FWD_CM} (IR fwd) + {X_TRIM_CM} (trim) = {X:.2f} cm")
    print(f"IR -> obstacle 1 edge now = X - {OBS1_HALF_W_CM:.0f} - {IR_SIDE_CM:.0f} = {gap0:.2f} cm")
    print(f"Shift move {tin}45 FW d {tback}45: sideways {t_lat:.1f} + 0.707d, forward {t_fwd:.1f} + 0.707d")
    print()
    print("  candidate          cmd                   gap after   forward")
    for i, (n, dd, gg, ff) in enumerate(cands):
        c = "NONE" if dd < 0 else (f"{tin}45 {tback}45" if dd == 0 else f"{tin}45 FW{dd} {tback}45")
        print(f"{'>' if i == best else ' '} {n:17s}  {c:20s}  {gg:6.1f} cm  {ff:6.1f} cm")
    print()

    cmd = [] if d < 0 else ([f"{tin}45", f"{tback}45"] if d == 0 else [f"{tin}45", f"FW{d}", f"{tback}45"])
    steps = [exit_turn] + ([f"FW{AFTER_LOOP_FW_CM}"] if AFTER_LOOP_FW_CM > 0 else []) + cmd
    home = f"drive until {ir} IR <= {HOME_NEAR_CM} cm"
    if a.ahead is not None:
        left = a.ahead - f
        home += f" (about {left:.1f} cm straight)" if left > 0 else \
                f" (!! the correction needs {f:.1f} cm but obstacle 1 is {a.ahead:.1f} cm ahead)"
    note = {"OK": f"inside the {BUDGET_CM:.0f} cm budget",
            "OVER": f"OVER the {BUDGET_CM:.0f} cm budget by {f - BUDGET_CM:.1f} cm",
            "OUTRANGE": f"10-20 cm NOT reachable, nearest possible"}[fit]
    print("PATH: " + " -> ".join(steps + [home]))
    print(f"Expected IR -> obstacle 1 edge: {g:.1f} cm   correction forward: {f:.1f} cm ({note})")


if __name__ == "__main__":
    main()
