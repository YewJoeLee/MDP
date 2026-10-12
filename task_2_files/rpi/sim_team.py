"""Batch simulation of the TEAM algorithm (task2_team.py) in your sim.py arena.

    python3 sim_team.py              # every arrow pair x G1 x G2 x length x wall, with IE
    python3 sim_team.py --no-ie      # old firmware: FW3 + IR steps instead of IE
    python3 sim_team.py --one L R 100 100 60     # one layout, prints every command

A layout passes when the run returns OK, the robot ends fully inside the carpark and
it never came closer than 3 cm to anything (same pass rule as sim.py).
"""
import itertools
import random
import re
import sys

import config as C
import sim
import task2 as B
import task2_team as T

CPD = C.T2_CARPARK_DEPTH_CM
CPH = C.T2_CARPARK_HALF_CM


class TeamWorld(sim.World):
    """sim.World, but LT/RT of any angle use task2_team.turn_model (sim.World only
    knows the 45 and 10 degree models)."""

    def send(self, cmd):
        m = re.match(r"^(LT|RT)(\d+)$", cmd)
        if not m:
            return super().send(cmd)
        self.cmds.append(cmd)
        p = self.p
        f, r, dg = T.turn_model(cmd)
        q = p.copy(); q.move(f, r, dg)
        for i in range(1, 11):
            t = i / 10
            mid = p.copy()
            mid.x, mid.y, mid.h = p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t, p.h + (q.h - p.h) * t
            self.check(mid, cmd)
        p.x, p.y, p.h = q.x, q.y, q.h


def run(G1, G2, L, a1, a2, wall=50, noise=0.0, ie=True):
    w = TeamWorld(G1, G2, L, a1, a2, wall=wall, noise=noise, ir_fwd=C.T2_IR_FWD_CM, s=C.T2_IR_INSET_CM)
    out = T.run_task2(w.send, w.read_us, w.read_ir_left, w.read_ir_right, w.snap,
                      drive_edge=w.drive_edge if ie else None, cp_front_from_start=CPD - 18)
    p = w.p
    inside = (p.x + 12 <= 0 and p.x - 12 >= -CPD and abs(p.y) + 9.5 <= CPH)
    return out, w, inside


def main():
    ie = "--no-ie" not in sys.argv
    if "--one" in sys.argv:
        i = sys.argv.index("--one")
        a1, a2 = [("LEFT" if s.upper().startswith("L") else "RIGHT") for s in sys.argv[i + 1:i + 3]]
        G1, G2, L = (float(v) for v in sys.argv[i + 3:i + 6])
        out, w, inside = run(G1, G2, L, a1, a2, ie=ie)
        for k in ("start", "obs1", "obs2", "loop", "home", "park"):
            if k in out:
                r = dict(out[k]); log = r.pop("log", [])
                print(f"== {k}: {r}")
                for l in log:
                    print("    ", l)
        print("commands:", " ".join(w.cmds))
        mc = w.minclear
        print(f"status {out['status']} inside={inside} closest {mc[0]:.1f} cm to {mc[1]} during {mc[2]}"
              f" pose=({w.p.x:.1f},{w.p.y:.1f},{w.p.h:.0f})")
        return
    random.seed(1)
    bad = n = 0
    why = {}
    allowed = re.compile(r"^(FW|BW|LT|RT)\d+$|^IE[LR][NA]\d+$")
    for a1, a2 in itertools.product(("LEFT", "RIGHT"), repeat=2):
        for G1 in (60, 100, 150):
            for G2 in (60, 100, 150):
                for L in (30, 40, 60, 90, 130):
                    for wall in (50, 0):
                        n += 1
                        try:
                            out, w, inside = run(G1, G2, L, a1, a2, wall=wall, ie=ie)
                        except Exception:
                            import traceback; traceback.print_exc()
                            print("EXC", a1, a2, G1, G2, L, wall); bad += 1; continue
                        badcmd = [c for c in w.cmds if not allowed.match(c)]
                        mc = w.minclear
                        ok = out["status"] == "OK" and inside and mc[0] >= 3 and not badcmd
                        if not ok:
                            bad += 1
                            k = (out["status"] if out["status"] != "OK" else
                                 ("OUTSIDE" if not inside else f"CLOSE {mc[1]} during {mc[2][:2]}"))
                            why[k] = why.get(k, 0) + 1
                            if bad <= 40:
                                print(f"{a1[0]}{a2[0]} G1={G1} G2={G2} L={L} wall={wall}: {out['status']} "
                                      f"inside={inside} minclear={mc[0]:.1f}({mc[1]},{mc[2]}) "
                                      f"pose=({w.p.x:.1f},{w.p.y:.1f},{w.p.h:.0f}) {badcmd[:3]}")
    for k, v in sorted(why.items(), key=lambda kv: -kv[1]):
        print(f"   {v:3d} x {k}")
    print(f"{n - bad}/{n} OK")


if __name__ == "__main__":
    main()
