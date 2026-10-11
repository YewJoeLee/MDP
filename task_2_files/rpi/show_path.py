"""Task 2 (fastest car): run the real task2.run_task2() in the simulated arena and draw the route.

Usage (laptop, in this folder):
    Edit the "EDIT HERE" block below (arrows, G1, G2, obstacle 2 length), then:
    python show_path.py                                   # defaults: G1=100 G2=100 L=60, LEFT / RIGHT
    python show_path.py --a1 R --a2 L                     # arrow on obstacle 1 / obstacle 2
    python show_path.py --g1 60 --g2 150 --length 90      # distances and obstacle 2 length (cm)
    python show_path.py --wall 0                          # no side walls
    python show_path.py --noise 1 --seed 3                # +-1 cm sensor noise
    python show_path.py --all                             # all 4 arrow combinations, one page

Arena layout (same as the brief's picture, seen from above):
    carpark (60 x 50 cm inside) on the left, opening facing right
    obstacle 1 (10 x 10)  G1 cm after the carpark opening      (60-150)
    obstacle 2 (10 x L)   G2 cm after obstacle 1's back face   (60-150), L >= 30
    optional walls        `wall` cm beyond each end of obstacle 2 (50 minimum in the rules)

Uses sim.World for the ultrasonic / IR / camera, so every decision is the one
task2.py really makes. Turns are drawn (and clearance-checked) along a smooth
arc ending on the TURN45_MODELS / SMALL_TURN_MODELS end point in config.py.
Prints the commands per step, then writes task2_path_view.html and opens it.
config.py and task2.py are not changed.
"""
import argparse
import html
import math
import os
import random
import sys
import webbrowser

import config as C
import sim
import task2 as T

# =============================================================================
#  EDIT HERE - the arena to simulate
# =============================================================================
ARROW_1 = "LEFT"        # arrow on obstacle 1: "LEFT" or "RIGHT"
ARROW_2 = "RIGHT"       # arrow on obstacle 2: "LEFT" or "RIGHT"
G1_CM = 100             # carpark opening -> obstacle 1 face       (rules: 60 - 150)
G2_CM = 100             # obstacle 1 back face -> obstacle 2 face  (rules: 60 - 150)
LENGTH_CM = 60          # obstacle 2 length                        (rules: 30 minimum)
WALL_CM = 50            # wall gap beyond each end of obstacle 2   (rules: 50 minimum, 0 = no wall)
ALL_ARROWS = False      # True = draw all 4 arrow combinations on one page
USE_IE = True           # True = STM IE command (one move per IR leg); False = old FW3 + IR steps
# =============================================================================

W, LR, A = C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM, C.SENSOR_FORWARD_CM

# run_task2 step function -> (short label, colour)
PHASES = [
    ("start_to_obs1",          "0 · Start → obstacle 1",        "#2a9d8f"),
    ("dodge_plan",             "1 · Dodge obstacle 1",          "#1f77b4"),
    ("pass_obs1_measure_ir",   "1b · Pass obstacle 1 (IR)",     "#6a4c93"),
    ("approach_and_snap_obs2", "2 · Approach + snap obstacle 2", "#e07a00"),
    ("loop_obs2",              "3 · Loop obstacle 2",           "#d62728"),
    ("return_to_carpark_edge", "4 · Back to carpark",           "#c2185b"),
    ("park_in_carpark",        "5 · Park",                      "#2e7d32"),
]
PHASE_LABEL = {k: lab for k, lab, _ in PHASES}
PHASE_COLOUR = {k: c for k, _, c in PHASES}
RESULT_KEY = {"start_to_obs1": "start", "dodge_plan": "dodge", "pass_obs1_measure_ir": "pass_obs1",
              "approach_and_snap_obs2": "obs2", "loop_obs2": "loop",
              "return_to_carpark_edge": "home", "park_in_carpark": "park"}
HEADLINE = ("obs1", "obs2", "wall_up", "wall_dn")      # what "closest approach" refers to
NICE = {"arm_up": "carpark arm (left)", "arm_dn": "carpark arm (right)", "back": "carpark back wall",
        "obs1": "obstacle 1", "obs2": "obstacle 2", "wall_up": "left wall", "wall_dn": "right wall"}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--g1", type=float, default=G1_CM, help="carpark opening -> obstacle 1 face (cm)")
    ap.add_argument("--g2", type=float, default=G2_CM, help="obstacle 1 back -> obstacle 2 face (cm)")
    ap.add_argument("--length", type=float, default=LENGTH_CM, help="obstacle 2 length (cm, >= 30)")
    ap.add_argument("--a1", default=ARROW_1, help="obstacle 1 arrow: L or R")
    ap.add_argument("--a2", default=ARROW_2, help="obstacle 2 arrow: L or R")
    ap.add_argument("--wall", type=float, default=WALL_CM, help="wall gap past obstacle 2's ends (0 = no wall)")
    ap.add_argument("--rear-gap", type=float, default=6, help="start: robot rear -> carpark back wall (cm)")
    ap.add_argument("--noise", type=float, default=0.0, help="+- sensor noise (cm)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--all", action="store_true", default=ALL_ARROWS, help="draw all four arrow combinations")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser")
    return ap.parse_args()


def arrow_name(s):
    return "LEFT" if s.strip().upper().startswith("L") else "RIGHT"


# =============================================================================
# Recording world: sim.World + a pose trace, smooth turn arcs, phase tags
# =============================================================================

def turn_arc(p, f, r, dg, n=None):
    """Poses along a turn that ends at (forward f, right r, dg deg clockwise) from p.
    ahead = f*sin(t)/sin(th), right = r*(1-cos t)/(1-cos th): the same ellipse
    shape the Task 1 planner draws, landing exactly on the config end point."""
    th = math.radians(abs(dg))
    n = n or max(4, int(abs(dg) / 3))
    sgn = 1 if dg > 0 else -1
    out = []
    for i in range(1, n + 1):
        t = th * i / n
        q = p.copy()
        q.move(f * math.sin(t) / math.sin(th), r * (1 - math.cos(t)) / (1 - math.cos(th)), 0)
        q.h = p.h - sgn * math.degrees(t)
        out.append(q)
    return out


class RecWorld(sim.World):
    def __init__(self, *a, rear_gap=6.0, **k):
        super().__init__(*a, **k)
        self.p.x = -50 + LR / 2 + rear_gap
        self.phase = "start_to_obs1"
        self.trace = [(self.p.x, self.p.y, self.p.h, self.phase, None)]
        self.cmd_log = []                      # (phase, cmd)
        self.snaps = []                        # (phase, x, y, h, result)
        self.phase_end = {}                    # phase -> pose at its last move
        self.min_by_obj = {}                   # box -> (gap, x, y, h, cmd)
        self.arc_stack = []

    def visit(self, q, cmd):
        for name, b in self.boxes.items():
            g = T._box_clear(q, b, W, LR)
            if name not in self.min_by_obj or g < self.min_by_obj[name][0]:
                self.min_by_obj[name] = (g, q.x, q.y, q.h, cmd)
        self.trace.append((q.x, q.y, q.h, self.phase, cmd))
        self.phase_end[self.phase] = (q.x, q.y, q.h)

    def send(self, cmd):
        self.cmds.append(cmd)
        self.cmd_log.append((self.phase, cmd))
        p = self.p
        if cmd[:2] in ("FW", "BW"):
            n = int(cmd[2:]) * (1 if cmd[:2] == "FW" else -1)
            for _ in range(abs(n)):
                p.move(1 if n > 0 else -1)
                self.visit(p, cmd)
            return
        if cmd[0] == "X" and cmd[1:] in ("R10", "L10"):   # undo: retrace the arc backwards
            pts = self.arc_stack.pop()
            for q in reversed(pts[:-1]):
                self.visit(q, cmd)
            q = pts[0]
        else:
            f, r, dg = T._turn_model(cmd, 1)
            pts = [p.copy()] + turn_arc(p, f, r, dg)
            if cmd in ("RT10", "LT10"):
                self.arc_stack.append(pts)
            for q in pts[1:]:
                self.visit(q, cmd)
            q = pts[-1]
        p.x, p.y, p.h = q.x, q.y, q.h

    def _advance(self, cm, label):                 # IE legs: record the trace too
        self.p.move(cm)
        self.visit(self.p, label)

    def drive_edge(self, side, need_seen, max_cm, **k):
        self.cmd_log.append((self.phase, f"IE{side[0]}{'N' if need_seen else 'A'}{int(max_cm)}"))
        return super().drive_edge(side, need_seen, max_cm, **k)

    def snap(self):
        res = super().snap()
        self.snaps.append((self.phase, self.p.x, self.p.y, self.p.h, res))
        return res


def run_scenario(g1, g2, length, a1, a2, wall, rear_gap, noise, seed):
    random.seed(seed)
    w = RecWorld(g1, g2, length, a1, a2, wall=wall, noise=noise,
                 ir_fwd=getattr(C, "T2_IR_FWD_CM", 0.0), s=getattr(C, "T2_IR_INSET_CM", 0.0),
                 rear_gap=rear_gap)
    originals = {}
    for fname, _, _ in PHASES:                 # tag every move with the step that sent it
        orig = getattr(T, fname)
        originals[fname] = orig

        def wrapped(*a, _f=fname, _o=orig, **k):
            w.phase = _f
            return _o(*a, **k)
        setattr(T, fname, wrapped)
    error = None
    try:
        out = T.run_task2(w.send, w.read_us, w.read_ir_left, w.read_ir_right, w.snap,
                          drive_edge=w.drive_edge if USE_IE else None)
    except Exception as e:                      # still draw what happened up to the crash
        import traceback
        error = traceback.format_exc()
        out = {"status": f"EXCEPTION: {e}"}
    finally:
        for k, v in originals.items():
            setattr(T, k, v)
    p = w.p
    hr = math.radians(p.h)
    xs = [p.x + f * LR / 2 * math.cos(hr) + s * W / 2 * math.sin(hr) for f in (1, -1) for s in (1, -1)]
    ys = [p.y + f * LR / 2 * math.sin(hr) - s * W / 2 * math.cos(hr) for f in (1, -1) for s in (1, -1)]
    inside = max(xs) <= 0 and min(xs) >= -50 and max(ys) <= 30 and min(ys) >= -30
    return {"world": w, "out": out, "inside": inside, "error": error,
            "args": dict(g1=g1, g2=g2, length=length, a1=a1, a2=a2, wall=wall)}


# =============================================================================
# Text report
# =============================================================================

def closest(w):
    """(name, (gap, x, y, h, cmd)) of the closest approach to an obstacle or wall.
    The carpark parts are left out: the robot starts a few cm off the back wall."""
    items = [kv for kv in w.min_by_obj.items() if kv[0] in HEADLINE] or list(w.min_by_obj.items())
    return min(items, key=lambda kv: kv[1][0])


def path_length(trace):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(trace, trace[1:]))


def print_report(r):
    w, out, ar = r["world"], r["out"], r["args"]
    print(f"\n=== G1={ar['g1']:g}  G2={ar['g2']:g}  L={ar['length']:g}  wall={ar['wall']:g}  "
          f"arrows {ar['a1']} / {ar['a2']} ===")
    for fname, label, _ in PHASES:
        cmds = [c for ph, c in w.cmd_log if ph == fname]
        res = out.get(RESULT_KEY[fname])
        if not cmds and res is None:
            continue
        st = "" if not isinstance(res, dict) else f"  [{res.get('status', 'ok' if res.get('safe', True) else 'UNSAFE')}]"
        print(f"{label}{st}")
        print("   " + (" ".join(cmds) if cmds else "(no moves)"))
    worst = closest(w)
    print(f"Status {out['status']}  ·  parked inside: {r['inside']}  ·  {len(w.cmd_log)} commands  ·  "
          f"path {path_length(w.trace):.0f} cm")
    print(f"Closest approach {worst[1][0]:.1f} cm to {NICE.get(worst[0], worst[0])} (during {worst[1][4]})"
          f"  ·  MIN_CLEARANCE_CM = {C.MIN_CLEARANCE_CM:g}")
    if r["error"]:
        print(r["error"])


# =============================================================================
# Drawing (SVG: x = forward from the carpark opening -> right, y = robot's left -> up)
# =============================================================================

def P(x, y):
    return f"{x:.1f},{-y:.1f}"


def robot_svg(x, y, h, colour, dash="", fill="none", width=1.0, opacity=1.0):
    hr = math.radians(h)
    c, s = math.cos(hr), math.sin(hr)
    pts = []
    for f, side in ((1, 1), (1, -1), (-1, -1), (-1, 1)):     # side +1 = left
        pts.append(P(x + f * LR / 2 * c - side * W / 2 * s, y + f * LR / 2 * s + side * W / 2 * c))
    nose = P(x + LR / 2 * c, y + LR / 2 * s)
    return (f'<g opacity="{opacity}"><polygon points="{" ".join(pts)}" fill="{fill}" stroke="{colour}" '
            f'stroke-width="{width}" {dash}/><line x1="{x:.1f}" y1="{-y:.1f}" x2="{nose.split(",")[0]}" '
            f'y2="{nose.split(",")[1]}" stroke="{colour}" stroke-width="{width * 1.4}"/></g>')


def box_svg(b, fill, stroke="none", extra=""):
    x0, x1, y0, y1 = b
    return (f'<rect x="{x0}" y="{-y1}" width="{x1 - x0}" height="{y1 - y0}" fill="{fill}" '
            f'stroke="{stroke}" {extra}/>')


def arrow_glyph(x, y, direction, colour="#fff"):
    """Small arrow image on an obstacle face, as seen by the robot (LEFT = towards +y)."""
    d = 1 if direction == "LEFT" else -1
    return (f'<path d="M{x:.1f},{-y:.1f} m0,{-d * 3.6} l-2.6,{d * 3.2} h1.6 v{d * 3.2} h2 v{-d * 3.2} h1.6 z" '
            f'fill="{colour}"/>')


def dim_line(x0, y0, x1, y1, text, colour="#888", off=(0, 0)):
    mx, my = (x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1]
    return (f'<line x1="{x0}" y1="{-y0}" x2="{x1}" y2="{-y1}" stroke="{colour}" stroke-width="0.6" '
            f'marker-start="url(#dimA)" marker-end="url(#dimA)"/>'
            f'<text x="{mx}" y="{-my}" font-size="5" fill="{colour}" text-anchor="middle">{text}</text>')


def chevrons(trace, colour_of, every=28.0):
    out, acc = [], every * 0.5
    for a, b in zip(trace, trace[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        acc += seg
        if acc >= every and seg > 0.2:
            acc = 0
            ang = math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))
            out.append(f'<path d="M-2.2,-1.8 L1.4,0 L-2.2,1.8" fill="none" stroke="{colour_of(b[3])}" '
                       f'stroke-width="0.9" transform="translate({b[0]:.1f},{-b[1]:.1f}) rotate({ang:.1f})"/>')
    return "".join(out)


def draw(r, uid):
    w, ar = r["world"], r["args"]
    B = w.boxes
    x2, L = w.x2, w.L
    wall = ar["wall"]
    ext_y = L / 2 + (wall + 12 if wall else 45)
    xmin, xmax = -75, x2 + 10 + 75
    ymin, ymax = -ext_y - 8, ext_y + 8
    parts = []

    # floor grid every 10 cm
    for gx in range(int(xmin // 10) * 10, int(xmax) + 1, 10):
        parts.append(f'<line x1="{gx}" y1="{-ymax}" x2="{gx}" y2="{-ymin}" class="grid"/>')
    for gy in range(int(ymin // 10) * 10, int(ymax) + 1, 10):
        parts.append(f'<line x1="{xmin}" y1="{-gy}" x2="{xmax}" y2="{-gy}" class="grid"/>')

    # carpark (inside 60 x 50) + carpark centre line
    parts.append(box_svg((-50, 0, -30, 30), "#fff4a8", extra='opacity="0.55"'))
    for k in ("arm_up", "arm_dn", "back"):
        parts.append(box_svg(B[k], "#5872c0"))
    parts.append(f'<line x1="{xmin}" y1="0" x2="{xmax}" y2="0" stroke="#9aa" stroke-width="0.4" '
                 f'stroke-dasharray="3,3"/>')
    parts.append('<text x="-25" y="-33" font-size="5" text-anchor="middle" fill="#665c00">CARPARK 60×50</text>')

    # walls
    for k in ("wall_up", "wall_dn"):
        if k in B:
            x0, x1, y0, y1 = B[k]
            parts.append(box_svg((0, x2 + 60, y0, y1), "#111"))
    # obstacles
    parts.append(box_svg(B["obs1"], "#5872c0"))
    parts.append(box_svg(B["obs2"], "#5872c0"))
    o1x = B["obs1"][0]
    parts.append(f'<circle cx="{o1x - 1.4}" cy="0" r="1.6" fill="#1a3cff"/>')
    parts.append(f'<circle cx="{x2 - 1.4}" cy="0" r="1.6" fill="#1a3cff"/>')
    parts.append(arrow_glyph(o1x + 5, 0, ar["a1"]))
    parts.append(arrow_glyph(x2 + 5, 0, ar["a2"]))
    lab_y = -9 if ar["a1"] == "LEFT" else 9                  # label on the side the robot does NOT pass
    parts.append(f'<text x="{o1x + 5}" y="{-lab_y + 2}" font-size="5" text-anchor="middle" class="lab">'
                 f'Obs 1 · {ar["a1"]}</text>')
    parts.append(f'<text x="{x2 - 4}" y="{-(L / 2) + 4}" font-size="5" text-anchor="end" class="lab">'
                 f'Obs 2 · {ar["a2"]} · L={L:g}</text>')

    # dimensions
    dy = -L / 2 - 14
    parts.append(dim_line(0, dy, o1x, dy, f"G1 {ar['g1']:g}", off=(0, 2)))
    parts.append(dim_line(o1x + 10, dy, x2, dy, f"G2 {ar['g2']:g}", off=(0, 2)))
    if wall:
        parts.append(dim_line(x2 + 55, L / 2, x2 + 55, L / 2 + wall, f"{wall:g}", off=(5, 0)))
        parts.append(dim_line(x2 + 55, -L / 2, x2 + 55, -L / 2 - wall, f"{wall:g}", off=(5, 0)))
        parts.append(f'<line x1="{x2 + 10}" y1="{-L / 2}" x2="{x2 + 58}" y2="{-L / 2}" stroke="#bbb" '
                     f'stroke-width="0.4" stroke-dasharray="1,1"/>'
                     f'<line x1="{x2 + 10}" y1="{L / 2}" x2="{x2 + 58}" y2="{L / 2}" stroke="#bbb" '
                     f'stroke-width="0.4" stroke-dasharray="1,1"/>')

    # path, coloured by step
    tr = w.trace
    runs, cur = [], [tr[0]]
    for q in tr[1:]:
        if q[3] != cur[-1][3]:
            runs.append(cur)
            cur = [cur[-1][:3] + (q[3], q[4])]
        cur.append(q)
    runs.append(cur)
    for run in runs:
        c = PHASE_COLOUR.get(run[-1][3], "#444")
        parts.append(f'<polyline fill="none" stroke="{c}" stroke-width="1.3" stroke-linejoin="round" '
                     f'points="{" ".join(P(q[0], q[1]) for q in run)}"/>')
    parts.append(chevrons(tr, lambda ph: PHASE_COLOUR.get(ph, "#444")))

    # ghost robot at the end of each step, snap poses, start and final
    for fname, _, colour in PHASES:
        if fname in w.phase_end and fname != "park_in_carpark":
            x, y, h = w.phase_end[fname]
            parts.append(robot_svg(x, y, h, colour, 'stroke-dasharray="2,1.2"', width=0.7, opacity=0.8))
    for i, (ph, x, y, h, res) in enumerate(w.snaps):
        c = PHASE_COLOUR.get(ph, "#444")
        parts.append(robot_svg(x, y, h, c, width=0.5, opacity=0.6))
        hr = math.radians(h)
        cx, cy = x + A * math.cos(hr), y + A * math.sin(hr)
        lab = (res or {}).get("label", "—") if res else "nothing"
        parts.append(f'<g class="snap"><circle cx="{cx:.1f}" cy="{-cy:.1f}" r="2.1" fill="#fff" '
                     f'stroke="{c}" stroke-width="0.8"/><circle cx="{cx:.1f}" cy="{-cy:.1f}" r="0.9" fill="{c}"/>'
                     f'<title>Photo {i + 1} ({PHASE_LABEL.get(ph, ph)}): {html.escape(str(lab))}</title></g>')
    x0, y0, h0 = tr[0][:3]
    parts.append(robot_svg(x0, y0, h0, "#2a7", width=1.0, fill="rgba(42,170,119,0.12)"))
    p = w.p
    parts.append(robot_svg(p.x, p.y, p.h, "#111", width=1.2,
                           fill="rgba(46,125,50,0.25)" if r["inside"] else "rgba(214,39,40,0.25)"))

    # closest approach marker
    worst = closest(w)
    g, wx, wy, wh, wcmd = worst[1]
    ok = g >= C.MIN_CLEARANCE_CM
    parts.append(robot_svg(wx, wy, wh, "#d00" if not ok else "#999", 'stroke-dasharray="1,1"', width=0.6))
    parts.append(f'<circle cx="{wx:.1f}" cy="{-wy:.1f}" r="5" fill="none" stroke="{"#d00" if not ok else "#999"}" '
                 f'stroke-width="0.6"><title>Closest: {g:.1f} cm to {NICE.get(worst[0], worst[0])} '
                 f'during {wcmd}</title></circle>')

    vb = f"{xmin} {-ymax} {xmax - xmin} {ymax - ymin}"
    return (f'<svg viewBox="{vb}" class="arena" xmlns="http://www.w3.org/2000/svg">'
            f'<defs><marker id="dimA" viewBox="0 0 6 6" refX="3" refY="3" markerWidth="4" markerHeight="4" '
            f'orient="auto-start-reverse"><path d="M0,0 L6,3 L0,6 z" fill="#888"/></marker></defs>'
            f'<rect x="{xmin}" y="{-ymax}" width="{xmax - xmin}" height="{ymax - ymin}" fill="#fff"/>'
            + "".join(parts) + "</svg>")


def summarise(res):
    if not isinstance(res, dict):
        return ""
    keep = []
    for k, v in res.items():
        if k in ("log", "photo"):
            continue
        if isinstance(v, float):
            v = round(v, 1)
        elif isinstance(v, (tuple, list)) and all(isinstance(e, (int, float)) for e in v):
            v = "(" + ", ".join(f"{e:.1f}" if isinstance(e, float) else str(e) for e in v) + ")"
        keep.append(f"{k}={v}")
    return ", ".join(keep)


def scenario_html(r, uid):
    w, out, ar = r["world"], r["out"], r["args"]
    worst = closest(w)
    ok = out.get("status") == "OK" and r["inside"] and worst[1][0] >= C.MIN_CLEARANCE_CM
    badge = (f'<span class="badge {"ok" if ok else "bad"}">{"PASS" if ok else "CHECK"}</span>')
    steps = []
    for fname, label, colour in PHASES:
        cmds = [c for ph, c in w.cmd_log if ph == fname]
        res = out.get(RESULT_KEY[fname])
        if not cmds and res is None:
            continue
        log = res.get("log", []) if isinstance(res, dict) else []
        log_html = ("<details><summary>log ({})</summary><pre>{}</pre></details>".format(
            len(log), html.escape("\n".join(map(str, log))))) if log else ""
        steps.append(f'<li><b style="color:{colour}">{label}</b> <span class="muted">{len(cmds)} cmds</span><br>'
                     f'<code>{html.escape(" ".join(cmds)) or "(no moves)"}</code>'
                     f'<div class="muted small">{html.escape(summarise(res))}</div>{log_html}</li>')
    clear_rows = "".join(
        f'<tr class="{"warn" if v[0] < C.MIN_CLEARANCE_CM else ""}"><td>{NICE.get(k, k)}</td>'
        f'<td>{v[0]:.1f} cm</td><td>{html.escape(str(v[4]))}</td></tr>'
        for k, v in sorted(w.min_by_obj.items(), key=lambda kv: kv[1][0]))
    err = f'<pre class="err">{html.escape(r["error"])}</pre>' if r["error"] else ""
    return f"""
<section class="scenario">
  <h2>{badge} Arrows {ar['a1']} / {ar['a2']} · G1 {ar['g1']:g} · G2 {ar['g2']:g} · L {ar['length']:g} ·
      wall {format(ar['wall'], 'g') if ar['wall'] else 'none'}</h2>
  <p class="meta">Status <b>{html.escape(str(out.get('status')))}</b> · parked inside: <b>{r['inside']}</b> ·
     {len(w.cmd_log)} commands · path {path_length(w.trace):.0f} cm · {len(w.snaps)} photos ·
     closest {worst[1][0]:.1f} cm to {NICE.get(worst[0], worst[0])}</p>
  <div class="wrap">
    <div class="figure">{draw(r, uid)}</div>
    <div class="side">
      <ol class="steps">{''.join(steps)}</ol>
      <table class="clear"><tr><th>Closest to</th><th>gap</th><th>during</th></tr>{clear_rows}</table>
      {err}
    </div>
  </div>
</section>"""


def page(results):
    legend = "".join(f'<span><i style="background:{c}"></i>{lab}</span>' for _, lab, c in PHASES)
    body = "".join(scenario_html(r, i) for i, r in enumerate(results))
    return f"""<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Task 2 path</title>
<style>
 body{{font-family:system-ui,sans-serif;margin:16px;color:#222;background:#fafafa}}
 h1{{font-size:20px;margin:0 0 6px}} h2{{font-size:16px;margin:0 0 4px}}
 .legend{{display:flex;flex-wrap:wrap;gap:10px 16px;font-size:13px;margin:6px 0 14px}}
 .legend i{{display:inline-block;width:18px;height:4px;border-radius:2px;margin-right:6px;vertical-align:middle}}
 .scenario{{background:#fff;border:1px solid #e3e3e3;border-radius:8px;padding:14px;margin-bottom:18px}}
 .wrap{{display:flex;gap:20px;flex-wrap:wrap;align-items:flex-start}}
 .figure{{flex:1 1 640px;min-width:320px}} .arena{{width:100%;height:auto;border:1px solid #ddd}}
 .side{{flex:1 1 340px;max-width:620px}}
 .grid{{stroke:#eee;stroke-width:0.3}}
 .lab{{fill:#334;stroke:#fff;stroke-width:1.6px;paint-order:stroke;font-weight:600}}
 .steps{{list-style:none;padding:0;margin:0;line-height:1.5}} .steps li{{margin-bottom:9px}}
 code{{font-size:12px;background:#f3f3f3;padding:1px 4px;border-radius:3px;word-break:break-word}}
 pre{{font-size:11px;background:#f6f6f6;padding:6px;white-space:pre-wrap;margin:4px 0}}
 .muted{{color:#777}} .small{{font-size:12px}} .meta{{font-size:13px;color:#444;margin:0 0 10px}}
 .badge{{font-size:12px;padding:2px 7px;border-radius:10px;color:#fff;vertical-align:middle}}
 .ok{{background:#2e7d32}} .bad{{background:#d62728}}
 .clear{{border-collapse:collapse;font-size:12px;margin-top:8px}} .clear td,.clear th{{padding:2px 8px;border-bottom:1px solid #eee;text-align:left}}
 .clear tr.warn td{{color:#d00;font-weight:600}} .err{{color:#d00}}
 details summary{{cursor:pointer;font-size:12px;color:#555}}
</style>
<h1>Task 2 – fastest car: simulated run</h1>
<div class="legend">{legend}</div>
<p class="small muted">Blue blocks: carpark arms / back wall and obstacles (blue dot = arrow image, white arrow = its direction).
Black bars: possible walls. Yellow: carpark inside. Green box: start pose. Dashed boxes: robot at the end of each step.
Faint boxes with a ring: photo poses (ring = camera/ultrasonic). Solid box: final pose (green fill = fully inside the carpark).
Dotted box + circle: the closest approach to anything (red if below MIN_CLEARANCE_CM = {C.MIN_CLEARANCE_CM:g} cm).
Hover a ring or circle for details. Turns are drawn as smooth arcs ending on the config.py turn models.</p>
{body}"""


def check_settings(args):
    """Warn (but still run) when a value is outside what the rules allow."""
    for name, v in (("G1", args.g1), ("G2", args.g2)):
        if not 60 <= v <= 150:
            print(f"WARNING: {name} = {v:g} cm is outside the rules' 60-150 cm")
    if args.length < 30:
        print(f"WARNING: obstacle 2 length {args.length:g} cm is below the rules' 30 cm minimum")
    if 0 < args.wall < 50:
        print(f"WARNING: wall gap {args.wall:g} cm is below the rules' 50 cm minimum")
    for name, v in (("ARROW_1", args.a1), ("ARROW_2", args.a2)):
        if not str(v).strip().upper().startswith(("L", "R")):
            sys.exit(f"{name} must be LEFT or RIGHT, not {v!r}")


def main():
    args = parse_args()
    check_settings(args)
    combos = ([(x, y) for x in ("LEFT", "RIGHT") for y in ("LEFT", "RIGHT")] if args.all
              else [(arrow_name(args.a1), arrow_name(args.a2))])
    results = []
    for a1, a2 in combos:
        r = run_scenario(args.g1, args.g2, args.length, a1, a2, args.wall,
                         args.rear_gap, args.noise, args.seed)
        print_report(r)
        results.append(r)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "task2_path_view.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(page(results))
    print(f"\nDrawing written to {out}")
    if not args.no_open:
        webbrowser.open("file://" + out)


if __name__ == "__main__":
    main()
