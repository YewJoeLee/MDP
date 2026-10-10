"""Plan the obstacles in test_maps.py and show the route.

Usage (laptop, in this folder):
    python show_path.py                  # uses config.py as-is
    python show_path.py --clearance 5    # try a different safety distance
    python show_path.py --no-align       # plan without AC20/RA

Prints the order, commands per obstacle and skip reasons, then writes
path_view.html (open it in a browser) showing the arena, obstacles, image
faces, photo poses and the robot-centre path including turn arcs.
Only changes config values in memory; config.py itself is not edited.
"""
import argparse
import html
import math
import os
import webbrowser

import config
import planner
import test_maps


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clearance", type=float, help="override MIN_CLEARANCE_CM")
    ap.add_argument("--align-cm", type=float, help="override ALIGN_MAX_TRAVEL_CM")
    ap.add_argument("--no-align", action="store_true", help="plan without AC20/RA")
    ap.add_argument("--no-open", action="store_true", help="do not open the browser")
    return ap.parse_args()


def centre_path(start, moves):
    """Robot-centre points along the planned moves, with turns drawn as the
    same quarter-ellipse the planner models."""
    pts, pose = [start[:2]], start
    for m in moves:
        if m in ("FW", "BW"):
            pose = planner.apply(pose, m)
            pts.append(pose[:2])
            continue
        a, r, q = config.TURN_MODELS[m]
        fx, fy = config.DIRECTION_STEP[pose[2]]
        rx, ry = fy, -fx
        for i in range(1, 16):
            t = i*math.pi/2/15
            ahead, right = a*math.sin(t), r*(1-math.cos(t))
            pts.append((pose[0]+fx*ahead+rx*right, pose[1]+fy*ahead+ry*right))
        pose = planner.apply(pose, m)
    return pts


def robot_rect(pose, colour, dash=""):
    w, l = config.ROBOT_WIDTH_CM, config.ROBOT_LENGTH_CM
    horiz = pose[2] in ("E", "W")
    bw, bh = (l, w) if horiz else (w, l)
    fx, fy = config.DIRECTION_STEP[pose[2]]
    nose = (pose[0]+fx*l/2, pose[1]+fy*l/2)
    return (f'<rect x="{pose[0]-bw/2}" y="{pose[1]-bh/2}" width="{bw}" height="{bh}" '
            f'fill="none" stroke="{colour}" stroke-width="1" {dash}/>'
            f'<line x1="{pose[0]}" y1="{pose[1]}" x2="{nose[0]}" y2="{nose[1]}" stroke="{colour}" stroke-width="1.5"/>')


def main():
    args = parse_args()
    if args.clearance is not None:
        config.MIN_CLEARANCE_CM = args.clearance
    if args.align_cm is not None:
        config.ALIGN_MAX_TRAVEL_CM = args.align_cm
    if args.no_align:
        config.ALIGN_ENABLED = False

    obstacles = planner.normalize_obstacles(test_maps.OBSTACLES)
    result = planner.plan(obstacles)

    print(f"Clearance {config.MIN_CLEARANCE_CM} cm (+{config.POSITION_MARGIN_CM} margin), "
          f"AC20 {'+/-'+format(config.ALIGN_MAX_TRAVEL_CM, 'g')+' cm' if config.ALIGN_ENABLED else 'off'}")
    print(f"Visited {len(result['order'])}/{len(obstacles)}  order {result['order']}  "
          f"cost {result['total_cost']}  planned in {result['planning_ms']/1000:.2f} s\n")
    for seg in result["segments"]:
        print(f"Obstacle {seg['obstacle_id']}: {tuple(seg['start_pose'])} -> {tuple(seg['goal_pose'])}  cost {seg['cost']:.2f}")
        print("   " + " ".join(seg["hardware_commands"]))
    for s in result["skipped"]:
        print(f"SKIPPED {s['obstacle_id']}: {s['reason']}")

    # ---- drawing (SVG, y flipped so north is up) ----
    size = config.GRID*config.CELL_CM
    cell = config.CELL_CM
    parts = []
    for i in range(config.GRID+1):
        parts.append(f'<line x1="{i*cell}" y1="0" x2="{i*cell}" y2="{size}" stroke="#ddd" stroke-width="0.3"/>')
        parts.append(f'<line x1="0" y1="{i*cell}" x2="{size}" y2="{i*cell}" stroke="#ddd" stroke-width="0.3"/>')
    skipped = {s["obstacle_id"] for s in result["skipped"]}
    for oid, x, y, face in obstacles:
        x0, y0 = x*cell, y*cell
        colour = "#999" if oid in skipped else "#333"
        parts.append(f'<rect x="{x0}" y="{y0}" width="{cell}" height="{cell}" fill="{colour}"/>')
        fx, fy = config.DIRECTION_STEP[face]
        cx, cy = x0+cell/2, y0+cell/2
        ex, ey = cx+fx*cell/2, cy+fy*cell/2
        dx, dy = (cell/2*abs(fy), cell/2*abs(fx))
        parts.append(f'<line x1="{ex-dx}" y1="{ey-dy}" x2="{ex+dx}" y2="{ey+dy}" stroke="#e8a000" stroke-width="2.5"/>')
        parts.append(f'<text x="{cx}" y="{-(cy)-0.5}" transform="scale(1,-1)" font-size="6" text-anchor="middle" '
                     f'dominant-baseline="middle" fill="white">{oid}</text>')
    start = tuple(result["start"])
    parts.append(robot_rect(start, "#2a7"))
    colours = ["#1f77b4", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#ff7f0e"]
    for i, seg in enumerate(result["segments"]):
        c = colours[i % len(colours)]
        pts = centre_path(tuple(seg["start_pose"]), seg["moves"])
        parts.append('<polyline fill="none" stroke="%s" stroke-width="1.2" points="%s"/>'
                     % (c, " ".join(f"{p[0]:.1f},{p[1]:.1f}" for p in pts)))
        goal = tuple(seg["goal_pose"])
        parts.append(robot_rect(goal, c, 'stroke-dasharray="2,1"'))
        parts.append(f'<text x="{goal[0]}" y="{-goal[1]}" transform="scale(1,-1)" font-size="7" '
                     f'text-anchor="middle" dominant-baseline="middle" fill="{c}" font-weight="bold">{i+1}</text>')
    svg = (f'<svg viewBox="-5 -5 {size+10} {size+10}" width="640" height="640" xmlns="http://www.w3.org/2000/svg">'
           f'<g transform="translate(0,{size}) scale(1,-1)">'
           f'<rect x="0" y="0" width="{size}" height="{size}" fill="white" stroke="#000"/>'
           + "".join(parts) + "</g></svg>")
    legend = ("".join(
        f'<li><b style="color:{colours[i % len(colours)]}">Leg {i+1} → obstacle {s["obstacle_id"]}</b> '
        f'(cost {s["cost"]:.2f}): <code>{html.escape(" ".join(s["hardware_commands"]))}</code></li>'
        for i, s in enumerate(result["segments"]))
        + "".join(f'<li style="color:#888">Skipped {s["obstacle_id"]}: {html.escape(s["reason"])}</li>'
                  for s in result["skipped"]))
    page = f"""<!doctype html><meta charset="utf-8"><title>Planned path</title>
<body style="font-family:system-ui,sans-serif;margin:16px;display:flex;gap:24px;flex-wrap:wrap;align-items:flex-start">
<div style="width:640px">{svg}<p style="font-size:12px;color:#555">Black squares: obstacles (grey = skipped). Orange edge: image face.
Green box: start. Dashed boxes: photo poses (numbered in visit order). Lines: robot-centre path.</p></div>
<div style="flex:1;min-width:320px;max-width:600px"><h2>Visited {len(result['order'])}/{len(obstacles)} · order {result['order']} · cost {result['total_cost']}</h2>
<p>Clearance {config.MIN_CLEARANCE_CM} cm + {config.POSITION_MARGIN_CM} margin ·
AC20 {'±'+format(config.ALIGN_MAX_TRAVEL_CM,'g')+' cm' if config.ALIGN_ENABLED else 'off'} · planned in {result['planning_ms']/1000:.2f} s</p>
<ol style="list-style:none;padding:0;line-height:1.6">{legend}</ol></div></body>"""
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "path_view.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"\nDrawing written to {out}")
    if not args.no_open:
        webbrowser.open("file://" + out)


if __name__ == "__main__":
    main()
