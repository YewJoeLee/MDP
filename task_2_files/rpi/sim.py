import math, itertools, random, sys
import config as C
import task2 as T

W, Lr, a = C.ROBOT_WIDTH_CM, C.ROBOT_LENGTH_CM, C.SENSOR_FORWARD_CM
CPH, CPD, ARM = C.T2_CARPARK_HALF_CM, C.T2_CARPARK_DEPTH_CM, C.T2_CARPARK_ARM_CM   # carpark inside half-width, depth, arm

def ray_box(ox, oy, dx, dy, box):
    x0, x1, y0, y1 = box
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi: return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2: t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < max(tmin, 0): return None
    return tmin if tmin >= 0 else None

class World:
    # test the photo fallback: "ok" | "angled" (straight photos weak, 10-deg ones fine)
    # | "low" (every photo weak) | "none" (no arrow in any photo)
    photos = "ok"
    body_edge = True    # False = like the real Camera.snap(), which never fills body_edge_px

    def __init__(self, G1, G2, L, a1, a2, wall=50, ir_fwd=0.0, s=0.0, noise=0.0, beam=7.5):
        self.ir_fwd, self.s, self.noise, self.beam = ir_fwd, s, noise, beam
        self.a1, self.a2 = a1, a2
        self.boxes = {}
        self.boxes["arm_up"] = (-CPD - ARM, 0, CPH, CPH + ARM)
        self.boxes["arm_dn"] = (-CPD - ARM, 0, -CPH - ARM, -CPH)
        self.boxes["back"] = (-CPD - ARM, -CPD, -CPH - ARM, CPH + ARM)
        self.boxes["obs1"] = (G1, G1 + 10, -5, 5)
        x2 = G1 + 10 + G2
        self.x2, self.L = x2, L
        self.boxes["obs2"] = (x2, x2 + 10, -L / 2, L / 2)
        if wall:
            self.boxes["wall_up"] = (0, x2 + 200, L / 2 + wall, L / 2 + wall + 5)
            self.boxes["wall_dn"] = (0, x2 + 200, -L / 2 - wall - 5, -L / 2 - wall)
        self.p = T._Pose(0.0); self.p.x = -CPD + 12 + 6   # rear 6 cm off the back wall
        self.stack = []
        self.minclear = (1e9, None, None)
        self.cmds = []

    def check(self, q, cmd):
        for k, b in self.boxes.items():
            c = T._box_clear(q, b, W, Lr)
            if c < self.minclear[0]:
                self.minclear = (c, k, cmd)

    def send(self, cmd):
        self.cmds.append(cmd)
        p = self.p
        if cmd[:2] in ("FW", "BW"):
            n = int(cmd[2:]) * (1 if cmd[:2] == "FW" else -1)
            for _ in range(abs(n)):
                p.move(1 if n > 0 else -1); self.check(p, cmd)
        elif cmd[0] == "X" and cmd[1:] in ("R10", "L10"):
            q = self.stack.pop()
            p.x, p.y, p.h = q.x, q.y, q.h; self.check(p, cmd)
        else:
            if cmd in ("RT10", "LT10"):
                self.stack.append(p.copy())
            f, r, dg = T._turn_model(cmd, 1)
            q = p.copy(); q.move(f, r, dg)
            for i in range(1, 11):
                t = i / 10
                mid = p.copy(); mid.x = p.x + (q.x - p.x) * t; mid.y = p.y + (q.y - p.y) * t; mid.h = p.h + (q.h - p.h) * t
                self.check(mid, cmd)
            p.x, p.y, p.h = q.x, q.y, q.h

    def _advance(self, cm, label):
        """Move straight 1 step (overridden by show_path to record the trace)."""
        self.p.move(cm); self.check(self.p, label)

    def drive_edge(self, side, need_seen, max_cm, near_max=40.0, spike_n=2, guard=15.0):
        """Simulated STM IE command: FW up to max_cm, stop when the side IR goes near -> far."""
        label = f"IE{side[0]}{'N' if need_seen else 'A'}{int(max_cm)}"
        self.cmds.append(label)
        read = self.read_ir_left if side == "LEFT" else self.read_ir_right
        seen, run, first_far, nears, moved, end = False, 0, None, [], 0, "MAX"
        while moved < max_cm:
            r = read()
            if r is not None and r <= near_max:
                seen, run = True, 0; nears.append(r)
            elif seen or not need_seen:
                if run == 0: first_far = moved
                run += 1
                if run >= spike_n:
                    end = "EDGE" if seen else "NEVER"; break
            u = self.read_us()
            if u is not None and u < guard:
                end = "WALL"; break
            self._advance(1, label); moved += 1
        nears.sort()
        return {"end": end, "edge_cm": first_far if end in ("EDGE", "NEVER") else None,
                "near_cm": nears[len(nears) // 2] if nears else None, "moved_cm": float(moved), "cmd": label}

    def cast(self, ox, oy, ang):
        dx, dy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        best = None
        for b in self.boxes.values():
            t = ray_box(ox, oy, dx, dy, b)
            if t is not None and (best is None or t < best): best = t
        return best

    def read_us(self):
        p = self.p; hr = math.radians(p.h)
        sx, sy = p.x + a * math.cos(hr), p.y + a * math.sin(hr)
        ds = [self.cast(sx, sy, p.h + k * self.beam / 2) for k in (-2, -1, 0, 1, 2)]
        ds = [d for d in ds if d is not None]
        if not ds: return None
        d = min(ds)
        return d + random.uniform(-self.noise, self.noise) if d <= 400 else None

    def _ir(self, side):
        p = self.p; hr = math.radians(p.h); c, sn = math.cos(hr), math.sin(hr)
        off = W / 2 - self.s
        # right vector = (sn, -c)
        ix = p.x + self.ir_fwd * c + side * off * sn
        iy = p.y + self.ir_fwd * sn - side * off * c
        d = self.cast(ix, iy, p.h - 90 * side)
        if d is None or d > 80: return None
        return d + random.uniform(-self.noise, self.noise)

    def read_ir_right(self): return self._ir(1)
    def read_ir_left(self): return self._ir(-1)

    def snap(self):
        p = self.p; hr = math.radians(p.h)
        cx, cy = p.x + a * math.cos(hr), p.y + a * math.sin(hr)
        img_w = 640; fx = (img_w / 2) / math.tan(math.radians(31.1))
        cands = [(self.boxes["obs1"][0], self.a1, "obs1"), (self.x2, self.a2, "obs2")]
        best = None
        for fxp, lab, name in cands:
            if cx >= fxp: continue
            dist = math.hypot(fxp - cx, cy)
            brg = math.degrees(math.atan2(0 - cy, fxp - cx)) - p.h
            if abs(brg) > 28 or dist > 200: continue
            # occlusion: obs1 blocks obs2
            if name == "obs2":
                t = ray_box(cx, cy, fxp - cx, -cy, self.boxes["obs1"])
                if t is not None and t < 1: continue
            if best is None or dist < best[0]:
                best = (dist, lab, brg, name)
        if best is None or self.photos == "none": return None
        dist, lab, brg, name = best
        box_cx = img_w / 2 - fx * math.tan(math.radians(brg))
        res = {"label": lab, "conf": 0.9 if dist < 150 else 0.4, "box_cx": box_cx, "img_w": img_w}
        straight = abs((p.h + 180) % 360 - 180) < 3
        if self.photos == "low" or (self.photos == "angled" and straight):
            res["conf"] = 0.3                       # seen, but below RECOGNISED_CONF
        if name == "obs2" and self.body_edge:
            side = 1 if cy > 0 else -1
            pts = [(self.x2, side * self.L / 2), (self.x2 + 10, side * self.L / 2)]
            brs = [math.degrees(math.atan2(py - cy, px - cx)) - p.h for px, py in pts]
            e = max(brs) if side == 1 else min(brs)
            if abs(e) >= 31.1: res["body_edge_px"] = 0 if side == 1 else img_w
            else: res["body_edge_px"] = img_w / 2 - fx * math.tan(math.radians(e))
        return res

def run(G1, G2, L, a1, a2, wall=50, noise=0.0, verbose=False, ie=True):
    w = World(G1, G2, L, a1, a2, wall=wall, noise=noise)
    out = T.run_task2(w.send, w.read_us, w.read_ir_left, w.read_ir_right, w.snap,
                      cp_front_from_start=CPD - 18, drive_edge=w.drive_edge if ie else None)
    p = w.p
    inside = (p.x + 12 <= 0 and p.x - 12 >= -CPD and abs(p.y) + 9.5 <= CPH)
    return out, w, inside

if __name__ == "__main__":
    IE = "--no-ie" not in sys.argv          # --no-ie = old FW3 stepping (old firmware)
    random.seed(1)
    bad = 0; n = 0
    for a1, a2 in itertools.product(("LEFT", "RIGHT"), repeat=2):
        for G1 in (60, 100, 150):
            for G2 in (60, 100, 150):
                for L in (30, 40, 60, 90, 130):
                    for wall in (50, 0):
                        n += 1
                        try:
                            out, w, inside = run(G1, G2, L, a1, a2, wall=wall, ie=IE)
                        except Exception as e:
                            import traceback; traceback.print_exc()
                            print("EXC", a1, a2, G1, G2, L, wall); bad += 1; continue
                        mc = w.minclear
                        ok = out["status"] == "OK" and inside and mc[0] >= 3
                        if not ok:
                            bad += 1
                            if bad <= 40:
                                print(f"{a1[0]}{a2[0]} G1={G1} G2={G2} L={L} wall={wall}: {out['status']} inside={inside} "
                                      f"minclear={mc[0]:.1f}({mc[1]},{mc[2]}) pose=({w.p.x:.1f},{w.p.y:.1f},{w.p.h:.0f})")
    print(f"{n - bad}/{n} OK")
