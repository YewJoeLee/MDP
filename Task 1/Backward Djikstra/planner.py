"""v5 planner. Centimetre planner using existing STM commands only.
Visit order: backward-Dijkstra cost table per target + exact search over all orders.
Nominal turn trajectory is a quarter ellipse with a rotating rectangular body.
Conservative boxes cover the complete modelled sweep, including between samples.
The trajectory model must be physically validated: endpoints alone cannot do so.
"""
import heapq
import math
import re
import time
import config

HEADINGS = ("N", "E", "S", "W")
COMMAND_PATTERN = re.compile(r"(?:FW[1-9][0-9]*|BW[1-9][0-9]*|LT90|RT90|XL90|XR90|AC[1-9][0-9]*|RA|SNAP[1-9][0-9]*)\Z")


def validate_config():
    positive = (config.GRID, config.CELL_CM, config.ROBOT_WIDTH_CM,
                config.ROBOT_LENGTH_CM, config.MAX_STRAIGHT_COMMAND_CM,
                config.PLAN_TIME_LIMIT_S, config.MAX_SEARCH_EXPANSIONS)
    if any(not math.isfinite(x) or x <= 0 for x in positive):
        raise ValueError("Arena, robot dimensions and search limits must be positive")
    if any(not math.isfinite(x) or x < 0 for x in
           (config.MIN_CLEARANCE_CM, config.POSITION_MARGIN_CM, config.BOUNDARY_MARGIN_CM)):
        raise ValueError("Clearance and margins must be non-negative")
    if config.STRAIGHT_STEP_CM != 1 or not 0 < config.TURN_SAMPLE_DEGREES <= 10:
        raise ValueError("Use 1 cm straight steps and turn samples in (0,10] degrees")
    # Sanity ranges only, so tuning config.py never needs a planner edit.
    target = config.ALIGN_TARGET_CM
    if isinstance(target, bool) or not isinstance(target, (int, float)) or target != int(target) or not 5 <= target <= 100:
        raise ValueError(f"config.ALIGN_TARGET_CM = {target!r}: must be a whole number of cm from 5 to 100 (sent as AC<cm>)")
    if not math.isfinite(config.ALIGN_MAX_TRAVEL_CM) or not 0 < config.ALIGN_MAX_TRAVEL_CM <= 60:
        raise ValueError("ALIGN_MAX_TRAVEL_CM must be in (0, 60] and match the STM firmware cap")
    if not set(config.ALL_MOVES) <= {"FW", "BW", "LT90", "RT90", "XL90", "XR90"}:
        raise ValueError("Unsupported movement")
    if any(v <= 0 for v in (*config.STRAIGHT_COST_PER_CM.values(), *config.TURN_COST.values())):
        raise ValueError("Costs must be positive")
    if type(config.GRID) is not int or type(config.MAX_STRAIGHT_COMMAND_CM) is not int:
        raise ValueError("Grid size and maximum straight command must be integers")
    if not config.ALL_MOVES:
        raise ValueError("config.ALL_MOVES must not be empty")
    tol = config.VIEW_POSITION_TOLERANCE_CM
    if not math.isfinite(tol) or not 0 <= tol <= config.CELL_CM/2:
        raise ValueError(f"config.VIEW_POSITION_TOLERANCE_CM = {tol!r}: must be 0 to {config.CELL_CM/2:g} cm (half a cell)")
    for cmd, (a, r, q) in config.TURN_MODELS.items():
        if cmd not in ("LT90", "RT90", "XL90", "XR90") or q not in (-1, 1) or any(type(v) is not int or v == 0 for v in (a, r)):
            raise ValueError("Turn models require integer nonzero cm offsets and quarter-turn headings")
    if not config.VIEW_SENSOR_DISTANCES_CM or any(not math.isfinite(d) or d <= 0 for d in config.VIEW_SENSOR_DISTANCES_CM):
        raise ValueError("Viewing distances must be positive and finite")
    if hasattr(config, "VIEW_LATERAL_OFFSETS_CM"):
        if not isinstance(config.VIEW_LATERAL_OFFSETS_CM, (list, tuple)) or any(not math.isfinite(x) for x in config.VIEW_LATERAL_OFFSETS_CM):
            raise ValueError("VIEW_LATERAL_OFFSETS_CM must be a list/tuple of numbers")
    for name in ("SNAP_FAR_MAX_SENSOR_CM", "SNAP_NEAR_MIN_SENSOR_CM", "SNAP_RETRY_MIN_MOVE_CM"):
        value = getattr(config, name, None)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"config.{name} = {value!r}: must be a non-negative number of cm")
    if config.SNAP_NEAR_MIN_SENSOR_CM > config.SNAP_FAR_MAX_SENSOR_CM:
        raise ValueError("config.SNAP_NEAR_MIN_SENSOR_CM must not exceed SNAP_FAR_MAX_SENSOR_CM")
    if getattr(config, "VIEW_PREFERRED_SENSOR_CM", None) is not None:
        for name in ("VIEW_PREFERRED_SENSOR_CM", "VIEW_FURTHER_COST_PER_CM", "VIEW_NEARER_COST_PER_CM"):
            value = getattr(config, name, None)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"config.{name} = {value!r}: must be a non-negative number")
    if hasattr(config, "LATERAL_COST_PER_CM"):
        if not math.isfinite(config.LATERAL_COST_PER_CM) or config.LATERAL_COST_PER_CM < 0:
            raise ValueError("LATERAL_COST_PER_CM must be a non-negative finite number")


def normalize_obstacles(raw):
    if not isinstance(raw, (list, tuple)):
        raise ValueError("Obstacles must be a list of (id,x,y,face)")
    result, ids, cells = [], set(), set()
    for row in raw:
        if not isinstance(row, (list, tuple)) or len(row) != 4:
            raise ValueError("Each obstacle must contain id,x,y,face")
        oid, x, y, face = row
        if any(type(v) is not int for v in (oid, x, y)) or oid <= 0:
            raise ValueError("ID must be positive; coordinates must be integer cell indices")
        face = str(face).upper()
        if face not in HEADINGS or not (0 <= x < config.GRID and 0 <= y < config.GRID):
            raise ValueError(f"Invalid obstacle {oid}: coordinates or image face")
        if oid in ids or (x, y) in cells:
            raise ValueError("Duplicate obstacle ID or overlapping obstacle cells")
        ids.add(oid); cells.add((x, y)); result.append((oid, x, y, face))
    return result


def normalize_pose(pose):
    if not isinstance(pose, (tuple, list)) or len(pose) != 3:
        raise ValueError("Pose must be (centre_x_cm,centre_y_cm,heading)")
    x, y, heading = pose
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (x, y)):
        raise ValueError("Pose coordinates must be finite numbers")
    heading = str(heading).upper()
    if heading not in HEADINGS:
        raise ValueError("Pose heading must be N/E/S/W")
    return (float(x), float(y), heading)


def obstacle_rect(o):
    s = config.CELL_CM
    return (o[1]*s, o[2]*s, (o[1]+1)*s, (o[2]+1)*s)


def rect_gap(a, b):
    return math.hypot(max(a[0]-b[2], b[0]-a[2], 0),
                      max(a[1]-b[3], b[1]-a[3], 0))


def body_box(x, y, angle):
    hw, hl = config.ROBOT_WIDTH_CM/2, config.ROBOT_LENGTH_CM/2
    dx = abs(math.cos(angle))*hw + abs(math.sin(angle))*hl
    dy = abs(math.sin(angle))*hw + abs(math.cos(angle))*hl
    return (x-dx, y-dy, x+dx, y+dy)


def apply(pose, move):
    x, y, h = pose
    fx, fy = config.DIRECTION_STEP[h]
    rx, ry = fy, -fx
    if move in ("FW", "BW"):
        d = config.STRAIGHT_STEP_CM * (1 if move == "FW" else -1)
        return (x+fx*d, y+fy*d, h)
    a, r, q = config.TURN_MODELS[move]
    return (x+fx*a+rx*r, y+fy*a+ry*r, HEADINGS[(HEADINGS.index(h)+q)%4])


class Geometry:
    def __init__(self, obstacles):
        self.rects = [obstacle_rect(o) for o in obstacles]
        self.rect_ids = [o[0] for o in obstacles]
        self.extent = config.GRID*config.CELL_CM
        self.templates, self.edges = {}, {}
        for h in HEADINGS:
            angle = HEADINGS.index(h)*math.pi/2
            fx, fy = config.DIRECTION_STEP[h]
            rx, ry = fy, -fx
            for move in config.ALL_MOVES:
                if move in ("FW", "BW"):
                    end = apply((0, 0, h), move)
                    b0, b1 = body_box(0, 0, angle), body_box(end[0], end[1], angle)
                    boxes = [(min(b0[0], b1[0]), min(b0[1], b1[1]),
                              max(b0[2], b1[2]), max(b0[3], b1[3]))]
                    pad = 0
                else:
                    a, r, q = config.TURN_MODELS[move]
                    n = math.ceil(90/config.TURN_SAMPLE_DEGREES)
                    # Body-point velocity w.r.t angle <= max ellipse radius
                    # plus half body diagonal. Nearest sample <= half interval.
                    pad = (max(abs(a), abs(r)) + math.hypot(config.ROBOT_WIDTH_CM/2,
                            config.ROBOT_LENGTH_CM/2))*math.pi/(4*n)
                    boxes = []
                    for i in range(n+1):
                        t = i*math.pi/(2*n)
                        ahead, right = a*math.sin(t), r*(1-math.cos(t))
                        boxes.append(body_box(fx*ahead+rx*right, fy*ahead+ry*right, angle+q*t))
                boxes = [(b[0]-pad, b[1]-pad, b[2]+pad, b[3]+pad) for b in boxes]
                union = (min(b[0] for b in boxes), min(b[1] for b in boxes),
                         max(b[2] for b in boxes), max(b[3] for b in boxes))
                self.templates[h, move] = (boxes, union)

    def box_valid(self, box):
        boundary = config.BOUNDARY_MARGIN_CM + config.POSITION_MARGIN_CM
        if box[0] < boundary-1e-8 or box[1] < boundary-1e-8 or box[2] > self.extent-boundary+1e-8 or box[3] > self.extent-boundary+1e-8:
            return False
        gap = config.MIN_CLEARANCE_CM + config.POSITION_MARGIN_CM
        return all(rect_gap(box, obstacle) >= gap-1e-8 for obstacle in self.rects)

    def box_problem(self, box):
        """Why box_valid(box) is False, in words (None when it is valid)."""
        boundary = config.BOUNDARY_MARGIN_CM + config.POSITION_MARGIN_CM
        walls = [("left (x=0)", box[0] - boundary), ("bottom (y=0)", box[1] - boundary),
                 (f"right (x={self.extent:g})", self.extent - boundary - box[2]),
                 (f"top (y={self.extent:g})", self.extent - boundary - box[3])]
        for name, slack in walls:
            if slack < -1e-8:
                d = slack + boundary
                where = (f"robot body would go {-d:.1f} cm past the wall" if d < 0
                         else f"robot body {d:.1f} cm from the wall, needs {boundary:g} cm "
                              "(BOUNDARY_MARGIN_CM + POSITION_MARGIN_CM)")
                return f"arena wall {name}: {where}"
        gap = config.MIN_CLEARANCE_CM + config.POSITION_MARGIN_CM
        worst = None
        for oid, rect in zip(self.rect_ids, self.rects):
            g = rect_gap(box, rect)
            if g < gap - 1e-8 and (worst is None or g < worst[1]):
                worst = (oid, g)
        if worst:
            return (f"obstacle {worst[0]}: robot body {worst[1]:.1f} cm away, needs {gap:g} cm "
                    f"(MIN_CLEARANCE_CM {config.MIN_CLEARANCE_CM:g} + POSITION_MARGIN_CM {config.POSITION_MARGIN_CM:g})")
        return None

    def straight_problem(self, pose, low, high):
        """Why straight_clear(pose, low, high) is False, in words (None if clear)."""
        fx, fy = config.DIRECTION_STEP[pose[2]]
        angle = HEADINGS.index(pose[2])*math.pi/2
        a = body_box(pose[0]+fx*low, pose[1]+fy*low, angle)
        b = body_box(pose[0]+fx*high, pose[1]+fy*high, angle)
        return self.box_problem((min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])))

    def pose_valid(self, pose):
        return self.box_valid(body_box(pose[0], pose[1], HEADINGS.index(pose[2])*math.pi/2))

    def straight_clear(self, pose, low, high):
        fx, fy = config.DIRECTION_STEP[pose[2]]
        angle = HEADINGS.index(pose[2])*math.pi/2
        a = body_box(pose[0]+fx*low, pose[1]+fy*low, angle)
        b = body_box(pose[0]+fx*high, pose[1]+fy*high, angle)
        return self.box_valid((min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])))

    def edge_valid(self, pose, move):
        key = (pose, move)
        if key in self.edges:
            return self.edges[key]
        boxes, union = self.templates[pose[2], move]
        x, y = pose[:2]
        shift = lambda b: (b[0]+x, b[1]+y, b[2]+x, b[3]+y)
        u = shift(union)
        boundary = config.BOUNDARY_MARGIN_CM + config.POSITION_MARGIN_CM
        valid = u[0] >= boundary-1e-8 and u[1] >= boundary-1e-8 and u[2] <= self.extent-boundary+1e-8 and u[3] <= self.extent-boundary+1e-8
        gap = config.MIN_CLEARANCE_CM + config.POSITION_MARGIN_CM
        if valid:
            nearby = [o for o in self.rects if rect_gap(u, o) < gap]
            valid = all(rect_gap(shift(b), o) >= gap-1e-8 for o in nearby for b in boxes)
        self.edges[key] = valid
        return valid


def view_distance_penalty(d):
    """Extra cost for photographing from sensor->face distance d (cm).

    VIEW_PREFERRED_SENSOR_CM (30) costs nothing. Further away costs
    VIEW_FURTHER_COST_PER_CM per cm (1.0, robot sits further BACK), closer
    costs VIEW_NEARER_COST_PER_CM per cm (2.0, robot sits further FORWARD),
    so 30 is tried first and further is preferred over closer.
    Added on top of the lateral-offset penalty, which is unchanged.
    """
    pref = getattr(config, "VIEW_PREFERRED_SENSOR_CM", None)
    if pref is None:
        return 0.0
    if d >= pref:
        return (d - pref) * config.VIEW_FURTHER_COST_PER_CM
    return (pref - d) * config.VIEW_NEARER_COST_PER_CM


def viewing_pose_candidates(obstacle, start=None):
    _, gx, gy, face = obstacle
    s = config.CELL_CM
    ox, oy = (gx+.5)*s, (gy+.5)*s
    fx, fy = config.DIRECTION_STEP[face]
    rx, ry = fy, -fx
    h = HEADINGS[(HEADINGS.index(face)+2)%4]
    ideals = [(ox+fx*(s/2+config.SENSOR_FORWARD_CM+d),
             oy+fy*(s/2+config.SENSOR_FORWARD_CM+d), h)
            for d in config.VIEW_SENSOR_DISTANCES_CM]
    if start is None:
        return ideals
    # Select command-reachable viewing positions near the ideal. Never round
    # the actual propagated robot position. Integer turn/straight offsets keep
    # the starting fractional x/y offsets invariant.
    lateral_offsets = getattr(config, "VIEW_LATERAL_OFFSETS_CM", (0,))
    lat_cost = getattr(config, "LATERAL_COST_PER_CM", 1.0)
    candidates = {}
    for d in config.VIEW_SENSOR_DISTANCES_CM:
        for lat in lateral_offsets:
            pix = ox + fx*(s/2 + config.SENSOR_FORWARD_CM + d) + rx*lat
            piy = oy + fy*(s/2 + config.SENSOR_FORWARD_CM + d) + ry*lat
            x = start[0] + round(pix - start[0])
            y = start[1] + round(piy - start[1])
            if max(abs(x - pix), abs(y - piy)) <= config.VIEW_POSITION_TOLERANCE_CM + 1e-8:
                pose = (x, y, h)
                pen = abs(lat) * lat_cost + view_distance_penalty(d)
                if pose not in candidates or pen < candidates[pose]:
                    candidates[pose] = pen
    return candidates


def view_target_info(obstacle, goal):
    """Where the robot is really aimed vs the planned obstacle centre.

    The ideal viewing pose sits exactly on the obstacle's centre line at the
    configured sensor distance. The planner may snap it by up to
    VIEW_POSITION_TOLERANCE_CM so the robot can reach it with whole-cm moves.
    "aimed_centre_cm" is the point straight ahead of the robot's actual goal
    pose at the planned distance, i.e. the obstacle centre it is lined up on.
    lateral_offset_cm > 0 means the robot sits to the RIGHT of the obstacle
    centre line (looking in the robot's heading), so the obstacle appears left.
    """
    _, gx, gy, _ = obstacle
    s = config.CELL_CM
    ox, oy = (gx+.5)*s, (gy+.5)*s
    ideals = viewing_pose_candidates(obstacle)
    ideal = min(ideals, key=lambda p: math.hypot(p[0]-goal[0], p[1]-goal[1]))
    hx, hy = config.DIRECTION_STEP[goal[2]]
    rx, ry = hy, -hx
    reach = math.hypot(ideal[0]-ox, ideal[1]-oy)   # centre-of-robot to obstacle centre
    ax, ay = goal[0]+hx*reach, goal[1]+hy*reach
    dx, dy = goal[0]-ideal[0], goal[1]-ideal[1]
    along = dx*hx+dy*hy                              # + means robot sits further forward (closer)
    planned_d = reach-s/2-config.SENSOR_FORWARD_CM
    lateral = dx*rx+dy*ry
    lat_pen = abs(round(lateral)) * getattr(config, "LATERAL_COST_PER_CM", 1.0)
    dist_pen = view_distance_penalty(round(planned_d))
    return {
        "obstacle_cell": [gx, gy],
        "obstacle_centre_cm": [round(ox, 2), round(oy, 2)],
        "ideal_view_pose": [round(ideal[0], 2), round(ideal[1], 2), ideal[2]],
        "goal_view_pose": [round(goal[0], 2), round(goal[1], 2), goal[2]],
        "aimed_centre_cm": [round(ax, 2), round(ay, 2)],
        "lateral_offset_cm": round(dx*rx+dy*ry, 2),
        "planned_sensor_to_face_cm": round(planned_d, 2),
        "expected_sensor_to_face_cm": round(planned_d-along, 2),
        "tolerance_cm": config.VIEW_POSITION_TOLERANCE_CM,
        # v5 viewing preference (cost added to the drive cost when choosing the pose)
        "preferred_sensor_to_face_cm": getattr(config, "VIEW_PREFERRED_SENSOR_CM", None),
        "distance_penalty": round(dist_pen, 2),
        "lateral_penalty": round(lat_pen, 2),
        "view_penalty": round(dist_pen + lat_pen, 2),
    }


def sensor_to_face_cm(obstacle, pose):
    """Straight-ahead distance from the ultrasonic/camera front to the image face."""
    _, gx, gy, face = obstacle
    s = config.CELL_CM
    nx, ny = config.DIRECTION_STEP[face]
    face_x, face_y = (gx+.5)*s + nx*s/2, (gy+.5)*s + ny*s/2       # face centre
    hx, hy = config.DIRECTION_STEP[pose[2]]
    sx, sy = pose[0] + hx*config.SENSOR_FORWARD_CM, pose[1] + hy*config.SENSOR_FORWARD_CM
    return (face_x-sx)*hx + (face_y-sy)*hy


def snap_retry_bounds(geometry, obstacle, goal):
    """v5: how far the robot may drive straight from its photo pose for retry photos.

    FAR  = BW (further from the face) up to config.SNAP_FAR_MAX_SENSOR_CM
    NEAR = FW (closer to the face)    down to config.SNAP_NEAR_MIN_SENSOR_CM
    Each is shortened until the whole straight sweep keeps MIN_CLEARANCE_CM
    from every obstacle (the photographed one included) and stays in the arena.
    The robot drives back to the photo pose afterwards, so the route is unchanged.
    """
    now = sensor_to_face_cm(obstacle, goal)
    hx, hy = config.DIRECTION_STEP[goal[2]]
    minimum = config.SNAP_RETRY_MIN_MOVE_CM

    def bound(name, command, sign, wanted, limit_name, wanted_sensor):
        # Try the full move first, then 1 cm shorter each time until the whole
        # straight sweep keeps clearance. Every try is recorded for the RPi log.
        full = int(math.floor(wanted + 1e-9))
        cm = full
        limit = limit_name
        tries, blocked_by = [], None
        while cm > 0:
            problem = geometry.straight_problem(goal, min(0, sign*cm), max(0, sign*cm))
            tries.append({"move_cm": cm, "sensor_to_face_cm": round(now - sign*cm, 2),
                          "ok": problem is None, "problem": problem})
            if problem is None:
                break
            blocked_by = problem
            limit = "clearance / arena wall"
            cm -= 1
        cm = max(cm, 0)
        info = {"name": name, "move_cm": cm, "wanted_move_cm": max(full, 0),
                "wanted_sensor_to_face_cm": wanted_sensor,
                "ideal_sensor_to_face_cm": round(now, 2),
                "short": cm < full, "blocked_by": blocked_by, "tries": tries,
                "limited_by": limit}
        if cm < minimum:
            info.update({"available": False,
                         "reason": f"no room: only {cm} cm possible ({limit}), need >= {minimum} cm"
                                   + (f"; blocked by {blocked_by}" if blocked_by else "")})
            return info
        pose = (goal[0] + hx*sign*cm, goal[1] + hy*sign*cm, goal[2])
        back = ("FW" if command == "BW" else "BW") + str(cm)
        info.update({"available": True, "command": f"{command}{cm}", "back_command": back,
                     "pose": [round(pose[0], 2), round(pose[1], 2), pose[2]],
                     "sensor_to_face_cm": round(now - sign*cm, 2)})
        return info

    return {
        "ideal_sensor_to_face_cm": round(now, 2),
        "far": bound("FAR", "BW", -1, config.SNAP_FAR_MAX_SENSOR_CM - now,
                     f"SNAP_FAR_MAX_SENSOR_CM = {config.SNAP_FAR_MAX_SENSOR_CM:g}",
                     config.SNAP_FAR_MAX_SENSOR_CM),
        "near": bound("NEAR", "FW", +1, now - config.SNAP_NEAR_MIN_SENSOR_CM,
                      f"SNAP_NEAR_MIN_SENSOR_CM = {config.SNAP_NEAR_MIN_SENSOR_CM:g}",
                      config.SNAP_NEAR_MIN_SENSOR_CM),
    }


# ---------------------------------------------------------------------------
# v7: CASE C angled backup photo (small RT/LT turn, undone by XR/XL)
# ---------------------------------------------------------------------------

def small_turn_model(name):
    """(forward cm, right cm, clockwise degrees) of a small turn such as RT20.

    Taken from config.SMALL_TURN_MODELS; a missing entry is derived from the
    90-degree model the same way the placeholders were (ellipse cut at the angle).
    """
    table = getattr(config, "SMALL_TURN_MODELS", {}) or {}
    if name in table:
        return tuple(float(v) for v in table[name])
    kind, deg = name[:2], int(name[2:])
    a, r, q = config.TURN_MODELS[kind + "90"]
    t = math.radians(deg)
    return (a*math.sin(t), r*(1-math.cos(t)), q*deg)


def _apply_small_turn(x, y, phi, model):
    """End pose of a small turn from (x, y, phi); phi = heading, radians clockwise from N."""
    ahead, right, deg = model
    fx, fy = math.sin(phi), math.cos(phi)
    rx, ry = math.cos(phi), -math.sin(phi)
    return x + ahead*fx + right*rx, y + ahead*fy + right*ry, phi + math.radians(deg)


def _small_turn_problem(geometry, x, y, phi, model):
    """First clearance problem along a small-turn arc (None if the whole arc is clear).

    The arc is sampled every degree with the same ellipse shape as the 90-degree
    templates: forward(t) ~ sin t, right(t) ~ (1 - cos t), heading + t.
    """
    ahead, right, deg = model
    T = math.radians(abs(deg))
    sign = 1 if deg >= 0 else -1
    n = max(4, int(abs(deg)))
    fx, fy = math.sin(phi), math.cos(phi)
    rx, ry = math.cos(phi), -math.sin(phi)
    for i in range(n + 1):
        t = T*i/n
        fa = math.sin(t)/math.sin(T) if T else 0.0
        fr = (1-math.cos(t))/(1-math.cos(T)) if T else 0.0
        px = x + ahead*fa*fx + right*fr*rx
        py = y + ahead*fa*fy + right*fr*ry
        problem = geometry.box_problem(body_box(px, py, phi + sign*t))
        if problem:
            return problem
    return None


def angled_options(geometry, obstacle, goal, far_bound):
    """
    v7 CASE C backup: where the robot can do the small angled turn.

    For each side (obstacle body seen on the RIGHT -> RT<deg>, LEFT -> LT<deg>):
      start spot : straight back from the IDEAL pose. The FAR spot is tried first
                   (that is where the robot already is), then 1 cm closer each try,
                   down to the IDEAL pose itself.
      turn       : RT/LT<deg> arc must keep clearance
      undo       : XR/XL<deg> arc (from the angled pose) must keep clearance
      back       : straight from where the undo is predicted to end, to the
                   IDEAL pose, must keep clearance
    Turn shapes are config.SMALL_TURN_MODELS (placeholders until measured).
    """
    deg = int(getattr(config, "ANGLE_TURN_DEG", 20))
    phi0 = HEADINGS.index(goal[2])*math.pi/2
    fx, fy = math.sin(phi0), math.cos(phi0)
    rx, ry = math.cos(phi0), -math.sin(phi0)
    max_back = int(far_bound.get("move_cm", 0)) if far_bound and far_bound.get("available") else 0
    out = {"turn_deg": deg, "max_back_cm": max_back}
    for side, turn, undo in (("right", f"RT{deg}", f"XR{deg}"), ("left", f"LT{deg}", f"XL{deg}")):
        mt, mu = small_turn_model(turn), small_turn_model(undo)
        tries, chosen = [], None
        for back in range(max_back, -1, -1):
            sx, sy = goal[0] - fx*back, goal[1] - fy*back
            problem, step = None, ""
            problem = _small_turn_problem(geometry, sx, sy, phi0, mt)
            step = turn
            if problem is None:
                ax, ay, aphi = _apply_small_turn(sx, sy, phi0, mt)
                problem = _small_turn_problem(geometry, ax, ay, aphi, mu)
                step = undo
                if problem is None:
                    ex, ey, ephi = _apply_small_turn(ax, ay, aphi, mu)
                    along = (ex-goal[0])*fx + (ey-goal[1])*fy
                    lateral = (ex-goal[0])*rx + (ey-goal[1])*ry
                    step = "drive back to IDEAL"
                    problem = geometry.straight_problem(goal, min(0.0, along), max(0.0, along))
            tries.append({"back_cm": back, "ok": problem is None, "step": step, "problem": problem})
            if problem is None:
                cam_x, cam_y = ax + math.sin(aphi)*config.SENSOR_FORWARD_CM, ay + math.cos(aphi)*config.SENSOR_FORWARD_CM
                chosen = {
                    "available": True, "side": side, "turn": turn, "undo": undo,
                    "start_back_cm": back,
                    "start_sensor_to_face_cm": round(sensor_to_face_cm(obstacle, goal) + back, 2),
                    "angled_pose": [round(ax, 2), round(ay, 2), round(math.degrees(aphi) % 360, 1)],
                    "return_along_cm": round(along, 2), "return_lateral_cm": round(lateral, 2),
                    "turn_model": list(mt), "undo_model": list(mu),
                }
                break
        if chosen is None:
            last = tries[-1] if tries else None
            why = (f"no spot from BW{max_back} (FAR) to IDEAL works; at IDEAL the {last['step']} is blocked by "
                   f"{last['problem']}") if last else "no FAR bound"
            chosen = {"available": False, "side": side, "turn": turn, "undo": undo, "reason": why}
        chosen["tries"] = tries
        out[side] = chosen
    return out


def move_cost(move):
    return config.STRAIGHT_COST_PER_CM[move] if move in ("FW", "BW") else config.TURN_COST[move]


def find_path(start, goals, geometry, deadline):
    goals = set(goals.keys() if isinstance(goals, dict) else goals)
    goals = {g for g in goals if all(abs((g[i]-start[i])-round(g[i]-start[i])) < 1e-7 for i in (0, 1))}
    if not goals:
        return None, None, "unreachable on the configured command lattice"
    ratio = min([move_cost(m)/(1 if m in ("FW", "BW") else sum(abs(v) for v in config.TURN_MODELS[m][:2])) for m in config.ALL_MOVES])
    def heuristic(p):
        return min(abs(p[0]-g[0])+abs(p[1]-g[1]) for g in goals)*ratio
    queue = [(heuristic(start), 0.0, start)]
    costs, previous = {start: 0.0}, {}
    expanded = 0
    while queue:
        if time.monotonic() >= deadline:
            return None, None, "planning time limit reached; reachability not established"
        _, cost, pose = heapq.heappop(queue)
        if cost > costs.get(pose, math.inf)+1e-9:
            continue
        if pose in goals:
            path, current = [], pose
            while current != start:
                current, m = previous[current]
                path.append(m)
            return list(reversed(path)), pose, None
        expanded += 1
        if expanded > config.MAX_SEARCH_EXPANSIONS:
            return None, None, "search limit reached; reachability not established"
        for m in config.ALL_MOVES:
            new = apply(pose, m)
            nc = cost+move_cost(m)
            if nc >= costs.get(new, math.inf)-1e-9:
                continue
            if not geometry.edge_valid(pose, m):
                continue
            costs[new], previous[new] = nc, (pose, m)
            heapq.heappush(queue, (nc+heuristic(new), nc, new))
    return None, None, "no route using the allowed movement models"


def map_hardware_commands(path):
    commands = []
    for m in path:
        if m in ("FW", "BW"):
            if commands and commands[-1].startswith(m) and int(commands[-1][2:]) < config.MAX_STRAIGHT_COMMAND_CM:
                commands[-1] = m+str(int(commands[-1][2:])+1)
            else:
                commands.append(m+"1")
        elif m in config.TURN_MODELS:
            commands.append(m)
        else:
            raise ValueError(f"Unknown primitive: {m}")
    return commands


def command_trace(start, commands):
    pose, trace = start, []
    for cmd in commands:
        before = pose
        if cmd[:2] in ("FW", "BW"):
            for _ in range(int(cmd[2:])):
                pose = apply(pose, cmd[:2])
        elif cmd in config.TURN_MODELS:
            pose = apply(pose, cmd)
        trace.append({"command": cmd, "before_cm": list(before), "after_cm": list(pose)})
    return trace


def predecessor(pose, move):
    """Inverse of apply(): the pose from which `move` lands exactly on `pose`."""
    x, y, h = pose
    if move in ("FW", "BW"):
        fx, fy = config.DIRECTION_STEP[h]
        d = config.STRAIGHT_STEP_CM * (1 if move == "FW" else -1)
        return (x-fx*d, y-fy*d, h)
    a, r, q = config.TURN_MODELS[move]
    h0 = HEADINGS[(HEADINGS.index(h)-q) % 4]
    fx, fy = config.DIRECTION_STEP[h0]
    rx, ry = fy, -fx
    return (x-(fx*a+rx*r), y-(fy*a+ry*r), h0)


def build_cost_table(goals, geometry, deadline):
    """Backward Dijkstra from every viewing pose of ONE target.

    Spreads outward from the goal poses, undoing each allowed move, until every
    reachable pose is settled. Result per pose: cheapest cost to the target and
    the first move to take from there (a signpost). Collision checks are the
    same forward sweep checks used everywhere else (edge_valid on the move).
    Returns (dist, nxt, reason); reason is None when the table is complete.
    """
    if isinstance(goals, dict):
        dist = dict(goals)
        queue = [(penalty, g) for g, penalty in goals.items()]
    else:
        dist = {g: 0.0 for g in goals}
        queue = [(0.0, g) for g in goals]
    nxt = {}
    heapq.heapify(queue)
    expanded = 0
    while queue:
        if time.monotonic() >= deadline:
            return dist, nxt, "planning time limit reached; reachability not established"
        cost, pose = heapq.heappop(queue)
        if cost > dist.get(pose, math.inf)+1e-9:
            continue
        expanded += 1
        if expanded > config.MAX_SEARCH_EXPANSIONS:
            return dist, nxt, "search limit reached; reachability not established"
        for m in config.ALL_MOVES:
            prev = predecessor(pose, m)
            nc = cost+move_cost(m)
            if nc >= dist.get(prev, math.inf)-1e-9:
                continue
            if apply(prev, m) != pose or not geometry.edge_valid(prev, m):
                continue
            dist[prev], nxt[prev] = nc, (m, pose)
            heapq.heappush(queue, (nc, prev))
    return dist, nxt, None


class OrderSolver:
    """Exact visit order: every order is considered.

    1. One cost table per target (build_cost_table).
    2. Each order is scored with table lookups only: start -> A, A's photo
       pose -> B, ... A leg ends at whichever of that target's viewing poses is
       cheapest from where the robot is.
    3. Orders that share the same visited set and the same current pose have
       identical futures, so that future is computed once and reused
       (memoisation). This gives the same answer as listing all n! orders.
    4. Preference: most targets visited first, then lowest total cost.
    """

    def __init__(self, targets, geometry, deadline):
        self.n = len(targets)
        self.tables = [build_cost_table(poses, geometry, deadline) for _, poses in targets]
        self._legs, self._memo = {}, {}

    def leg(self, index, pose):
        """(cost, end_pose, moves) from pose to target `index`, or None."""
        key = (index, pose)
        if key not in self._legs:
            dist, nxt, _ = self.tables[index]
            if pose not in dist:
                self._legs[key] = None
            else:
                moves, p = [], pose
                while p in nxt:
                    m, p = nxt[p]
                    moves.append(m)
                self._legs[key] = (dist[pose], p, moves)
        return self._legs[key]

    def _best(self, mask, pose):
        key = (mask, pose)
        if key in self._memo:
            return self._memo[key]
        best = (0, 0.0, ())
        for i in range(self.n):
            if mask & (1 << i):
                continue
            leg = self.leg(i, pose)
            if leg is None:
                continue
            count, cost, order = self._best(mask | (1 << i), leg[1])
            count, cost = count+1, cost+leg[0]
            if count > best[0] or (count == best[0] and cost < best[1]-1e-9):
                best = (count, cost, (i,)+order)
        self._memo[key] = best
        return best

    def solve(self, start):
        """Returns (order as target indices, {skipped index: reason})."""
        order = list(self._best(0, start)[2])
        stops, pose = [start], start
        for i in order:
            pose = self.leg(i, pose)[1]
            stops.append(pose)
        reasons = {}
        for i in range(self.n):
            if i in order:
                continue
            table_reason = self.tables[i][2]
            if table_reason:
                reasons[i] = table_reason
            elif all(self.leg(i, p) is None for p in stops):
                reasons[i] = "no route using the allowed movement models"
            else:
                reasons[i] = "reachable, but including it would mean fewer targets visited in total"
        return order, reasons


def plan(obstacles, start_pose=None):
    validate_config()
    obstacles = normalize_obstacles(obstacles)
    start = normalize_pose(config.START_POSE_CM if start_pose is None else start_pose)
    began = time.monotonic()
    deadline = began+config.PLAN_TIME_LIMIT_S
    geometry = Geometry(obstacles)
    if not geometry.pose_valid(start):
        raise ValueError("Starting robot footprint violates boundary or obstacle clearance")
    result = {"status": "SUCCESS", "start": list(start), "pose_units": "cm",
              "commands": [], "optimal_moves": [], "segments": [], "order": [],
              "skipped": [], "total_cost": 0.0, "planning_ms": 0.0,
              "route_policy": "exact visit order over backward cost tables: most targets, then lowest cost"}
    pending = []
    for o in obstacles:
        candidates = viewing_pose_candidates(o, start)
        if isinstance(candidates, dict):
            valid = {p: pen for p, pen in candidates.items() if geometry.pose_valid(p)}
            if not valid:
                reason = "no valid viewing pose: outside arena or insufficient body clearance"
            elif config.ALIGN_ENABLED:
                bound = config.ALIGN_MAX_TRAVEL_CM
                valid = {p: pen for p, pen in valid.items() if geometry.straight_clear(p, -bound, bound)}
                reason = f"no safe viewing pose for full AC{int(config.ALIGN_TARGET_CM)}/RA excursion (up to {bound:g} cm either direction)"
            else:
                reason = "no valid viewing pose"
        else:
            valid = [p for p in candidates if geometry.pose_valid(p)]
            if not valid:
                reason = "no valid viewing pose: outside arena or insufficient body clearance"
            elif config.ALIGN_ENABLED:
                bound = config.ALIGN_MAX_TRAVEL_CM
                valid = [p for p in valid if geometry.straight_clear(p, -bound, bound)]
                reason = f"no safe viewing pose for full AC{int(config.ALIGN_TARGET_CM)}/RA excursion (up to {bound:g} cm either direction)"
            else:
                reason = "no valid viewing pose"
        if valid:
            pending.append((o, valid))
        else:
            result["skipped"].append({"obstacle_id": o[0], "reason": reason})
    solver = OrderSolver(pending, geometry, deadline)
    order, reasons = solver.solve(start)
    current = start
    for index in order:
        obstacle, _ = pending[index]
        _, goal, path = solver.leg(index, current)
        commands = map_hardware_commands(path)
        trace = command_trace(current, commands)
        end = tuple(trace[-1]["after_cm"]) if trace else current
        if end != goal:
            raise AssertionError("Command conversion changed the planned endpoint")
        photo = (["AC"+str(int(config.ALIGN_TARGET_CM))] if config.ALIGN_ENABLED else []) + ["SNAP"+str(obstacle[0])] + (["RA"] if config.ALIGN_ENABLED else [])
        commands += photo
        if not all(COMMAND_PATTERN.fullmatch(c) for c in commands):
            raise AssertionError("Generated unsupported command")
        cost = sum(move_cost(m) for m in path)
        target = view_target_info(obstacle, goal)
        target["face"] = obstacle[3]
        target["retry"] = snap_retry_bounds(geometry, obstacle, goal)   # v5 FAR / NEAR photos
        target["angled"] = angled_options(geometry, obstacle, goal, target["retry"]["far"])   # v7 CASE C
        result["segments"].append({"obstacle_id": obstacle[0], "start_pose": list(current),
            "goal_pose": list(goal), "hardware_commands": commands, "moves": path,
            "trace": trace, "photo_sequence": photo, "cost": cost,
            "target": target})
        result["commands"].extend(commands)
        result["optimal_moves"].extend(path+["SNAP"+str(obstacle[0])])
        result["order"].append(obstacle[0]); result["total_cost"] += cost
        current = goal
    result["skipped"].extend({"obstacle_id": pending[i][0][0], "reason": reasons[i]}
                             for i in range(len(pending)) if i not in order)
    result["planning_ms"] = round((time.monotonic()-began)*1000, 3)
    result["total_cost"] = round(result["total_cost"], 3)
    return result
