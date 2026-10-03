"""Centimetre planner using existing STM commands only.
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


def viewing_pose_candidates(obstacle, start=None):
    _, gx, gy, face = obstacle
    s = config.CELL_CM
    ox, oy = (gx+.5)*s, (gy+.5)*s
    fx, fy = config.DIRECTION_STEP[face]
    h = HEADINGS[(HEADINGS.index(face)+2)%4]
    ideals = [(ox+fx*(s/2+config.SENSOR_FORWARD_CM+d),
             oy+fy*(s/2+config.SENSOR_FORWARD_CM+d), h)
            for d in config.VIEW_SENSOR_DISTANCES_CM]
    if start is None:
        return ideals
    # Select command-reachable viewing positions near the ideal. Never round
    # the actual propagated robot position. Integer turn/straight offsets keep
    # the starting fractional x/y offsets invariant.
    candidates = []
    for p in ideals:
        x = start[0]+round(p[0]-start[0])
        y = start[1]+round(p[1]-start[1])
        if max(abs(x-p[0]), abs(y-p[1])) <= config.VIEW_POSITION_TOLERANCE_CM+1e-8:
            candidates.append((x, y, h))
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
    }


def move_cost(move):
    return config.STRAIGHT_COST_PER_CM[move] if move in ("FW", "BW") else config.TURN_COST[move]


def find_path(start, goals, geometry, deadline):
    goals = set(goals)
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
    dist = {g: 0.0 for g in goals}
    nxt = {}
    queue = [(0.0, g) for g in goals]
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
                self._legs[key] = (sum(move_cost(m) for m in moves), p, moves)
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


def plan(obstacles, start_pose=None, target_ids=None, preferred_order=None):
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
        if target_ids is not None and o[0] not in target_ids:
            continue
        candidates = viewing_pose_candidates(o, start)
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
    if preferred_order is not None:
        by_id = {o[0]: i for i, (o, poses) in enumerate(pending)}
        proposed, pose = [], start
        for oid in preferred_order:
            if oid not in by_id:
                break
            i = by_id[oid]
            leg = solver.leg(i, pose)
            if leg is None:
                break
            proposed.append(i)
            pose = leg[1]
        if len(proposed) == len(preferred_order) == len(pending):
            order = proposed

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
        result["segments"].append({"obstacle_id": obstacle[0], "start_pose": list(current),
            "goal_pose": list(goal), "hardware_commands": commands, "moves": path,
            "trace": trace, "photo_sequence": photo, "cost": cost,
            "target": view_target_info(obstacle, goal),
            "backup": backup_pose(obstacle, goal, geometry)})
        result["commands"].extend(commands)
        result["optimal_moves"].extend(path+["SNAP"+str(obstacle[0])])
        result["order"].append(obstacle[0]); result["total_cost"] += cost
        current = goal
    result["skipped"].extend({"obstacle_id": pending[i][0][0], "reason": reasons[i]}
                             for i in range(len(pending)) if i not in order)
    result["planning_ms"] = round((time.monotonic()-began)*1000, 3)
    result["total_cost"] = round(result["total_cost"], 3)
    return result


def backup_pose(obstacle, goal, geometry):
    """One straight reverse within the ORIGINAL calibrated viewing-distance range.

    Uses all physical obstacles, not just targets remaining to photograph.
    No new camera-range claim is introduced by this fallback.
    """
    import recovery_config as recovery
    face_x, face_y = config.DIRECTION_STEP[obstacle[3]]
    ox = (obstacle[1] + .5) * config.CELL_CM
    oy = (obstacle[2] + .5) * config.CELL_CM
    sensor_distance = ((goal[0]-ox)*face_x + (goal[1]-oy)*face_y
                       - config.CELL_CM/2 - config.SENSOR_FORWARD_CM)
    for distance in range(recovery.MAX_BACKUP_CM, recovery.MIN_BACKUP_CM-1, -1):
        if not min(config.VIEW_SENSOR_DISTANCES_CM) <= sensor_distance+distance <= max(config.VIEW_SENSOR_DISTANCES_CM):
            continue
        if not geometry.straight_clear(goal, -distance, 0):
            continue
        fx, fy = config.DIRECTION_STEP[goal[2]]
        end = (goal[0]-fx*distance, goal[1]-fy*distance, goal[2])
        return {"command": "BW"+str(distance), "pose": list(end),
                "sensor_distance_cm": sensor_distance+distance}
    return None
