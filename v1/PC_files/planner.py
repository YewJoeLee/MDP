"""
apply()
→ works out where a move sends the robot

viewing_pose()
→ finds a valid position from which we can see an obstacle

is_valid()
→ checks whether that resulting pose is safe

turn_offset() / turn_sweep_cells() / turn_is_clear()
→ describe a turn's geometry and check the whole arc is safe, not just the end

neighbours()
→ returns all legal next moves and their costs

neighbours() → gives legal next moves
heuristic()  → estimates remaining cost
find_path()  → searches for a route
reconstruct()→ rebuilds the move list

"""

import heapq
import math
import time
from itertools import permutations

import config

# ---------------------------------------------------------------------
# COMPASS HELPERS (built from config.DIRECTION_STEP)
# ---------------------------------------------------------------------

# Which way is "right" / "left" / "behind" for each facing direction.
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}
OPPOSITE_OF = {"N": "S", "S": "N", "E": "W", "W": "E"}


# After each turn, which way is the robot facing? (a lookup into the tables above)
TURN_RESULT_FACING = {
    "FR": RIGHT_OF,
    "FL": LEFT_OF,
    "BR": LEFT_OF,     # reversing to the right swings the nose LEFT
    "BL": RIGHT_OF,    # reversing to the left swings the nose RIGHT
}


def is_turn(move):
    """True if this move is one of the four 90 degree turns."""
    return move in TURN_RESULT_FACING


def turn_offset(move):
    """
    How far one turn shifts the robot, in the ROBOT'S OWN frame.

    Returns (ahead, right):
        ahead > 0 means forward, ahead < 0 means backward
        right > 0 means to its right, right < 0 means to its left

    Every turn has its own two numbers in config.py because every turn has
    a different turning radius on the real car.
    """

    if move == "FR":
        return (config.FORWARD_RIGHT_TURN_AHEAD, config.FORWARD_RIGHT_TURN_SIDE)

    if move == "FL":
        return (config.FORWARD_LEFT_TURN_AHEAD, -config.FORWARD_LEFT_TURN_SIDE)

    if move == "BR":
        return (-config.REVERSE_RIGHT_TURN_BACK, config.REVERSE_RIGHT_TURN_SIDE)

    if move == "BL":
        return (-config.REVERSE_LEFT_TURN_BACK, -config.REVERSE_LEFT_TURN_SIDE)

    raise ValueError("Not a turn: " + str(move))


def apply(robot, move):
    """
    Work out where a move sends the robot.

    Every distance comes from config.py:
        FW / BW  -> config.STRAIGHT_CELLS
        turns    -> the per-turn values read by turn_offset()

    Distances are applied along the direction the robot is facing
    (forward / right), so no per-direction if-chains are needed.
    """
    x = robot[0]
    y = robot[1]
    facing = robot[2]

    forward_x, forward_y = config.DIRECTION_STEP[facing]
    right_x, right_y = config.DIRECTION_STEP[RIGHT_OF[facing]]

    straight = config.STRAIGHT_CELLS


    # Forward

    if move == "FW":
        return (x + forward_x * straight,
                y + forward_y * straight,
                facing)


    # Backward

    if move == "BW":
        return (x - forward_x * straight,
                y - forward_y * straight,
                facing)


    # Turns: FR, FL, BR, BL

    if is_turn(move):
        ahead, right = turn_offset(move)

        return (x + forward_x * ahead + right_x * right,
                y + forward_y * ahead + right_y * right,
                TURN_RESULT_FACING[move][facing])


    return robot


def turn_sweep_cells(pose, move):
    """
    The cells the robot's centre passes through DURING a turn.

    A turn is not a teleport. If we only checked where it starts and ends,
    the planner would happily sweep the car through an obstacle corner or
    over the arena wall halfway round. So we sample points along the arc.

    The arc is a quarter ellipse whose two radii are the turn's own
    "ahead/back" and "side" distances from config.py. In the robot's frame,
    at fraction t of the way round (angle = t * 90 degrees):

        ahead_now = ahead * sin(angle)
        right_now = right * (1 - cos(angle))

    At t = 0 that is the start pose; at t = 1 it is exactly what apply()
    returns. Sample points come from config.TURN_SWEEP_FRACTIONS.
    """
    x = pose[0]
    y = pose[1]
    facing = pose[2]

    forward_x, forward_y = config.DIRECTION_STEP[facing]
    right_x, right_y = config.DIRECTION_STEP[RIGHT_OF[facing]]

    ahead, right = turn_offset(move)

    cells = []

    for fraction in config.TURN_SWEEP_FRACTIONS:

        angle = fraction * (math.pi / 2.0)

        ahead_now = ahead * math.sin(angle)
        right_now = right * (1.0 - math.cos(angle))

        cell_x = round(x + forward_x * ahead_now + right_x * right_now)
        cell_y = round(y + forward_y * ahead_now + right_y * right_now)

        cells.append((cell_x, cell_y))

    return cells


def turn_is_clear(pose, move, obstacles):
    """True if every sampled cell along the turn's arc is a valid position."""
    for cell_x, cell_y in turn_sweep_cells(pose, move):

        if not is_valid((cell_x, cell_y, pose[2]), obstacles):
            return False

    return True


def viewing_pose(obstacle):
    """
    Nominal pose from which we can photograph an obstacle:
    config.VIEW_DISTANCE cells in front of the image face, looking back at it.
    """
    obstacle_x = obstacle[1]
    obstacle_y = obstacle[2]
    image_face = obstacle[3]

    step_x, step_y = config.DIRECTION_STEP[image_face]

    stand_x = obstacle_x + step_x * config.VIEW_DISTANCE
    stand_y = obstacle_y + step_y * config.VIEW_DISTANCE

    return (stand_x, stand_y, OPPOSITE_OF[image_face])


def is_valid(pose, obstacles):

    x = pose[0]
    y = pose[1]

    # Robot centre must keep ROBOT_REACH cells of body inside the arena
    smallest = config.ROBOT_REACH
    largest = config.GRID - 1 - config.ROBOT_REACH

    # Robot body + safety gap around every obstacle
    banned = config.ROBOT_REACH + config.SAFETY_CELLS


    # Check robot stays inside the arena

    if x < smallest or x > largest:
        return False

    if y < smallest or y > largest:
        return False


    # Check robot does not get too close to an obstacle

    for obstacle in obstacles:

        obstacle_x = obstacle[1]
        obstacle_y = obstacle[2]

        if abs(x - obstacle_x) <= banned:

            if abs(y - obstacle_y) <= banned:

                return False


    return True

'''
# For testing - viewing_pose and is_valid 
# Module-level: actually build and print the results
VIEWING_POSES = [viewing_pose(obstacle) for obstacle in OBSTACLES]
print(VIEWING_POSES)

for pose in VIEWING_POSES:
    print(pose, is_valid(pose, OBSTACLES))
'''


def neighbours(pose, obstacles):

    valid_moves = []


    for move in config.ALL_MOVES:

        new_pose = apply(pose, move)

        if is_valid(new_pose, obstacles):

            # A turn must be safe along its whole arc, not just at the end
            if is_turn(move) and not turn_is_clear(pose, move, obstacles):
                continue

            cost = config.MOVE_COST[move]

            valid_moves.append(
                (new_pose, move, cost)
            )


    return valid_moves


'''
#Testing
print(neighbours((5, 8, "N"), OBSTACLES))
'''

def heuristic(current, goal):

    current_x = current[0]
    current_y = current[1]

    goal_x = goal[0]
    goal_y = goal[1]

    distance = abs(current_x - goal_x) + abs(current_y - goal_y)

    return distance * config.HEURISTIC_WEIGHT


def reconstruct(came_from, current):

    moves = []

    while current in came_from:

        previous_pose = came_from[current][0]
        move = came_from[current][1]

        moves.append(move)

        current = previous_pose

    moves.reverse()

    return moves


def find_path(start_pose, goal_pose, obstacles):

    open_set = []

    heapq.heappush(open_set, (0, start_pose))

    came_from = {}

    g_score = {}
    g_score[start_pose] = 0

    while open_set:

        current_item = heapq.heappop(open_set)

        current = current_item[1]


        # We reached the destination

        if current == goal_pose:

            return reconstruct(came_from, current)


        # Look at all legal next moves

        next_moves = neighbours(current, obstacles)

        for item in next_moves:

            new_pose = item[0]
            move = item[1]
            cost = item[2]

            new_cost = g_score[current] + cost


            # First time reaching this pose
            # OR we found a cheaper way to reach it

            if new_pose not in g_score or new_cost < g_score[new_pose]:

                g_score[new_pose] = new_cost

                came_from[new_pose] = (current, move)

                estimated_cost = new_cost + heuristic(new_pose, goal_pose)

                heapq.heappush(
                    open_set,
                    (estimated_cost, new_pose)
                )


    # No route found

    return None


"""
#Testing 
start = (1, 1, "N")
goal = (1, 10, "E")

path = find_path(start, goal, [])

print(path)
"""

#For B2 -> Choosing based on move count ( greedy-optimised ), rather than using cost function 
def choose_visit_order(start_pose, viewing_poses, obstacles):

    current = start_pose

    unvisited = list(viewing_poses)

    order = []

    while len(unvisited) > 0:

        best_pose = None
        best_length = None


        for pose in unvisited:

            path = find_path(current, pose, obstacles)

            if path is not None:

                path_length = len(path)


                if best_length is None or path_length < best_length:

                    best_length = path_length

                    best_pose = pose


        order.append(best_pose)

        unvisited.remove(best_pose)

        current = best_pose

    return order


def build_full_path(start_pose, order, obstacles):

    current = start_pose

    full_path = []


    for goal in order:

        path = find_path(current, goal, obstacles)


        for move in path:

            full_path.append(move)


        full_path.append("SNAP")

        current = goal


    return full_path


# For B3 -> A star optimised path cost
def path_cost(path):

    total_cost = 0

    for move in path:

        total_cost = total_cost + config.MOVE_COST[move]

    return total_cost


# =====================================================================
# Section added for the RPi <-> Algo integration test.
# Everything above this line is unchanged.
#
# This gives algo_server.py the single entry point it calls:
#     planner.plan(obstacles, start_pose=config.START_POSE)
# =====================================================================

# (time, config, permutations are imported at the top of the file)

# key = (start_pose, goal_pose, obstacles) -> (path_tuple_or_None, cost_or_None)
PATH_CACHE = {}
CACHE_HITS = 0
CACHE_MISSES = 0


def reset_cache():
    """Clear the A* cache. Call between runs with different obstacle maps."""
    global PATH_CACHE, CACHE_HITS, CACHE_MISSES
    PATH_CACHE = {}
    CACHE_HITS = 0
    CACHE_MISSES = 0


def viewing_pose_candidates(obstacle):
    """
    All acceptable poses for photographing one obstacle, best first.

    viewing_pose() returns only the single nominal pose. If that one
    happens to fall outside the arena or inside another obstacle's
    clearance zone, the whole route used to fail. Here we degrade
    gracefully instead.
    """
    obstacle_x = obstacle[1]
    obstacle_y = obstacle[2]
    image_face = obstacle[3]

    # Unit step pointing out of the image face, and the axis running along
    # the face (x for N/S faces, y for E/W faces) used for the offsets.
    out_x, out_y = config.DIRECTION_STEP[image_face]
    side_x = abs(out_y)
    side_y = abs(out_x)

    candidates = []

    for distance in config.VIEW_DISTANCES:
        for offset in config.VIEW_OFFSETS:

            stand_x = obstacle_x + out_x * distance + side_x * offset
            stand_y = obstacle_y + out_y * distance + side_y * offset

            candidates.append((stand_x, stand_y, OPPOSITE_OF[image_face]))

    return candidates


def cached_leg(start_pose, goal_pose, obstacles):
    """
    find_path() + path_cost(), memoised.

    With 8 obstacles there are 40320 visit orders but only 72 distinct
    legs, so without this the permutation search runs A* ~300000 times.
    """
    global CACHE_HITS, CACHE_MISSES

    cache_key = (start_pose, goal_pose, tuple(obstacles))

    if cache_key in PATH_CACHE:
        CACHE_HITS += 1
        cached_path, cached_cost = PATH_CACHE[cache_key]

        if cached_path is None:
            return None, None

        return list(cached_path), cached_cost

    CACHE_MISSES += 1

    path = find_path(start_pose, goal_pose, obstacles)

    if path is None:
        PATH_CACHE[cache_key] = (None, None)
        return None, None

    cost = path_cost(path)
    PATH_CACHE[cache_key] = (tuple(path), cost)

    return path, cost


def choose_targets(start_pose, obstacles):
    """
    Pick one reachable viewing pose per obstacle.

    Returns:
        targets = [(obstacle_id, pose), ...]
        skipped = [{"obstacle_id": ..., "reason": ...}, ...]
    """
    targets = []
    skipped = []

    for obstacle in obstacles:
        obstacle_id = obstacle[0]
        chosen_pose = None

        for candidate in viewing_pose_candidates(obstacle):

            if not is_valid(candidate, obstacles):
                continue

            path, _cost = cached_leg(start_pose, candidate, obstacles)

            if path is None:
                continue

            chosen_pose = candidate
            break

        if chosen_pose is None:
            skipped.append({
                "obstacle_id": obstacle_id,
                "reason": "no reachable viewing pose inside the arena",
            })
        else:
            targets.append((obstacle_id, chosen_pose))

    return targets, skipped


def best_visit_order(start_pose, targets, obstacles):
    """
    Exhaustive search over visit orders, using the cached A* legs.

    Returns (best_order, best_cost) where best_order is a tuple of
    (obstacle_id, pose).
    """
    if len(targets) == 0:
        return (), 0

    best_order = None
    best_cost = None

    for order in permutations(targets):
        current_pose = start_pose
        total_cost = 0
        valid_order = True

        for _obstacle_id, goal_pose in order:
            path, cost = cached_leg(current_pose, goal_pose, obstacles)

            if path is None:
                valid_order = False
                break

            total_cost += cost
            current_pose = goal_pose

        if valid_order:
            if best_cost is None or total_cost < best_cost:
                best_cost = total_cost
                best_order = order

    return best_order, best_cost


def map_hardware_commands(path):
    """
    Compress A* primitives into the STM32 command format.

        FW FW FW -> FW30
        BW BW    -> BW20
        FL       -> LT90
        FR       -> RT90
        BL       -> XL90
        BR       -> XR90

    SNAP markers are passed through untouched.
    """
    turn_map = config.TURN_COMMANDS

    commands = []
    i = 0

    while i < len(path):
        move = path[i]

        if move in config.STRAIGHT_COMMANDS:
            count = 0

            while i < len(path) and path[i] == move:
                count += 1
                i += 1

            distance_cm = count * config.STRAIGHT_CELLS * config.CELL_CM
            commands.append(move + str(distance_cm))
            continue

        if move in turn_map:
            commands.append(turn_map[move])
            i += 1
            continue

        if move.startswith(config.SNAP_PREFIX):
            commands.append(move)
            i += 1
            continue

        raise ValueError("Unknown movement primitive: " + str(move))

    return commands


def plan(obstacles, start_pose=config.START_POSE):
    """
    Full planning pipeline, called by algo_server.py.

    obstacles  : [(id, x, y, image_face), ...]
    start_pose : (x, y, facing)

    Returns a dict that is safe to json.dumps().
    """
    obstacles = [tuple(obstacle) for obstacle in obstacles]
    start_pose = tuple(start_pose)

    reset_cache()
    start_time = time.perf_counter()

    targets, skipped = choose_targets(start_pose, obstacles)
    best_order, best_cost = best_visit_order(start_pose, targets, obstacles)

    if best_order is None:
        return {
            "status": "NO_ROUTE",
            "message": "No visit order can reach every chosen viewing pose.",
            "skipped": skipped,
            "segments": [],
            "order": [],
            "optimal_moves": [],
            "commands": [],
            "total_cost": 0,
        }

    current_pose = start_pose
    segments = []
    optimal_moves = []
    commands = []

    for obstacle_id, goal_pose in best_order:
        path, cost = cached_leg(current_pose, goal_pose, obstacles)
        snap = config.SNAP_PREFIX + str(obstacle_id)

        segment_commands = map_hardware_commands(path)

        segments.append({
            "obstacle_id": obstacle_id,
            "start_pose": list(current_pose),
            "goal_pose": list(goal_pose),
            "moves": path,
            "hardware_commands": segment_commands,
            "cost": cost,
        })

        optimal_moves.extend(path)
        optimal_moves.append(snap)

        commands.extend(segment_commands)
        commands.append(snap)

        current_pose = goal_pose

    planning_ms = (time.perf_counter() - start_time) * 1000

    return {
        "status": "SUCCESS",
        "start": list(start_pose),
        "order": [obstacle_id for obstacle_id, _pose in best_order],
        "total_cost": best_cost,
        "planning_ms": round(planning_ms, 3),
        "astar_runs": CACHE_MISSES,
        "cache_hits": CACHE_HITS,
        "segments": segments,
        "optimal_moves": optimal_moves,
        "commands": commands,
        "skipped": skipped,
    }


