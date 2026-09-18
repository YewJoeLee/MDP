"""
apply()
→ works out where a move sends the robot

viewing_pose()
→ finds a valid position from which we can see an obstacle

is_valid()
→ checks whether that resulting pose is safe

neighbours()
→ returns all legal next moves and their costs

neighbours() → gives legal next moves
heuristic()  → estimates remaining cost
find_path()  → searches for a route
reconstruct()→ rebuilds the move list

"""

import heapq
from itertools import permutations

def apply(robot, move):

    x = robot[0]
    y = robot[1]
    facing = robot[2]


    # Forward

    if move == "FW":

        if facing == "N":
            return (x, y + 1, "N")

        if facing == "S":
            return (x, y - 1, "S")

        if facing == "E":
            return (x + 1, y, "E")

        if facing == "W":
            return (x - 1, y, "W")


    # Backward

    if move == "BW":

        if facing == "N":
            return (x, y - 1, "N")

        if facing == "S":
            return (x, y + 1, "S")

        if facing == "E":
            return (x - 1, y, "E")

        if facing == "W":
            return (x + 1, y, "W")


    # Forward-right turn

    if move == "FR":

        if facing == "N":
            return (x + 3, y + 2, "E")

        if facing == "E":
            return (x + 3, y - 2, "S")

        if facing == "S":
            return (x - 3, y - 2, "W")

        if facing == "W":
            return (x - 3, y + 2, "N")


    # Forward-left turn

    if move == "FL":

        if facing == "N":
            return (x - 3, y + 2, "W")

        if facing == "W":
            return (x - 3, y - 2, "S")

        if facing == "S":
            return (x + 3, y - 2, "E")

        if facing == "E":
            return (x + 3, y + 2, "N")

    # Backward-right turn

    if move == "BR":

        if facing == "N":
            return (x + 3, y - 4, "W")

        if facing == "W":
            return (x + 3, y + 4, "S")

        if facing == "S":
            return (x - 3, y + 4, "E")

        if facing == "E":
            return (x - 3, y - 4, "N")


    # Backward-left turn

    if move == "BL":

        if facing == "N":
            return (x - 3, y - 4, "E")

        if facing == "E":
            return (x - 3, y + 4, "S")

        if facing == "S":
            return (x + 3, y + 4, "W")

        if facing == "W":
            return (x + 3, y - 4, "N")


    return robot

def viewing_pose(obstacle):

    obstacle_x = obstacle[1]
    obstacle_y = obstacle[2]
    image_face = obstacle[3]


    if image_face == "S":
        return (obstacle_x, obstacle_y - 4, "N")


    if image_face == "N":
        return (obstacle_x, obstacle_y + 4, "S")


    if image_face == "E":
        return (obstacle_x + 4, obstacle_y, "W")


    if image_face == "W":
        return (obstacle_x - 4, obstacle_y, "E")


def is_valid(pose, obstacles):

    x = pose[0]
    y = pose[1]


    # Check robot stays inside the arena

    if x < 1 or x > 18:
        return False

    if y < 1 or y > 18:
        return False


    # Check robot does not get too close to an obstacle

    for obstacle in obstacles:

        obstacle_x = obstacle[1]
        obstacle_y = obstacle[2]

        if abs(x - obstacle_x) <= 2:

            if abs(y - obstacle_y) <= 2:

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

    possible_moves = ["FW", "BW", "FR", "FL","BR", "BL"]

    valid_moves = []


    for move in possible_moves:

        new_pose = apply(pose, move)

        if is_valid(new_pose, obstacles):

            if move == "FW":
                cost = 1

            if move == "BW":
                cost = 1.5

            if move == "FR":
                cost = 4

            if move == "FL":
                cost = 4

            if move == "BR":
                cost = 5

            if move == "BL":
                cost = 5


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

    return distance * 0.5

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

        if move == "FW":
            total_cost = total_cost + 1

        if move == "BW":
            total_cost = total_cost + 1.5

        if move == "FR":
            total_cost = total_cost + 4

        if move == "FL":
            total_cost = total_cost + 4

        if move == "BR":
            total_cost = total_cost + 5

        if move == "BL":
            total_cost = total_cost + 5

    return total_cost

def find_best_order(start_pose, viewing_poses, obstacles):

    best_order = None
    best_cost = None


    for order in permutations(viewing_poses):

        current = start_pose

        total_cost = 0

        valid_order = True


        for goal in order:

            path = find_path(current, goal, obstacles)

            if path is None:
                valid_order = False
                break

            total_cost = total_cost + path_cost(path)

            current = goal


        if valid_order == True:

            if best_cost is None or total_cost < best_cost:

                best_cost = total_cost

                best_order = order


    return best_order, best_cost

# =====================================================================
# Section added for the RPi <-> Algo integration test.
# Everything above this line is unchanged.
#
# This gives algo_server.py the single entry point it calls:
#     planner.plan(obstacles, start_pose=(1, 1, "N"))
# =====================================================================

import time

CELL_CM = 10

# Distances (in cells) we are willing to stand off from the obstacle,
# best first. 4 is the nominal photo distance.
VIEW_DISTANCES = [4, 5, 3]

# Sideways offsets we are willing to accept if we cannot stand
# dead centre in front of the image.
VIEW_OFFSETS = [0, -1, 1]

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

    candidates = []

    for distance in VIEW_DISTANCES:
        for offset in VIEW_OFFSETS:

            if image_face == "S":
                candidates.append((obstacle_x + offset, obstacle_y - distance, "N"))

            elif image_face == "N":
                candidates.append((obstacle_x + offset, obstacle_y + distance, "S"))

            elif image_face == "E":
                candidates.append((obstacle_x + distance, obstacle_y + offset, "W"))

            elif image_face == "W":
                candidates.append((obstacle_x - distance, obstacle_y + offset, "E"))

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
    turn_map = {
        "FL": "LT90",
        "FR": "RT90",
        "BL": "XL90",
        "BR": "XR90",
    }

    commands = []
    i = 0

    while i < len(path):
        move = path[i]

        if move == "FW" or move == "BW":
            count = 0

            while i < len(path) and path[i] == move:
                count += 1
                i += 1

            commands.append(move + str(count * CELL_CM))
            continue

        if move in turn_map:
            commands.append(turn_map[move])
            i += 1
            continue

        if move.startswith("SNAP"):
            commands.append(move)
            i += 1
            continue

        raise ValueError("Unknown movement primitive: " + str(move))

    return commands


def plan(obstacles, start_pose=(1, 1, "N")):
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
        snap = "SNAP" + str(obstacle_id)

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
