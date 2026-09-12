"""
planner.py
----------
All the thinking. No drawing - simulator.py does that.

A "pose" is always a flat tuple of three simple values:  (x, y, facing)
x and y are the robot's CENTRE cell, facing is "N", "E", "S" or "W".
It must stay a flat tuple because A* uses it as a dictionary key.

Order of the file:
    apply_move()        one move changes the pose
    turn_sweep_cells()  the cells a turn sweeps through
    viewing_pose()      where to stand to photograph an image
    is_valid()          may the robot be here
    neighbours()        where can it go from here
    find_path()         A* between two poses
    plan()              B.2 and B.3 tours
"""

import heapq
import itertools
import math
import config


# ---------------------------------------------------------------------
# DIRECTION TABLES
# ---------------------------------------------------------------------

#Robot needs to move forward, but 'forward' is different depending on which direction it is facing
FORWARD_STEP = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}

# One step to the robot's right, and to its left.
RIGHT_STEP = {"N": (1, 0), "E": (0, -1), "S": (-1, 0), "W": (0, 1)}
LEFT_STEP = {"N": (-1, 0), "E": (0, 1), "S": (1, 0), "W": (0, -1)}

# Which way you end up facing after turning 90 degrees. 
# For eg, if robot is currently facing north, then after turning 90 degrees to the right with either FR or BL, so we use RIGHT_OF
# Likewise for when robot is turning 90 degrees to the left (FL or BR), we use LEFT_OF.
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}

# The opposite direction, used to work out viewing poses.
OPPOSITE_OF = {"N": "S", "S": "N", "E": "W", "W": "E"}

# ---------------------------------------------------------------------
# MOTION PRIMITIVES
# ---------------------------------------------------------------------

def apply_move(pose, move):
    """Take a pose and a move, return the new pose.

    Straight moves step one cell along the facing direction.

    Turns are the interesting part. The car cannot spin on the spot - it
    drives round a quarter circle. Driving forward while steering right
    from facing North lands it 3 cells further north AND 3 cells further
    east, now facing East.

    Reversing is the mirror image. Reverse with the wheels turned right
    and the body ends up to the right, but the nose swings the OTHER way,
    so the robot ends up facing left. That is real car behaviour.
    """
    x = pose[0]
    y = pose[1]
    facing = pose[2]

    forward_x = FORWARD_STEP[facing][0]
    forward_y = FORWARD_STEP[facing][1]
    right_x = RIGHT_STEP[facing][0]
    right_y = RIGHT_STEP[facing][1]
    left_x = LEFT_STEP[facing][0]
    left_y = LEFT_STEP[facing][1]

    turn = config.TURN_CELLS

    if move == "FW":
        new_x = x + forward_x
        new_y = y + forward_y
        new_facing = facing

    elif move == "BW":
        new_x = x - forward_x
        new_y = y - forward_y
        new_facing = facing

    elif move == "FR":
        new_x = x + (forward_x * turn) + (right_x * turn)
        new_y = y + (forward_y * turn) + (right_y * turn)
        new_facing = RIGHT_OF[facing]

    elif move == "FL":
        new_x = x + (forward_x * turn) + (left_x * turn)
        new_y = y + (forward_y * turn) + (left_y * turn)
        new_facing = LEFT_OF[facing]

    elif move == "BR":
        new_x = x - (forward_x * turn) + (right_x * turn)
        new_y = y - (forward_y * turn) + (right_y * turn)
        new_facing = LEFT_OF[facing]

    elif move == "BL":
        new_x = x - (forward_x * turn) + (left_x * turn)
        new_y = y - (forward_y * turn) + (left_y * turn)
        new_facing = RIGHT_OF[facing]

    else:
        raise ValueError("Unknown move: " + str(move))

    return (new_x, new_y, new_facing)


def is_turn(move):
    """True if this move is one of the four 90 degree turns."""
    return move in ("FL", "FR", "BL", "BR")


def turn_sweep_cells(pose, move):
    """The cells the robot's centre passes through during a turn.

    A turn is not a teleport. If we only checked the start and end of a
    turn, the planner would happily sweep the car straight through an
    obstacle corner. So we sample three points along the quarter circle.
    It makes sure that the entire turn is valid, i.e the robot does not cross boundary / hit obstacle during turn
    """
    x = pose[0]
    y = pose[1]
    facing = pose[2]
    turn = config.TURN_CELLS

    # The car pivots around a point TURN_CELLS cells to its left or right.
    if move in ("FR", "BR"):
        side = RIGHT_STEP[facing]
    else:
        side = LEFT_STEP[facing]

    centre_x = x + (side[0] * turn)
    centre_y = y + (side[1] * turn)

    # The arm from that pivot back to the robot. It rotates 90 degrees.
    arm_x = x - centre_x
    arm_y = y - centre_y

    # FR and BL sweep clockwise, FL and BR sweep anticlockwise.
    if move in ("FR", "BL"):
        direction = -1.0
    else:
        direction = 1.0

    cells = []

    for fraction in [0.25, 0.5, 0.75]:

        angle = direction * fraction * (math.pi / 2.0)
        cos_part = math.cos(angle)
        sin_part = math.sin(angle)

        rotated_x = (arm_x * cos_part) - (arm_y * sin_part)
        rotated_y = (arm_x * sin_part) + (arm_y * cos_part)

        sample_x = centre_x + rotated_x
        sample_y = centre_y + rotated_y

        cell_x = round(sample_x)
        cell_y = round(sample_y)

        cells.append((cell_x, cell_y))

    return cells


# ---------------------------------------------------------------------
# WHERE MUST THE ROBOT STAND TO SEE AN IMAGE?
# ---------------------------------------------------------------------

def viewing_pose(obstacle):
    """The pose the robot must reach to photograph one image.

    The image is on one face of the obstacle, so the robot stands in front
    of that face, VIEW_DISTANCE cells away, looking back at it.

    Example: image on the SOUTH face of the obstacle at (a, b). South means
    the picture points downwards, so the robot stands below it at (a, b-4)
    and looks NORTH.
    """
    obstacle_x = obstacle[1]
    obstacle_y = obstacle[2]
    face = obstacle[3]

    step = FORWARD_STEP[face]

    stand_x = obstacle_x + (step[0] * config.VIEW_DISTANCE)
    stand_y = obstacle_y + (step[1] * config.VIEW_DISTANCE)

    return (stand_x, stand_y, OPPOSITE_OF[face])


# ---------------------------------------------------------------------
# IS THE ROBOT ALLOWED TO BE HERE?
# ---------------------------------------------------------------------

def is_cell_allowed(x, y, obstacles):
    """True if the robot's centre may sit on this cell.

    Two questions. First, does the whole 3 x 3 body fit inside the arena?
    The body covers x-1 .. x+1, so the centre cannot sit on the very edge.

    Second, is it far enough from every obstacle? The body reaches
    ROBOT_REACH cells out, and we demand SAFETY_CELLS of empty space on
    top of that. So the centre is banned within (reach + safety) cells of
    an obstacle - which is exactly the same as saying the robot's body may
    never touch the obstacle grown by SAFETY_CELLS in every direction.
    """
    smallest = config.ROBOT_REACH
    largest = config.GRID - 1 - config.ROBOT_REACH

    if x < smallest or x > largest:
        return False
    if y < smallest or y > largest:
        return False

    banned = config.ROBOT_REACH + config.SAFETY_CELLS

    for obstacle in obstacles:

        #Inflate the obstacle
        gap_x = abs(obstacle[1] - x) 
        gap_y = abs(obstacle[2] - y)

        if gap_x <= banned and gap_y <= banned:
            return False

    return True


def is_valid(pose, obstacles):
    """True if this pose is legal.

    The body is a square, so the facing makes no difference to whether it
    fits. That is why a square 3 x 3 footprint was chosen.
    """
    return is_cell_allowed(pose[0], pose[1], obstacles)


# ---------------------------------------------------------------------
# WHERE CAN THE ROBOT GO FROM HERE?
# ---------------------------------------------------------------------

def neighbours(pose, obstacles):
    """Returns every legal next step, as a list of (new_pose, move, cost).

    This is the only thing A* needs to know about the world.
    """
    results = []

    for move in config.ALL_MOVES:

        new_pose = apply_move(pose, move)

        # Is the place we land legal?
        if is_valid(new_pose, obstacles) is False:
            continue

        # If it is a turn, did we sweep through anything on the way?
        if is_turn(move) is True:

            blocked = False

            for cell in turn_sweep_cells(pose, move):
                if is_cell_allowed(cell[0], cell[1], obstacles) is False:
                    blocked = True

            if blocked is True:
                continue

        results.append((new_pose, move, config.MOVE_COST[move]))

    return results


# ---------------------------------------------------------------------
# A* - THE FASTEST WAY FROM ONE POSE TO ANOTHER
# ---------------------------------------------------------------------

# A* needs a guess of the remaining cost, and the guess must never be too
# big or A* stops being guaranteed to find the best answer - and B.3 is
# graded on the best answer.
#
# A turn costs 4 and moves the robot 3 cells forward plus 3 cells sideways
# = 6 cells of progress, so 4/6 of a cost unit per cell. That is the
# cheapest progress any move can buy (a straight FW buys 1 cell for 1).
# So we scale the Manhattan distance down by that factor.
CHEAPEST_PER_CELL = config.MOVE_COST["FR"] / (config.TURN_CELLS * 2.0)


def heuristic(pose_a, pose_b):
    """Optimistic guess of the cost from pose_a to pose_b. Ignores facing."""
    steps = abs(pose_a[0] - pose_b[0]) + abs(pose_a[1] - pose_b[1])
    return steps * CHEAPEST_PER_CELL


def find_path(start_pose, goal_pose, obstacles):
    """A* search. Returns (list_of_moves, total_cost).

    If there is no way through, returns ([], infinity). It never crashes
    and never loops forever.
    """
    open_set = [(0.0, start_pose)] #Contains states that A* needs to visit

    came_from = {}                    # pose -> (previous_pose, move). B came from A using FR. Allows us to reconstruct final path
    g_score = {start_pose: 0.0}       # Actual real cost found to each pose
    closed = set()                    # poses we are finished with

    while len(open_set) > 0:

        cheapest = heapq.heappop(open_set)
        current = cheapest[1]

        if current in closed:
            continue

        closed.add(current)

        if current == goal_pose: #Reconstruct the final path
            moves = []
            pose = current
            while pose in came_from:
                moves.append(came_from[pose][1])
                pose = came_from[pose][0]
            moves.reverse()
            return moves, g_score[current]

        for next_pose, move, step_cost in neighbours(current, obstacles):
            if next_pose in closed:
                continue

            tentative = g_score[current] + step_cost

            if next_pose in g_score: # What is the best route to this pose we already knew about?
                best_known = g_score[next_pose]
            else:
                best_known = float("inf")

            if tentative < best_known:
                g_score[next_pose] = tentative
                came_from[next_pose] = (current, move)
                f_score = tentative + heuristic(next_pose, goal_pose)
                heapq.heappush(open_set, (f_score, next_pose))

    return [], float("inf")


# ---------------------------------------------------------------------
# B.2 AND B.3 - WHAT ORDER TO VISIT THEM IN
# ---------------------------------------------------------------------

def build_cost_matrix(start_pose, stops, obstacles):
    """Cost and route between every pair of stops, worked out ONCE.

    With 5 images that is 30 A* runs. Caching matters: B.3 below asks
    "how much from A to B?" 600 times, and without this cache each of
    those would be a fresh search.

    Returns two dictionaries keyed by (from_pose, to_pose).
    """
    costs = {}
    routes = {}

    places = [start_pose] + stops

    for from_pose in places:
        for to_pose in stops:
            if from_pose == to_pose:
                continue
            moves, cost = find_path(from_pose, to_pose, obstacles)
            costs[(from_pose, to_pose)] = cost
            routes[(from_pose, to_pose)] = moves

    return costs, routes


def order_cost(start_pose, order, costs):
    """Total cost of visiting a list of stops in this exact order."""
    total = 0.0
    current = start_pose

    for stop in order:
        total = total + costs[(current, stop)]
        current = stop

    return total


def nearest_neighbour_order(start_pose, stops, costs):
    """B.2 - a sensible order.

    Greedy: from wherever you are, always go to the cheapest place you
    have not visited yet. Fast and usually decent, but not always the
    best, because taking the cheap option now can strand you far from the
    last two images and cost you a huge final leg.
    """
    order = []
    unvisited = list(stops)
    current = start_pose

    while len(unvisited) > 0:

        best_stop = unvisited[0]
        best_cost = costs[(current, best_stop)]

        for candidate in unvisited:
            if costs[(current, candidate)] < best_cost:
                best_cost = costs[(current, candidate)]
                best_stop = candidate

        order.append(best_stop)
        unvisited.remove(best_stop)
        current = best_stop

    return order


def best_order(start_pose, stops, costs):
    """B.3 - the FASTEST order, proved by trying every possible order.

    5 images means 5! = 120 orderings. Because the costs are already
    cached this runs in well under a second, so there is no need to guess.
    """
    winner = None
    winner_cost = float("inf")

    for candidate in itertools.permutations(stops):

        cost = order_cost(start_pose, list(candidate), costs)

        if cost < winner_cost:
            winner_cost = cost
            winner = list(candidate)

    return winner, winner_cost


def construct_moves_list(start_pose, order, routes, pose_to_id):
    """Glue the separate legs into ONE flat list of moves.

    A "SNAP" marker is dropped in wherever the robot stops to photograph
    an image, so the robot knows where to pause.
    """

    moves = []
    current = start_pose

    for stop in order:

        for move in routes[(current, stop)]:
            moves.append(move)

        moves.append("SNAP" + str(pose_to_id[stop]))
        current = stop

    return moves


def plan(obstacles, start_pose=config.START_POSE):
    """Work out both tours. Returns a dictionary the simulator can draw.

    Steps: find where to stand for each image, cost every trip between
    those places once, then pick the order twice - greedily for B.2, and
    by trying every order for B.3.
    """
    stops = [] #List of locations where robot must stop (i.e 20 - 50cm away from image of each of the 5 obstacles)
    pose_to_id = {} #Viewing Pose : ID

    #Compute stops list and pose_to_id hashmap
    for obstacle in obstacles:
        pose = viewing_pose(obstacle)
        stops.append(pose)
        pose_to_id[pose] = obstacle[0]

    costs, routes = build_cost_matrix(start_pose, stops, obstacles) #Runs A* between all pairs of stops

    # If the map is ever edited into something impossible, say so clearly
    # instead of failing in a confusing way later on.
    for cost in costs.values():
        if cost == float("inf"):
            raise ValueError(
                "Some images cannot be reached with this obstacle layout. "
                "Move the obstacles further apart in test_maps.py."
            )

    greedy = nearest_neighbour_order(start_pose, stops, costs)
    greedy_cost = order_cost(start_pose, greedy, costs)

    optimal, optimal_cost = best_order(start_pose, stops, costs)

    saved = greedy_cost - optimal_cost
    improvement = (saved / greedy_cost) * 100.0

    return {
        "pose_to_id": pose_to_id,
        "greedy_order": greedy,
        "greedy_cost": greedy_cost,
        "greedy_moves": construct_moves_list(start_pose, greedy, routes, pose_to_id),
        "optimal_order": optimal,
        "optimal_cost": optimal_cost,
        "optimal_moves": construct_moves_list(start_pose, optimal, routes, pose_to_id),
        "improvement": improvement,
        "orders_tried": math.factorial(len(stops)),
    }


# ---------------------------------------------------------------------
# TURNING A MOVE LIST INTO ANIMATION FRAMES
# ---------------------------------------------------------------------

def build_frames(start_pose, moves):
    """Expand a move list into the poses the animation should show.

    A straight move is one frame. A turn becomes four frames - three along
    the quarter circle plus the landing pose - so you can watch the car
    sweep round and see that it keeps its clearance the whole way, instead
    of jumping 3 cells.

    Each frame is (pose, label, is_snap).
    """
    frames = [(start_pose, "-", False)]
    pose = start_pose

    for move in moves:

        if move.startswith("SNAP"):
            frames.append((pose, move, True))
            continue

        if is_turn(move) is True:

            landing = apply_move(pose, move)

            for cell in turn_sweep_cells(pose, move):
                frames.append(((cell[0], cell[1], landing[2]), move, False))

            pose = landing

        else:
            pose = apply_move(pose, move)

        frames.append((pose, move, False))

    return frames

