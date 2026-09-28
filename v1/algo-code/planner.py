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
            return (x + 3, y + 3, "E")

        if facing == "E":
            return (x + 3, y - 3, "S")

        if facing == "S":
            return (x - 3, y - 3, "W")

        if facing == "W":
            return (x - 3, y + 3, "N")


    # Forward-left turn

    if move == "FL":

        if facing == "N":
            return (x - 3, y + 3, "W")

        if facing == "W":
            return (x - 3, y - 3, "S")

        if facing == "S":
            return (x + 3, y - 3, "E")

        if facing == "E":
            return (x + 3, y + 3, "N")

    # Backward-right turn

    if move == "BR":

        if facing == "N":
            return (x + 3, y - 3, "W")

        if facing == "W":
            return (x + 3, y + 3, "S")

        if facing == "S":
            return (x - 3, y + 3, "E")

        if facing == "E":
            return (x - 3, y - 3, "N")


    # Backward-left turn

    if move == "BL":

        if facing == "N":
            return (x - 3, y - 3, "E")

        if facing == "E":
            return (x - 3, y + 3, "S")

        if facing == "S":
            return (x + 3, y + 3, "W")

        if facing == "W":
            return (x + 3, y - 3, "N")


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