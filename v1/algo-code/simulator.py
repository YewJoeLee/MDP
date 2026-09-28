import pygame
from planner import apply, viewing_pose,choose_visit_order, build_full_path,find_best_order,path_cost

pygame.init()

#Obstacle coordinates to create the obstacle simulation in pygame
OBSTACLES = [
    (1, 5, 12, "S"),
    (2, 14, 16, "W"),
    (3, 17, 6, "N"),
    (4, 8, 4, "E"),
    (5, 12, 9, "S")
]

# Robot is stored as:
# (x, y, facing)

START_ROBOT = (1, 1, "N")
robot = START_ROBOT

VIEWING_POSES = []

for obstacle in OBSTACLES:

    pose = viewing_pose(obstacle)

    VIEWING_POSES.append(pose)


GREEDY_ORDER = choose_visit_order(
    START_ROBOT,
    VIEWING_POSES,
    OBSTACLES
)

GREEDY_PATH = build_full_path(
    START_ROBOT,
    GREEDY_ORDER,
    OBSTACLES
)

GREEDY_COST = path_cost(GREEDY_PATH)

BEST_ORDER, BEST_COST = find_best_order(
    START_ROBOT,
    VIEWING_POSES,
    OBSTACLES
)

OPTIMAL_PATH = build_full_path(
    START_ROBOT,
    BEST_ORDER,
    OBSTACLES
)



print("Greedy order:", GREEDY_ORDER)

print("Greedy cost:", GREEDY_COST)

print("Greedy path:", GREEDY_PATH)

print("Best order:", BEST_ORDER)

print("Best cost:", BEST_COST)

print("Optimal path:", OPTIMAL_PATH)

PATH = OPTIMAL_PATH

"""
# Task B1 
PATH = [
    "FW",
    "FW",
    "BW",
    "FR",
    "FW",
    "FL"
]
"""

# Grid settings
GRID = 20
CELL_CM = 10
PX = 40

# Grid (0,0) is BOTTOM-LEFT
# x goes right
# y goes up

WIDTH = 800
HEIGHT = 800

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("MDP Algo-Simulator")

# Colours
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (180, 230, 180)
GREY = (200, 200, 200)

OBSTACLE_COLOUR = (70, 70, 70)
IMAGE_COLOUR = (255, 80, 80)

ROBOT_COLOUR = (80, 120, 255)
ROBOT_DIRECTION_COLOUR = (255, 255, 0)

font = pygame.font.Font(None, 20)
obstacle_font = pygame.font.Font(None, 24)

def cell_to_px(cx, cy):
    px = cx * PX
    py = (GRID - 1 - cy) * PX

    return px, py

def draw_robot(surface, robot):

    robot_x = robot[0]
    robot_y = robot[1]
    facing = robot[2]

    # The robot is 3 x 3 cells
    # robot_x and robot_y represent the centre cell

    px, py = cell_to_px(robot_x - 1, robot_y + 1)

    pygame.draw.rect(
        surface,
        ROBOT_COLOUR,
        (px, py, PX * 3, PX * 3)
    )


    # Find the centre of the robot

    centre_px, centre_py = cell_to_px(robot_x, robot_y)

    centre_x = centre_px + PX // 2
    centre_y = centre_py + PX // 2


    # Draw a line showing which way the robot faces

    if facing == "N":

        pygame.draw.line(
            surface,
            ROBOT_DIRECTION_COLOUR,
            (centre_x, centre_y),
            (centre_x, centre_y - PX),
            6
        )


    if facing == "S":

        pygame.draw.line(
            surface,
            ROBOT_DIRECTION_COLOUR,
            (centre_x, centre_y),
            (centre_x, centre_y + PX),
            6
        )


    if facing == "E":

        pygame.draw.line(
            surface,
            ROBOT_DIRECTION_COLOUR,
            (centre_x, centre_y),
            (centre_x + PX, centre_y),
            6
        )


    if facing == "W":

        pygame.draw.line(
            surface,
            ROBOT_DIRECTION_COLOUR,
            (centre_x, centre_y),
            (centre_x - PX, centre_y),
            6
        )


path_index = 0
playing = False
last_move_time = 0

running = True

while running:

    # Check events
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:

            if event.key == pygame.K_p:

                robot = START_ROBOT
                path_index = 0
                playing = True
                last_move_time = pygame.time.get_ticks()

    if playing == True:

        current_time = pygame.time.get_ticks()

        if current_time - last_move_time >= 1000:

            if path_index < len(PATH):

                move = PATH[path_index]

                robot = apply(robot, move)

                path_index = path_index + 1

                last_move_time = current_time

            else:

                playing = False

    # Background
    screen.fill(WHITE)

    # Draw start zone
    for x in range(4):
        for y in range(4):

            px, py = cell_to_px(x, y)

            pygame.draw.rect(
                screen,
                GREEN,
                (px, py, PX, PX)
            )

    # Draw grid
    for x in range(GRID):
        for y in range(GRID):

            px, py = cell_to_px(x, y)

            pygame.draw.rect(
                screen,
                GREY,
                (px, py, PX, PX),
                1
            )

    # Draw obstacles

    for obstacle in OBSTACLES:

        obstacle_id = obstacle[0]
        obstacle_x = obstacle[1]
        obstacle_y = obstacle[2]
        image_face = obstacle[3]

        px, py = cell_to_px(obstacle_x, obstacle_y)


        # Draw obstacle

        pygame.draw.rect(
            screen,
            OBSTACLE_COLOUR,
            (px, py, PX, PX)
        )


        # Draw obstacle number

        text = obstacle_font.render(
            str(obstacle_id),
            True,
            WHITE
        )

        screen.blit(
            text,
            (px + 15, py + 10)
        )


        # Draw image face

        if image_face == "N":

            pygame.draw.line(
                screen,
                IMAGE_COLOUR,
                (px, py),
                (px + PX, py),
                5
            )


        if image_face == "S":

            pygame.draw.line(
                screen,
                IMAGE_COLOUR,
                (px, py + PX),
                (px + PX, py + PX),
                5
            )


        if image_face == "W":

            pygame.draw.line(
                screen,
                IMAGE_COLOUR,
                (px, py),
                (px, py + PX),
                5
            )


        if image_face == "E":

            pygame.draw.line(
                screen,
                IMAGE_COLOUR,
                (px + PX, py),
                (px + PX, py + PX),
                5
            )

    draw_robot(screen, robot)

    # Draw x-axis numbers
    for x in range(GRID):

        text = font.render(str(x), True, BLACK)

        px, py = cell_to_px(x, 0)

        screen.blit(
            text,
            (px + 15, py + 22)
        )

    # Draw y-axis numbers
    for y in range(GRID):

        text = font.render(str(y), True, BLACK)

        px, py = cell_to_px(0, y)

        screen.blit(
            text,
            (px + 2, py + 2)
        )

    pygame.display.flip()


pygame.quit()