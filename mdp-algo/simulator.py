"""
simulator.py
------------
The picture you show the supervisor. All the thinking is in planner.py.

Run it with:   python simulator.py

KEYS
  1   plan and load the B.2 route (nearest neighbour)
  2   plan and load the B.3 route (fastest - every order tried)
  P   play / pause
  R   reset the robot to the start
  H   show / hide the no-go zone around the obstacles
"""

import sys

import pygame

import config
import planner
import test_maps


# ---------------------------------------------------------------------
# GRID CELLS TO SCREEN PIXELS
# ---------------------------------------------------------------------

def cell_to_px(cell_x, cell_y):
    """Top-left pixel of a grid cell.

    This is the ONLY place that knows the screen's y axis points down
    while our grid's y axis points up. If the start zone ever appears in
    the top-left instead of the bottom-left, the bug is here.
    """
    pixel_x = config.MARGIN_LEFT + (cell_x * config.PX)
    pixel_y = config.MARGIN_TOP + ((config.GRID - 1 - cell_y) * config.PX)
    return pixel_x, pixel_y


def cell_centre_px(cell_x, cell_y):
    """The pixel in the middle of a cell."""
    corner = cell_to_px(cell_x, cell_y)
    half = config.PX // 2
    return corner[0] + half, corner[1] + half


def fill_cell(surface, cell_x, cell_y, colour, thickness=0):
    """Paint one grid cell. thickness=0 fills it, 1 draws an outline."""
    corner = cell_to_px(cell_x, cell_y)
    box = (corner[0], corner[1], config.PX, config.PX)
    pygame.draw.rect(surface, colour, box, thickness)


# ---------------------------------------------------------------------
# DRAWING THE ARENA
# ---------------------------------------------------------------------

def draw_start_zone(surface):
    """The 40 x 40 cm start zone in the bottom-left corner."""
    for cell_x in range(config.START_ZONE_SIZE):
        for cell_y in range(config.START_ZONE_SIZE):
            fill_cell(surface, cell_x, cell_y, config.COLOUR_START_ZONE)


def draw_halo(surface, obstacles):
    """The no-go zone: each obstacle grown by SAFETY_CELLS in every direction.

    The robot's body must never touch this shading. Because the robot is
    modelled 5 cm larger than it really is on every side, the real car
    keeps a 15 cm gap - and the turn check in planner.py enforces it
    through the whole quarter circle, not just at the endpoints.
    """
    reach = config.SAFETY_CELLS

    for obstacle in obstacles:
        for offset_x in range(-reach, reach + 1):
            for offset_y in range(-reach, reach + 1):

                cell_x = obstacle[1] + offset_x
                cell_y = obstacle[2] + offset_y

                if 0 <= cell_x < config.GRID and 0 <= cell_y < config.GRID:
                    fill_cell(surface, cell_x, cell_y, config.COLOUR_HALO)


def draw_grid_lines(surface):
    """A thin outline round every cell, so it reads as a grid map."""
    for cell_x in range(config.GRID):
        for cell_y in range(config.GRID):
            fill_cell(surface, cell_x, cell_y, config.COLOUR_GRID_LINE, 1)


def draw_axis_labels(surface, font):
    """Cell numbers 0-19 along the bottom and down the left side."""
    bottom = config.MARGIN_TOP + (config.GRID * config.PX) + 6

    for cell in range(config.GRID):

        along = cell_centre_px(cell, 0)
        label = font.render(str(cell), True, config.COLOUR_DIM_TEXT)
        surface.blit(label, (along[0] - 6, bottom))

        down = cell_centre_px(0, cell)
        label = font.render(str(cell), True, config.COLOUR_DIM_TEXT)
        surface.blit(label, (8, down[1] - 8))


def draw_obstacles(surface, obstacles, font_small, font_big, visit_numbers):
    """Each obstacle, its number, and the face its image is on.

    The thick yellow line is the image position - it tells you which side
    the robot has to approach from.
    """
    for obstacle in obstacles:

        obstacle_id = obstacle[0]
        cell_x = obstacle[1]
        cell_y = obstacle[2]
        face = obstacle[3]

        fill_cell(surface, cell_x, cell_y, config.COLOUR_OBSTACLE)

        corner = cell_to_px(cell_x, cell_y)
        left = corner[0]
        top = corner[1]
        right = left + config.PX
        bottom = top + config.PX

        label = font_small.render(str(obstacle_id), True, (255, 255, 255))
        surface.blit(label, (left + 12, top + 9))

        if face == "N":
            line = ((left, top), (right, top))
        elif face == "S":
            line = ((left, bottom), (right, bottom))
        elif face == "E":
            line = ((right, top), (right, bottom))
        else:
            line = ((left, top), (left, bottom))

        pygame.draw.line(surface, config.COLOUR_IMAGE_FACE, line[0], line[1], 6)

        # Once a route is planned, show when this obstacle gets visited.
        if obstacle_id in visit_numbers:
            order = str(visit_numbers[obstacle_id])
            label = font_big.render(order, True, config.COLOUR_IMAGE_FACE)
            surface.blit(label, (right + 4, top - 6))


# ---------------------------------------------------------------------
# DRAWING THE ROBOT
# ---------------------------------------------------------------------

def draw_trail(surface, frames, upto):
    """Faint dots showing everywhere the robot has already been."""
    for index in range(upto + 1):
        pose = frames[index][0]
        pygame.draw.circle(
            surface, config.COLOUR_TRAIL, cell_centre_px(pose[0], pose[1]), 3
        )


def draw_robot(surface, pose):
    """The robot: a 3 x 3 block with a clear nose showing its facing.

    pose[0] and pose[1] are the CENTRE cell. The body is the 9 cells
    around that centre.
    """
    centre_x = pose[0]
    centre_y = pose[1]
    facing = pose[2]

    # Top-left of the body. Note the +1 on y: on screen the top of the
    # body is the row ABOVE the centre.
    top_left = cell_to_px(centre_x - 1, centre_y + 1)
    size = config.PX * config.ROBOT_SIZE
    body = (top_left[0], top_left[1], size, size)

    pygame.draw.rect(surface, config.COLOUR_ROBOT, body)
    pygame.draw.rect(surface, config.COLOUR_ROBOT_EDGE, body, 3)

    middle = cell_centre_px(centre_x, centre_y)
    step = planner.FORWARD_STEP[facing]
    reach = int(config.PX * 1.4)

    nose = (middle[0] + (step[0] * reach), middle[1] - (step[1] * reach))
    pygame.draw.line(surface, config.COLOUR_ROBOT_EDGE, middle, nose, 5)
    pygame.draw.circle(surface, config.COLOUR_ROBOT_EDGE, nose, 6)


# ---------------------------------------------------------------------
# THE INFORMATION PANEL
# ---------------------------------------------------------------------

def draw_panel(surface, fonts, state):
    """Everything the supervisor needs to read while you talk."""
    panel_x = config.MARGIN_LEFT + (config.GRID * config.PX) + 8
    pygame.draw.rect(
        surface,
        config.COLOUR_PANEL,
        (panel_x, 0, config.PANEL_WIDTH, config.WINDOW_HEIGHT),
    )

    left = panel_x + 12
    line_y = 12

    def write(text, colour, font, gap=20):
        """Draw one line of text and step down the panel."""
        label = font.render(text, True, colour)
        surface.blit(label, (left, line_y))
        return line_y + gap

    small = fonts["small"]
    medium = fonts["medium"]

    # Build the arena description from the constants, so the panel can
    # never disagree with config.py.
    arena_cm = config.GRID * config.CELL_CM
    arena_text = "Arena " + str(arena_cm) + " x " + str(arena_cm) + " cm"
    arena_text = arena_text + "  |  " + str(config.GRID) + " x "
    arena_text = arena_text + str(config.GRID) + " cells of "
    arena_text = arena_text + str(config.CELL_CM) + " cm"

    line_y = write("MDP ALGO SIMULATOR", config.COLOUR_TEXT, medium, 26)
    line_y = write(arena_text, config.COLOUR_DIM_TEXT, small, 26)

    pose = state["pose"]
    line_y = write(
        "Robot: (" + str(pose[0]) + ", " + str(pose[1]) + ") facing " + pose[2],
        config.COLOUR_ROBOT_EDGE, medium, 24)

    line_y = write("Move: " + state["move"], config.COLOUR_TEXT, small)
    line_y = write("Step " + str(state["step"]) + " of " + str(state["total"]),
                   config.COLOUR_DIM_TEXT, small)

    if state["playing"] is True:
        line_y = write("PLAYING", config.COLOUR_GOOD, small, 28)
    else:
        line_y = write("PAUSED  (press P)", config.COLOUR_DIM_TEXT, small, 28)

    line_y = write("PATH COST  (lower = faster)", config.COLOUR_TEXT, medium, 24)

    if state["plan"] is None:
        line_y = write("Press 1 or 2 to plan a route.",
                       config.COLOUR_DIM_TEXT, small, 28)
    else:
        result = state["plan"]

        line_y = write("B.2  nearest neighbour : "
                       + str(round(result["greedy_cost"], 1)),
                       config.COLOUR_DIM_TEXT, small)
        line_y = write("B.3  best of all orders: "
                       + str(round(result["optimal_cost"], 1)),
                       config.COLOUR_IMAGE_FACE, small)
        line_y = write("improvement: "
                       + str(round(result["improvement"], 1)) + " %",
                       config.COLOUR_GOOD, small)
        line_y = write("orders tried: " + str(result["orders_tried"])
                       + " (all of them)", config.COLOUR_DIM_TEXT, small)
        line_y = write("showing: " + state["showing"], config.COLOUR_TEXT, small)
        line_y = write("visit order: " + state["order_text"],
                       config.COLOUR_TEXT, small, 28)

    line_y = write("COST MODEL (seconds, not cm)", config.COLOUR_TEXT, medium, 24)
    line_y = write("FW 1.0   BW 1.5", config.COLOUR_DIM_TEXT, small, 18)
    line_y = write("FL 4.0   FR 4.0", config.COLOUR_DIM_TEXT, small, 18)
    line_y = write("BL 5.0   BR 5.0", config.COLOUR_DIM_TEXT, small, 18)
    line_y = write("turns cost more, so this is", config.COLOUR_DIM_TEXT, small, 18)
    line_y = write("shortest TIME, not shortest distance",
                   config.COLOUR_DIM_TEXT, small, 28)

    line_y = write("KEYS", config.COLOUR_TEXT, medium, 24)
    line_y = write("1  plan B.2 route    2  plan B.3 route",
                   config.COLOUR_DIM_TEXT, small, 18)
    line_y = write("P  play / pause      R  reset",
                   config.COLOUR_DIM_TEXT, small, 18)
    line_y = write("H  no-go zone on / off", config.COLOUR_DIM_TEXT, small, 18)


# ---------------------------------------------------------------------
# THE MAIN PROGRAM
# ---------------------------------------------------------------------

def main():

    pygame.init()

    screen = pygame.display.set_mode(
        (config.WINDOW_WIDTH, config.WINDOW_HEIGHT)
    )
    pygame.display.set_caption("MDP Algorithms Simulator - B.1 / B.2 / B.3")
    clock = pygame.time.Clock()

    fonts = {
        "small": pygame.font.SysFont("consolas", 14),
        "medium": pygame.font.SysFont("consolas", 17, bold=True),
        "big": pygame.font.SysFont("consolas", 22, bold=True),
    }

    obstacles = test_maps.OBSTACLES

    # The animation is just a list of poses and an index into it.
    frames = [(config.START_POSE, "-", False)]
    index = 0
    playing = False
    last_step_time = 0
    step_delay = 220              # milliseconds between frames

    plan_result = None
    showing = "-"
    order_text = "-"
    visit_numbers = {}
    show_halo = True

    def load_route(which):
        """Plan a route and set up the animation for it.

        which is "greedy" for B.2 or "optimal" for B.3.
        """
        result = planner.plan(obstacles, config.START_POSE)

        moves = result[which + "_moves"]
        order = result[which + "_order"]

        new_frames = planner.build_frames(config.START_POSE, moves)

        # Which obstacle is visited first, second, third...
        numbers = {}
        position = 1

        for stop in order:
            numbers[result["pose_to_id"][stop]] = position
            position = position + 1

        # A readable "3 > 2 > 1 > 5 > 4" for the panel.
        pieces = []
        for stop in order:
            pieces.append(str(result["pose_to_id"][stop]))

        return result, new_frames, numbers, " > ".join(pieces)

    running = True

    while running:

        now = pygame.time.get_ticks()

        # --- input ---------------------------------------------------
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_p:
                    playing = not playing
                    last_step_time = now

                elif event.key == pygame.K_r:
                    index = 0
                    playing = False

                elif event.key == pygame.K_h:
                    show_halo = not show_halo

                elif event.key == pygame.K_1:
                    plan_result, frames, visit_numbers, order_text = \
                        load_route("greedy")
                    showing = "B.2 nearest-neighbour route"
                    index = 0
                    playing = False

                elif event.key == pygame.K_2:
                    plan_result, frames, visit_numbers, order_text = \
                        load_route("optimal")
                    showing = "B.3 fastest route"
                    index = 0
                    playing = False

        # --- advance the animation -----------------------------------
        # We do NOT use time.sleep() - that freezes the window. Instead we
        # look at the clock and decide whether it is time for the next
        # frame yet.
        if playing is True:

            if index < len(frames) - 1:

                # Pause longer on a SNAP so the photo stop is visible.
                if frames[index + 1][2] is True:
                    wait_for = step_delay * 4
                else:
                    wait_for = step_delay

                if now - last_step_time >= wait_for:
                    index = index + 1
                    last_step_time = now

            else:
                playing = False

        # --- draw ----------------------------------------------------
        screen.fill(config.COLOUR_BACKGROUND)

        draw_start_zone(screen)

        if show_halo is True:
            draw_halo(screen, obstacles)

        draw_grid_lines(screen)
        draw_axis_labels(screen, fonts["small"])
        draw_obstacles(screen, obstacles, fonts["small"], fonts["big"],
                       visit_numbers)
        draw_trail(screen, frames, index)
        draw_robot(screen, frames[index][0])

        draw_panel(screen, fonts, {
            "pose": frames[index][0],
            "move": frames[index][1],
            "step": index,
            "total": len(frames) - 1,
            "playing": playing,
            "plan": plan_result,
            "showing": showing,
            "order_text": order_text,
        })

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
