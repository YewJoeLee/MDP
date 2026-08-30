# MDP Algorithms — B.1 / B.2 / B.3

Path planning simulator for the MDP robot. Covers checklist items B.1, B.2 and B.3.

## Run it

```
pip install pygame
python simulator.py
```

A window opens showing the arena. Click on the window first, then use the keys below.

## Keys

| Key | Does |
|---|---|
| `1` | Plan the **B.2** route — nearest neighbour |
| `2` | Plan the **B.3** route — fastest, every order tried |
| `P` | Play / pause the animation |
| `R` | Reset the robot to the start |
| `H` | Show / hide the no-go zone around obstacles |

Normal demo: press `2`, then `P`.

## The files

| File | What it is |
|---|---|
| `config.py` | Every number — arena size, robot size, turning radius, move costs. Change numbers **here only**. |
| `test_maps.py` | The obstacle layout: `(id, x, y, image_face)` for each obstacle. |
| `planner.py` | All the thinking — motion primitives, A\*, and the two tour planners. No drawing. |
| `simulator.py` | All the drawing — the pygame window. No thinking. |

## What you're looking at

- **Grid** — 20 × 20 cells, each 10 cm, so 200 × 200 cm
- **Green square, bottom-left** — the 40 × 40 cm start zone
- **Red squares** — obstacles, numbered
- **Yellow stripe on an obstacle** — the face the image is on
- **Brown shading** — no-go zone; the robot's body never touches it
- **Blue square** — the robot (3 × 3 cells), white line is its nose
- **Green dots** — where it has already been
- **Big yellow number beside an obstacle** — when it gets visited

## How it works, in four lines

1. The robot only has six moves: `FW BW FL FR BL BR`. Turns are 90° and sweep 3 cells forward and 3 sideways, because the car can't spin on the spot.
2. `is_cell_allowed()` decides where the robot may stand. Turns are checked along the whole arc, not just at the endpoints.
3. `find_path()` is A\* — cheapest sequence of moves between two poses.
4. `plan()` costs every trip between images once, then picks the visiting order twice: greedily for B.2, and by trying all 120 orders for B.3.

Move costs are **seconds, not centimetres** (`FW` 1, turn 4, reverse turn 5), which is what makes B.3 shortest *time* rather than shortest distance.

## Before the real robot

Measure the car and update these in `config.py` — nothing else needs to change:

`TURN_CELLS` (how far a 90° turn actually displaces the car), `MOVE_COST` (timed with a stopwatch), `SAFETY_CELLS` (clearance), `VIEW_DISTANCE` (how far back to photograph from).
