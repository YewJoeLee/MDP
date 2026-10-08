# Image id, X position, Y position, Image is on which side of the obstacle 
# Existing active map preserved. IDs identify obstacles, not detected symbols.
# Lowercase faces are accepted. A cell (7,7) occupies [70,80] x [70,80] cm.

OBSTACLES = [
    (1, 12, 12, "S"),
    (2, 8, 16, "W"),
    (3, 4, 7, "S"),
    (4, 13, 4, "N"),
    (5, 8, 3, "W"),
    (6, 9, 8, "E"),
    (7, 3, 12, "E"),
    (8, 16, 16, "S"),
]

# For a single test, replace OBSTACLES above, or uncomment ONE assignment:
# OBSTACLES = [(1, 7, 7, "w")]
# OBSTACLES = [(1, 7, 7, "n")]
# Both are deliberately omitted with current AC20's full travel envelope.
# See end_to_end_examples.md for explanations and movement-only diagnostics.
# Example with room for the existing AC20 envelope in the 200 cm arena:
# OBSTACLES = [(1, 17, 10, "W")]
