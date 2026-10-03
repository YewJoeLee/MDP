# Image id, X position, Y position, Image is on which side of the obstacle 
# Existing active map preserved. IDs identify obstacles, not detected symbols.
# Lowercase faces are accepted. A cell (7,7) occupies [70,80] x [70,80] cm.

OBSTACLES = [
    (1, 3, 8, "E"),
    (2, 6, 14, "S"),
    (3, 7, 3, "N"),
    (4, 11, 10, "W"),
    (5, 12, 17, "S"),
    (6, 16, 12, "W"),
    (7, 15, 5, "N")
]

# For a single test, replace OBSTACLES above, or uncomment ONE assignment:
# OBSTACLES = [(1, 7, 7, "w")]
# OBSTACLES = [(1, 7, 7, "n")]
# Both are deliberately omitted with current AC20's full travel envelope.
# See end_to_end_examples.md for explanations and movement-only diagnostics.
# Example with room for the existing AC20 envelope in the 200 cm arena:
# OBSTACLES = [(1, 17, 10, "W")]
