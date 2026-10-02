package com.example.mdpandroid

internal enum class ArenaGestureAction { TAP, FACE, MOVE }

/** Classifies a completed release using canvas pixel distances and its grid destination. */
internal fun classifyArenaGesture(
    moved: Boolean,
    distance: Float,
    tapSlop: Float,
    cellSize: Float,
    releasedAt: GridPoint,
    obstacleSelected: Boolean
): ArenaGestureAction = when {
    obstacleSelected && (releasedAt.x !in 0 until MAP_COLUMNS || releasedAt.y !in 0 until MAP_ROWS) -> ArenaGestureAction.MOVE
    !moved || distance < tapSlop -> ArenaGestureAction.TAP
    distance < cellSize * 0.65f -> ArenaGestureAction.FACE
    else -> ArenaGestureAction.MOVE
}
