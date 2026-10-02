package com.example.mdpandroid

internal data class ArenaChange(val state: AppState, val commands: List<String> = emptyList(), val message: String? = null)

internal fun planObstacleMove(state: AppState, id: String, point: GridPoint): ArenaChange {
    val obstacle = state.obstacles.firstOrNull { it.id == id } ?: return ArenaChange(state)
    if (point.x !in 0 until MAP_COLUMNS || point.y !in 0 until MAP_ROWS) {
        return ArenaChange(state.copy(obstacles = state.obstacles.filterNot { it.id == id },
            selectedObstacleId = state.selectedObstacleId.takeUnless { it == id }))
    }
    if (state.robot.occupies(point.x, point.y) || state.obstacles.any { it.id != id && it.x == point.x && it.y == point.y }) {
        return ArenaChange(state, message = "Cannot move $id there: the cell is occupied")
    }
    return ArenaChange(state.copy(obstacles = state.obstacles.map { if (it.id == id) it.copy(x = point.x, y = point.y) else it },
        selectedObstacleId = id))
}

internal fun planClearObstacleFace(state: AppState, id: String): ArenaChange {
    if (state.obstacles.none { it.id == id && it.targetFace != null }) return ArenaChange(state)
    val updated = state.copy(obstacles = clearObstacleFace(state.obstacles, id))
    return ArenaChange(updated, message = RobotMessages.selectedFaceCleared(id))
}
