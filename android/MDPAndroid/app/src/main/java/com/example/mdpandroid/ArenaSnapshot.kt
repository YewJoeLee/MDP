package com.example.mdpandroid

import java.io.Serializable

/** Arena data that must remain available when Android recreates the Activity. */
data class ArenaSnapshot(
    val obstacles: List<Obstacle>,
    val selectedObstacleId: String?,
    val robot: RobotState
) : Serializable

fun AppState.arenaSnapshot(): ArenaSnapshot =
    ArenaSnapshot(obstacles, selectedObstacleId, robot)

fun AppState.withArenaSnapshot(snapshot: ArenaSnapshot): AppState = copy(
    obstacles = snapshot.obstacles,
    selectedObstacleId = snapshot.selectedObstacleId?.takeIf { id ->
        snapshot.obstacles.any { it.id == id }
    },
    robot = snapshot.robot
)
