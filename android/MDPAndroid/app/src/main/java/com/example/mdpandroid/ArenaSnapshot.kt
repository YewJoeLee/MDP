package com.example.mdpandroid

import java.io.Serializable

/** Arena data that must remain available when Android recreates the Activity. */
data class ArenaSnapshot(
    val obstacles: List<Obstacle>,
    val selectedObstacleId: String?,
    val robot: RobotState,
    val taskRun: TaskRun? = null
) : Serializable

fun AppState.arenaSnapshot(): ArenaSnapshot =
    ArenaSnapshot(obstacles, selectedObstacleId, robot, taskRun)

fun AppState.withArenaSnapshot(snapshot: ArenaSnapshot): AppState = copy(
    obstacles = snapshot.obstacles,
    selectedObstacleId = snapshot.selectedObstacleId?.takeIf { id ->
        snapshot.obstacles.any { it.id == id }
    },
    robot = snapshot.robot,
    taskRun = snapshot.taskRun?.copy(phase = TaskPhase.DELIVERY_UNKNOWN),
    sentArena = null
)
