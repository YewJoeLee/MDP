package com.example.mdpandroid

import java.io.Serializable

/** Arena data used for Activity restoration and persistent map storage. */
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

/** Selection, task execution and Bluetooth readiness are not retained in the saved map. */
fun AppState.persistentArenaSnapshot(): ArenaSnapshot = ArenaSnapshot(obstacles, null, robot)
