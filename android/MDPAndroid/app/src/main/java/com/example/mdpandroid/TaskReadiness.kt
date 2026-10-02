package com.example.mdpandroid

import java.io.Serializable

enum class TaskPhase { STARTING, RUNNING, DELIVERY_UNKNOWN }

data class TaskRun(
    val command: RobotCommand,
    val phase: TaskPhase,
    val simulated: Boolean = false
) : Serializable

val AppState.arenaLocked: Boolean get() = taskRun != null

val AppState.canSendArena: Boolean
    get() = (connected || demoMode) && !arenaLocked && !arenaSendPending

val AppState.canDrive: Boolean
    get() = (connected || demoMode) && !arenaLocked

/** Recognition results and live pose updates do not change the obstacle layout sent to the Pi. */
fun AppState.taskStartIssue(): String? = when {
    arenaLocked -> "A task is already active. Wait for it to finish."
    !connected && !demoMode -> "Connect to the robot before starting a task."
    driveCommandPending -> "Wait for the drive command to finish sending."
    arenaSendPending -> "Wait for the arena to finish sending."
    obstacles.isEmpty() -> "Add at least one obstacle before starting a task."
    obstacles.any { it.targetFace == null } -> "Set a face for every obstacle before starting a task."
    localRobotPose(robot.x, robot.y, robot.direction, obstacles) == null -> "Set a valid robot position before starting a task."
    sentArena != RobotProtocol.obstacleList(obstacles) -> "Send the current arena before starting a task."
    else -> null
}

fun TaskRun.description(): String {
    val task = if (command == RobotCommand.BEGIN_EXPLORE) "Task 1" else "Task 2"
    return when (phase) {
        TaskPhase.STARTING -> "$task is starting. The map is locked."
        TaskPhase.RUNNING -> "$task is running. The map is locked until the task finishes."
        TaskPhase.DELIVERY_UNKNOWN -> "$task delivery is unconfirmed. The map remains locked while the robot may be running."
    }
}
