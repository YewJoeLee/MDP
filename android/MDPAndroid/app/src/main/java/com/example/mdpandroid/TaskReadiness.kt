package com.example.mdpandroid

import java.io.Serializable

enum class TaskPhase { STARTING, RUNNING, DELIVERY_UNKNOWN }

data class TaskRun(
    val command: RobotCommand,
    val phase: TaskPhase,
    val simulated: Boolean = false
) : Serializable

val AppState.taskActive: Boolean get() = taskRun != null

val AppState.canSendArena: Boolean
    get() = (connected || demoMode) && !taskActive && !arenaSendPending

val AppState.canDrive: Boolean
    get() = (connected || demoMode) && !taskActive

/** Starting a task requires the current obstacle layout to have been sent successfully. */
fun AppState.taskStartIssue(): String? = when {
    taskActive -> "A task is already active. Wait for it to finish."
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
        TaskPhase.STARTING -> "$task is starting."
        TaskPhase.RUNNING -> "$task is running."
        TaskPhase.DELIVERY_UNKNOWN -> "$task delivery is unconfirmed. The robot may be running."
    }
}
