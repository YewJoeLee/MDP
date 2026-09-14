package com.example.mdpandroid

import kotlin.math.abs

internal const val MAP_COLUMNS = 20
internal const val MAP_ROWS = 20

data class GridPoint(val x: Int, val y: Int)

enum class Face(val code: String, val dx: Int, val dy: Int) {
    N("N", 0, 1),
    E("E", 1, 0),
    S("S", 0, -1),
    W("W", -1, 0);

    fun turnRight(): Face = when (this) {
        N -> E
        E -> S
        S -> W
        W -> N
    }

    fun turnLeft(): Face = when (this) {
        N -> W
        W -> S
        S -> E
        E -> N
    }
}

data class RobotState(
    val x: Int = 6,
    val y: Int = 2,
    val direction: Face = Face.W
)

/** The robot occupies a square footprint centered on (x,y); a radius of 1 means 3x3. */
const val ROBOT_FOOTPRINT_RADIUS = 1

fun RobotState.occupies(x: Int, y: Int): Boolean =
    abs(x - this.x) <= ROBOT_FOOTPRINT_RADIUS && abs(y - this.y) <= ROBOT_FOOTPRINT_RADIUS

/** Clamps a robot center so its whole footprint stays on the MAP_COLUMNS x MAP_ROWS grid. */
fun clampRobotCenter(x: Int, y: Int): GridPoint = GridPoint(
    x.coerceIn(ROBOT_FOOTPRINT_RADIUS, MAP_COLUMNS - 1 - ROBOT_FOOTPRINT_RADIUS),
    y.coerceIn(ROBOT_FOOTPRINT_RADIUS, MAP_ROWS - 1 - ROBOT_FOOTPRINT_RADIUS)
)

data class Obstacle(
    val id: String,
    val x: Int,
    val y: Int,
    val targetId: String? = null,
    val targetFace: Face? = null
)

/** A monotonic [order] keeps the combined activity view faithful when events share a timestamp. */
data class StatusMessage(val time: String, val text: String, val order: Long = 0)

data class ActivityLogEntry(val message: StatusMessage, val source: ActivitySource)

enum class ActivitySource { STATUS, RECEIVED }

fun mergeActivityLog(
    statusMessages: List<StatusMessage>,
    receivedRawMessages: List<StatusMessage>
): List<ActivityLogEntry> =
    (statusMessages.map { ActivityLogEntry(it, ActivitySource.STATUS) } +
        receivedRawMessages.map { ActivityLogEntry(it, ActivitySource.RECEIVED) })
        .sortedBy { it.message.order }

sealed interface ProtocolMessage {
    data class Text(val text: String) : ProtocolMessage
    data class Target(val obstacleId: String, val targetId: String, val face: Face?) : ProtocolMessage
    data class Robot(val x: Int, val y: Int, val direction: Face?) : ProtocolMessage
}

fun parseProtocolMessage(line: String): ProtocolMessage? {
    val parts = line.split(",").map { it.trim().removePrefix("[").removeSuffix("]") }
    return when (parts.firstOrNull()?.uppercase()) {
        RobotProtocol.MESSAGE -> parts.drop(1).joinToString(",").takeIf { it.isNotBlank() }?.let(ProtocolMessage::Text)
        RobotProtocol.TARGET -> {
            val id = parts.getOrNull(1)?.let(::canonicalObstacleId) ?: return null
            val target = parts.getOrNull(2)?.takeIf { it.isNotBlank() } ?: return null
            val face = parts.getOrNull(3)?.uppercase()?.let { code -> Face.entries.firstOrNull { it.code == code } }
            ProtocolMessage.Target(id, target, face)
        }
        RobotProtocol.ROBOT -> {
            val x = parts.getOrNull(1)?.toIntOrNull() ?: return null
            val y = parts.getOrNull(2)?.toIntOrNull() ?: return null
            val direction = parts.getOrNull(3)?.let { rawDirection ->
                rawDirection.uppercase().let { code -> Face.entries.firstOrNull { it.code == code } } ?: return null
            }
            ProtocolMessage.Robot(x, y, direction)
        }
        else -> null
    }
}

fun canonicalObstacleId(rawId: String): String {
    val cleaned = rawId.trim().uppercase()
    return if (cleaned.startsWith("B")) cleaned else "B$cleaned"
}

fun applyTargetRecognition(obstacles: List<Obstacle>, message: ProtocolMessage.Target): List<Obstacle> =
    obstacles.map { obstacle ->
        if (obstacle.id.equals(message.obstacleId, ignoreCase = true)) {
            obstacle.copy(targetId = message.targetId, targetFace = message.face ?: obstacle.targetFace)
        } else {
            obstacle
        }
    }

fun clearObstacleFace(obstacles: List<Obstacle>, id: String): List<Obstacle> =
    obstacles.map { obstacle -> if (obstacle.id == id) obstacle.copy(targetFace = null) else obstacle }
