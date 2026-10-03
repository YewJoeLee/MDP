package com.example.mdpandroid

/** Commands understood by the robot-side protocol and the AMD Tool defaults. */
enum class RobotCommand(val wireValue: String) {
    FORWARD("f"),
    REVERSE("r"),
    TURN_LEFT("tl"),
    TURN_RIGHT("tr"),
    STOP("STOP"),
    BEGIN_EXPLORE("beginExplore"),
    BEGIN_FASTEST("beginFastest")
}

/** Single source of truth for Bluetooth wire tokens and message formatting. */
object RobotProtocol {
    const val ADD = "ADD"
    const val SUBTRACT = "SUB"
    const val FACE = "FACE"
    const val ROBOT = "ROBOT"
    const val MESSAGE = "MSG"
    const val TARGET = "TARGET"

    fun command(command: RobotCommand): String = command.wireValue

    fun addObstacle(id: String, point: GridPoint): String = "$ADD,$id,(${point.x},${point.y})"

    fun removeObstacle(id: String): String = "$SUBTRACT,$id"

    fun setObstacleFace(id: String, face: Face, point: GridPoint): String =
        "$FACE,$id,${face.code},(${point.x},${point.y})"

    fun robotPose(robot: RobotState): String = "$ROBOT,${robot.x},${robot.y},${robot.direction.code}"

    /** Renders every obstacle as one combined line of (id, x, y, "face") tuples — e.g.
     * [(1, 5, 12, "S"), (2, 14, 16, "W")] — instead of a separate ADD/FACE pair per obstacle. */
    fun obstacleList(obstacles: List<Obstacle>): String =
        obstacles.joinToString(prefix = "[", postfix = "]", separator = ", ") { obstacle ->
            val numericId = obstacle.id.removePrefix("B")
            val direction = obstacle.targetFace?.code.orEmpty()
            "($numericId, ${obstacle.x}, ${obstacle.y}, \"$direction\")"
        }
}
