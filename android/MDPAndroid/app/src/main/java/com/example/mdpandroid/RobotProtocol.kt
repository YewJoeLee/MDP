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
}
