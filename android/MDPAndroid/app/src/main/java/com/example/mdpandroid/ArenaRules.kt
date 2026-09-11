package com.example.mdpandroid

data class RemoteRobotPose(
    val pose: RobotState,
    val conflictingObstacle: Obstacle?
)

fun robotOverlapsObstacle(robot: RobotState, obstacles: List<Obstacle>): Obstacle? =
    obstacles.firstOrNull { robot.occupies(it.x, it.y) }

fun localRobotPose(x: Int, y: Int, direction: Face, obstacles: List<Obstacle>): RobotState? {
    val center = clampRobotCenter(x, y)
    val pose = RobotState(center.x, center.y, direction)
    return pose.takeUnless { robotOverlapsObstacle(it, obstacles) != null }
}

fun remoteRobotPose(x: Int, y: Int, direction: Face, obstacles: List<Obstacle>): RemoteRobotPose {
    val center = clampRobotCenter(x, y)
    val pose = RobotState(center.x, center.y, direction)
    return RemoteRobotPose(pose, robotOverlapsObstacle(pose, obstacles))
}

fun nextRobotPose(command: RobotCommand, robot: RobotState): RobotState? = when (command) {
    RobotCommand.FORWARD -> robot.copy(x = robot.x + robot.direction.dx, y = robot.y + robot.direction.dy)
    RobotCommand.REVERSE -> robot.copy(x = robot.x - robot.direction.dx, y = robot.y - robot.direction.dy)
    RobotCommand.TURN_LEFT -> robot.copy(direction = robot.direction.turnLeft())
    RobotCommand.TURN_RIGHT -> robot.copy(direction = robot.direction.turnRight())
    RobotCommand.STOP, RobotCommand.BEGIN_EXPLORE, RobotCommand.BEGIN_FASTEST -> null
}
