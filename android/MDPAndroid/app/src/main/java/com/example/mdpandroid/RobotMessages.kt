package com.example.mdpandroid

/** Controller and demo status templates, kept apart from Compose screen labels. */
object RobotMessages {
    const val PERMISSION_REQUIRED_TO_SCAN = "Bluetooth permission is required before scanning"
    const val PERMISSION_REQUIRED_TO_CONNECT = "Bluetooth permission is required before connecting"
    const val PERMISSION_DENIED = "Bluetooth permission denied"
    const val BLUETOOTH_CONNECTION_ESTABLISHED = "Bluetooth connection established"
    const val BLUETOOTH_DISCONNECTED = "Bluetooth disconnected"
    const val RETRYING_CONNECTION = "Retrying Bluetooth connection automatically"
    const val BLUETOOTH_DEVICE_DISCONNECTED = "Bluetooth device disconnected"
    const val SCAN_COMPLETE = "Bluetooth scan complete"
    const val SCAN_TIMED_OUT = "Bluetooth scan timed out"
    const val SCAN_CANNOT_START = "Bluetooth could not start scanning"
    const val TURN_ON_BLUETOOTH = "Turn on Bluetooth and scan again"
    const val BLUETOOTH_UNAVAILABLE = "This device has no Bluetooth adapter"
    const val SCANNING = "Scanning for nearby devices..."
    const val DISCONNECTED_BY_USER = "Disconnected by user"
    const val CONNECTION_TIMED_OUT = "Bluetooth connection timed out"
    const val ARENA_SYNC_PREPARED = "Demo arena sync prepared"
    const val DEMO_SCAN_SKIPPED = "Demo mode: Bluetooth scan skipped"
    const val DEMO_NO_BLUETOOTH_DEVICES = "Demo mode: no Bluetooth devices"

    fun selectedDevice(name: String): String = "Selected $name"
    fun deviceNotFound(address: String): String = "Could not find $address"
    fun connectingTo(name: String): String = "Connecting to $name..."
    fun connectedTo(name: String): String = "Connected to $name"
    fun connectionFailed(reason: String): String = "Connection failed: $reason"
    fun waitingToReconnect(address: String): String = "Waiting to reconnect to $address..."
    fun commandNotSent(command: String): String = "Not connected; command not sent: $command"
    fun commandSent(command: String): String = "Sent: $command"
    fun demoCommand(command: RobotCommand): String = "Demo command: ${RobotProtocol.command(command)}"
    fun obstacleCannotBePlacedOnRobot(): String = "Cannot place an obstacle on the robot"
    fun obstaclePlaced(id: String, point: GridPoint): String = "$id placed at (${point.x},${point.y})"
    fun obstacleRemovedRemotely(id: String): String = "$id removed (from remote)"
    fun obstacleFaceSetRemotely(id: String, face: Face): String = "$id face ${face.code} set (from remote)"
    fun remoteObstacleIgnored(id: String, point: GridPoint): String =
        "Ignored $id at (${point.x},${point.y}): robot is there"
    fun remoteObstaclePlaced(id: String, point: GridPoint): String =
        "$id placed at (${point.x},${point.y}) (from remote)"
    fun robotPoseBlocked(obstacle: Obstacle): String =
        "Robot move blocked: ${obstacle.id} occupies its footprint"
    fun remoteRobotConflict(obstacle: Obstacle): String =
        "Remote robot overlaps ${obstacle.id}; keeping reported robot pose"
    fun robotStartSet(robot: RobotState): String = "Robot start set to (${robot.x},${robot.y})"
    fun robotPoseSet(robot: RobotState): String =
        "Robot start set to (${robot.x},${robot.y}) facing ${robot.direction.code}"
    fun robotFacingSet(face: Face): String = "Robot facing set to ${face.code}"
    fun arenaSetupSent(obstacleCount: Int): String = "Arena setup sent ($obstacleCount obstacle(s))"
    fun selectedFaceCleared(id: String): String = "Cleared selected face for $id"
}
