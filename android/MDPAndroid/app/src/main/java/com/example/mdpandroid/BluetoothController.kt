package com.example.mdpandroid

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothServerSocket
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Handler
import android.os.Looper
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.core.content.ContextCompat
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import java.util.concurrent.Executors

data class BluetoothDeviceInfo(val address: String, val name: String?)

data class AppState(
    val devices: List<BluetoothDeviceInfo> = emptyList(),
    val selectedDevice: BluetoothDeviceInfo? = null,
    val connectedAddress: String? = null,
    val selectedDeviceName: String? = null,
    val connected: Boolean = false,
    val scanning: Boolean = false,
    val connectionStatus: String = "Disconnected",
    val connectionDetail: String = "Select a device, or use Demo mode to test the interface.",
    val obstacles: List<Obstacle> = emptyList(),
    val selectedObstacleId: String? = null,
    val robot: RobotState = RobotState(),
    val statusMessages: List<StatusMessage> = emptyList(),
    /** Raw Bluetooth input, rendered with the curated status feed in the single activity log. */
    val receivedRawLog: List<StatusMessage> = emptyList()
)

/** Classic Bluetooth SPP transport used by the AMD Tool and the robot-side serial bridge. */
class BluetoothController(private val context: Context) {
    companion object {
        private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        private const val RECONNECT_DELAY_MS = 3_000L
        private const val MAX_RECONNECT_DELAY_MS = 30_000L
        private const val CONNECT_TIMEOUT_MS = 12_000L
        private const val SCAN_TIMEOUT_MS = 18_000L
        private const val SERVER_RETRY_DELAY_MS = 5_000L
        private const val INCOMING_IDLE_FLUSH_MS = 100L
        private const val SERVICE_NAME = "MDPAndroid"
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val connectionExecutor = Executors.newCachedThreadPool()
    private val writeExecutor = Executors.newSingleThreadExecutor()
    private val adapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    private val socketLock = Any()
    private var socket: BluetoothSocket? = null
    private var output: OutputStream? = null
    private var pendingSocket: BluetoothSocket? = null
    private var connectionAttemptId = 0L
    private var activeSessionId = 0L
    private var connectTimeoutRunnable: Runnable? = null
    private var lastDevice: BluetoothDevice? = null
    private var reconnectRunnable: Runnable? = null
    @Volatile private var closed = false
    private var registered = false
    private var serverSocket: BluetoothServerSocket? = null
    @Volatile private var listening = false
    private var scanTimeoutRunnable: Runnable? = null
    private var reconnectAttempt = 0
    private val incomingBuffer = StringBuilder()
    private var incomingFlushRunnable: Runnable? = null
    private var nextLogOrder = 0L

    var state by mutableStateOf(AppState())
        private set

    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                BluetoothAdapter.ACTION_DISCOVERY_STARTED -> {
                    state = state.copy(scanning = true)
                }
                BluetoothDevice.ACTION_FOUND -> {
                    val device = intent.parcelableBluetoothDevice() ?: return
                    addDevice(device)
                }
                BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                    finishScan(RobotMessages.SCAN_COMPLETE)
                }
            }
        }
    }

    init {
        registerReceiver()
    }

    @SuppressLint("MissingPermission")
    fun refreshDevices() {
        if (!hasBluetoothPermission()) return
        val paired = try { adapter?.bondedDevices.orEmpty().map(::toInfo) } catch (_: SecurityException) { emptyList() }
        val selected = state.selectedDevice?.let { old -> paired.firstOrNull { it.address == old.address } ?: old }
        state = state.copy(
            devices = mergeDevices(paired, state.devices),
            selectedDevice = selected,
            selectedDeviceName = selected?.name ?: selected?.address
        )
    }

    @SuppressLint("MissingPermission")
    fun startScan() {
        if (!hasBluetoothPermission()) {
            addStatus(RobotMessages.PERMISSION_REQUIRED_TO_SCAN)
            return
        }
        val bluetoothAdapter = adapter
        if (bluetoothAdapter == null) {
            state = state.copy(connectionStatus = "Unavailable", connectionDetail = RobotMessages.BLUETOOTH_UNAVAILABLE)
            return
        }
        try {
            if (!bluetoothAdapter.isEnabled) {
                addStatus(RobotMessages.TURN_ON_BLUETOOTH)
                return
            }
            if (bluetoothAdapter.isDiscovering) bluetoothAdapter.cancelDiscovery()
            cancelScanTimeout()
            val paired = bluetoothAdapter.bondedDevices.orEmpty().map(::toInfo)
            state = state.copy(scanning = true, connectionDetail = RobotMessages.SCANNING)
            if (!bluetoothAdapter.startDiscovery()) {
                finishScan(RobotMessages.SCAN_CANNOT_START)
                return
            }
            val timeout = Runnable {
                if (state.scanning) {
                    try { bluetoothAdapter.cancelDiscovery() } catch (_: SecurityException) { }
                    finishScan(RobotMessages.SCAN_TIMED_OUT)
                }
            }
            scanTimeoutRunnable = timeout
            mainHandler.postDelayed(timeout, SCAN_TIMEOUT_MS)
            // A new scan should not keep stale, previously-discovered unpaired devices visible.
            state = state.copy(devices = mergeDevices(paired))
        } catch (_: SecurityException) {
            finishScan(RobotMessages.PERMISSION_DENIED)
            addStatus(RobotMessages.PERMISSION_DENIED)
        }
    }

    fun selectDevice(device: BluetoothDeviceInfo) {
        state = state.copy(selectedDevice = device, selectedDeviceName = device.name ?: device.address)
        addStatus(RobotMessages.selectedDevice(device.name ?: device.address))
    }

    fun selectObstacle(id: String) {
        if (state.obstacles.any { it.id == id }) state = state.copy(selectedObstacleId = id)
    }

    fun clearObstacleSelection() {
        state = state.copy(selectedObstacleId = null)
    }

    @SuppressLint("MissingPermission")
    fun connect(deviceInfo: BluetoothDeviceInfo) {
        if (!hasBluetoothPermission()) {
            addStatus(RobotMessages.PERMISSION_REQUIRED_TO_CONNECT)
            return
        }
        val bluetoothDevice = try { adapter?.getRemoteDevice(deviceInfo.address) } catch (_: Exception) { null }
        if (bluetoothDevice == null) {
            addStatus(RobotMessages.deviceNotFound(deviceInfo.address))
            return
        }
        val attemptId = synchronized(socketLock) {
            connectionAttemptId += 1
            activeSessionId += 1 // Invalidate any old read/write callbacks.
            closeCurrentSocketsLocked()
            connectionAttemptId
        }
        lastDevice = bluetoothDevice
        cancelConnectTimeout()
        cancelReconnect()
        state = state.copy(
            selectedDevice = deviceInfo,
            selectedDeviceName = deviceInfo.name ?: deviceInfo.address,
            connectionStatus = "Connecting",
            connectionDetail = RobotMessages.connectingTo(deviceInfo.name ?: deviceInfo.address),
            connected = false,
            connectedAddress = null
        )
        try { adapter?.cancelDiscovery() } catch (_: SecurityException) { }

        connectionExecutor.execute {
            try {
                val newSocket = openSocket(bluetoothDevice, attemptId)
                val sessionId = synchronized(socketLock) {
                    if (closed || connectionAttemptId != attemptId || pendingSocket !== newSocket) {
                        null
                    } else {
                        pendingSocket = null
                        activeSessionId += 1
                        socket = newSocket
                        output = newSocket.outputStream
                        activeSessionId
                    }
                }
                if (sessionId == null) {
                    try { newSocket.close() } catch (_: IOException) { }
                    return@execute
                }
                cancelConnectTimeout()
                beginSession(newSocket, deviceInfo, sessionId)
            } catch (error: IOException) {
                handleConnectionFailure(attemptId, RobotMessages.connectionFailed(error.message ?: "device unavailable"))
            } catch (_: SecurityException) {
                handleConnectionFailure(attemptId, RobotMessages.PERMISSION_DENIED, retry = false)
            }
        }
    }

    /**
     * The AMD Tool's documented default connection flow has the *tool* scan for and dial into the
     * Android device (the tool acts as Bluetooth client), which requires this app to hold an open
     * listening socket rather than only dialing out itself. This runs continuously in the
     * background so either direction of connection works: our own Scan/Connect UI (Android as
     * client), or the AMD Tool / robot initiating the connection to us (Android as server).
     */
    @SuppressLint("MissingPermission")
    fun startServerListening() {
        if (listening || closed || !hasBluetoothPermission()) return
        listening = true
        connectionExecutor.execute { runServerLoop() }
    }

    @SuppressLint("MissingPermission")
    private fun runServerLoop() {
        val bluetoothAdapter = adapter
        if (bluetoothAdapter == null) {
            listening = false
            return
        }
        try {
            val server = try {
                bluetoothAdapter.listenUsingRfcommWithServiceRecord(SERVICE_NAME, SPP_UUID)
            } catch (_: IOException) {
                null
            } ?: return
            serverSocket = server
            while (!closed) {
                val accepted = try { server.accept() } catch (_: IOException) { null } ?: break
                val sessionId = synchronized(socketLock) {
                    if (closed || socket != null) {
                        null
                    } else {
                        // An AMD Tool reconnect must win over a slow outgoing retry. Cancelling the
                        // pending client socket lets the app continue to accept its server-side flow.
                        connectionAttemptId += 1
                        try { pendingSocket?.close() } catch (_: IOException) { }
                        pendingSocket = null
                        activeSessionId += 1
                        socket = accepted
                        output = accepted.outputStream
                        activeSessionId
                    }
                }
                if (sessionId == null) {
                    try { accepted.close() } catch (_: IOException) { }
                    continue
                }
                val info = try { toInfo(accepted.remoteDevice) } catch (_: SecurityException) {
                    BluetoothDeviceInfo(accepted.remoteDevice.address, null)
                }
                cancelConnectTimeout()
                cancelReconnect()
                beginSession(accepted, info, sessionId)
            }
        } catch (_: SecurityException) {
            // Permission revoked mid-listen; stop quietly, startServerListening() can retry later.
        } finally {
            try { serverSocket?.close() } catch (_: IOException) { }
            serverSocket = null
            listening = false
            if (!closed && hasBluetoothPermission()) {
                mainHandler.postDelayed({ startServerListening() }, SERVER_RETRY_DELAY_MS)
            }
        }
    }

    /** Shared success path for a connection established either by dialing out or by accepting an incoming one. */
    private fun beginSession(newSocket: BluetoothSocket, deviceInfo: BluetoothDeviceInfo, sessionId: Long) {
        lastDevice = newSocket.remoteDevice
        reconnectAttempt = 0
        mainHandler.post { resetIncomingBuffer() }
        mainHandler.post {
            if (!isCurrentSession(sessionId, newSocket)) return@post
            state = state.copy(
                selectedDevice = deviceInfo,
                selectedDeviceName = deviceInfo.name ?: deviceInfo.address,
                connected = true,
                connectedAddress = deviceInfo.address,
                connectionStatus = "Connected",
                connectionDetail = RobotMessages.connectedTo(deviceInfo.name ?: deviceInfo.address)
            )
            addStatus(RobotMessages.BLUETOOTH_CONNECTION_ESTABLISHED)
        }
        readLoop(newSocket, sessionId)
    }

    fun disconnect() {
        cancelReconnect()
        cancelConnectTimeout()
        lastDevice = null
        synchronized(socketLock) {
            connectionAttemptId += 1
            activeSessionId += 1
            closeCurrentSocketsLocked()
        }
        mainHandler.post { resetIncomingBuffer() }
        state = state.copy(
            connected = false,
            connectedAddress = null,
            connectionStatus = "Disconnected",
            connectionDetail = RobotMessages.DISCONNECTED_BY_USER
        )
        addStatus(RobotMessages.BLUETOOTH_DISCONNECTED)
    }

    fun moveRobot(command: RobotCommand, sendToRobot: Boolean = true) {
        nextRobotPose(command, state.robot)?.let { candidate ->
            val updated = localRobotPose(candidate.x, candidate.y, candidate.direction, state.obstacles)
            if (updated == null) {
                robotOverlapsObstacle(candidate, state.obstacles)?.let {
                    addStatus(RobotMessages.robotPoseBlocked(it))
                }
                return
            }
            state = state.copy(robot = updated)
        }
        if (sendToRobot) send(RobotProtocol.command(command))
    }

    /**
     * AMDTOOL compares received command text against its configured command token. Keep the
     * payload exact by default; callers that need a line-based robot protocol can opt in to a
     * terminator explicitly.
     */
    fun send(command: String, appendLineTerminator: Boolean = false) {
        val connection = synchronized(socketLock) { Triple(socket, output, activeSessionId) }
        val currentSocket = connection.first
        val currentOutput = connection.second
        val sessionId = connection.third
        if (!state.connected || currentSocket == null || currentOutput == null) {
            addStatus(RobotMessages.commandNotSent(command))
            return
        }
        writeExecutor.execute {
            try {
                val payload = command.trim() + if (appendLineTerminator) "\n" else ""
                currentOutput.write(payload.toByteArray(Charsets.UTF_8))
                currentOutput.flush()
                mainHandler.post { addStatus(RobotMessages.commandSent(command)) }
            } catch (_: IOException) {
                mainHandler.post { markConnectionLost("Connection lost while sending", sessionId, currentSocket) }
            }
        }
    }

    fun addObstacle(point: GridPoint) {
        if (point.x !in 0 until MAP_COLUMNS || point.y !in 0 until MAP_ROWS) return
        if (state.robot.occupies(point.x, point.y)) {
            addStatus(RobotMessages.obstacleCannotBePlacedOnRobot())
            return
        }
        if (state.obstacles.any { it.x == point.x && it.y == point.y }) return
        val nextNumber = (state.obstacles.mapNotNull { it.id.removePrefix("B").toIntOrNull() }.maxOrNull() ?: 0) + 1
        val obstacle = Obstacle("B$nextNumber", point.x, point.y)
        state = state.copy(obstacles = state.obstacles + obstacle, selectedObstacleId = obstacle.id)
        send(RobotProtocol.addObstacle(obstacle.id, point))
    }

    fun moveObstacle(id: String, x: Int, y: Int) {
        val obstacle = state.obstacles.firstOrNull { it.id == id } ?: return
        val invalidTarget = x !in 0 until MAP_COLUMNS || y !in 0 until MAP_ROWS ||
            state.obstacles.any { it.id != id && it.x == x && it.y == y } ||
            state.robot.occupies(x, y)
        if (invalidTarget) {
            removeObstacle(id)
            return
        }
        state = state.copy(obstacles = state.obstacles.map { if (it.id == id) it.copy(x = x, y = y) else it }, selectedObstacleId = id)
        val point = GridPoint(x, y)
        send(RobotProtocol.addObstacle(id, point))
        if (obstacle.x != x || obstacle.y != y) addStatus(RobotMessages.obstaclePlaced(id, point))
    }

    fun removeObstacle(id: String) {
        if (state.obstacles.none { it.id == id }) return
        state = state.copy(obstacles = state.obstacles.filterNot { it.id == id }, selectedObstacleId = null)
        send(RobotProtocol.removeObstacle(id))
    }

    fun setObstacleFace(id: String, face: Face) {
        val obstacle = state.obstacles.firstOrNull { it.id == id } ?: return
        state = state.copy(obstacles = state.obstacles.map { if (it.id == id) it.copy(targetFace = face) else it }, selectedObstacleId = id)
        send(RobotProtocol.setObstacleFace(id, face, GridPoint(obstacle.x, obstacle.y)))
    }

    fun clearObstacleTarget(id: String) {
        state = state.copy(obstacles = clearObstacleFace(state.obstacles, id))
        addStatus(RobotMessages.selectedFaceCleared(id))
    }

    /** Updates the configured robot start position and syncs it to the remote side. */
    fun setRobotStart(x: Int, y: Int) {
        val robot = localRobotPose(x, y, state.robot.direction, state.obstacles) ?: run {
            reportRobotPoseBlocked(x, y, state.robot.direction)
            return
        }
        state = state.copy(robot = robot)
        send(RobotProtocol.robotPose(robot))
        addStatus(RobotMessages.robotStartSet(robot))
    }

    /** Updates the configured robot facing direction and syncs it to the remote side. */
    fun setRobotFace(face: Face) {
        val robot = localRobotPose(state.robot.x, state.robot.y, face, state.obstacles) ?: run {
            reportRobotPoseBlocked(state.robot.x, state.robot.y, face)
            return
        }
        state = state.copy(robot = robot)
        send(RobotProtocol.robotPose(robot))
        addStatus(RobotMessages.robotFacingSet(face))
    }

    /** Sets the robot's full starting pose (position + facing) in one update and syncs it to the remote side. */
    fun setRobotPose(x: Int, y: Int, direction: Face) {
        val robot = localRobotPose(x, y, direction, state.obstacles) ?: run {
            reportRobotPoseBlocked(x, y, direction)
            return
        }
        state = state.copy(robot = robot)
        send(RobotProtocol.robotPose(robot))
        addStatus(RobotMessages.robotPoseSet(robot))
    }

    private fun reportRobotPoseBlocked(x: Int, y: Int, direction: Face) {
        val center = clampRobotCenter(x, y)
        val candidate = RobotState(center.x, center.y, direction)
        robotOverlapsObstacle(candidate, state.obstacles)?.let {
            addStatus(RobotMessages.robotPoseBlocked(it))
        }
    }

    /** Sends the current arena layout as the same ADD, FACE, and ROBOT messages used live. */
    fun sendArenaSnapshot() {
        state.obstacles.forEach { obstacle ->
            val point = GridPoint(obstacle.x, obstacle.y)
            send(RobotProtocol.addObstacle(obstacle.id, point))
            obstacle.targetFace?.let { face ->
                send(RobotProtocol.setObstacleFace(obstacle.id, face, point))
            }
        }
        send(RobotProtocol.robotPose(state.robot))
        addStatus(RobotMessages.arenaSetupSent(state.obstacles.size))
    }

    fun addStatus(message: String) {
        state = state.copy(statusMessages = (state.statusMessages + newLogMessage(message)).takeLast(40))
    }

    private fun addRawReceived(line: String) {
        state = state.copy(receivedRawLog = (state.receivedRawLog + newLogMessage(line)).takeLast(100))
    }

    private fun newLogMessage(text: String): StatusMessage {
        val timestamp = java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.getDefault()).format(java.util.Date())
        nextLogOrder += 1
        return StatusMessage(timestamp, text, nextLogOrder)
    }

    fun deviceName(info: BluetoothDeviceInfo): String = info.name ?: info.address

    fun close() {
        closed = true
        cancelReconnect()
        cancelScanTimeout()
        cancelConnectTimeout()
        synchronized(socketLock) {
            connectionAttemptId += 1
            activeSessionId += 1
            closeCurrentSocketsLocked()
        }
        mainHandler.post { resetIncomingBuffer() }
        try { serverSocket?.close() } catch (_: IOException) { }
        if (registered) {
            try { context.unregisterReceiver(receiver) } catch (_: IllegalArgumentException) { }
            registered = false
        }
        connectionExecutor.shutdownNow()
        writeExecutor.shutdownNow()
    }

    /**
     * Many robot-side SPP servers (HC-05 modules, ESP32/RPi rfcomm servers) don't answer SDP
     * lookups correctly, which makes [BluetoothDevice.createRfcommSocketToServiceRecord] hang or
     * fail even though the device is reachable. Fall back to the hidden channel-1 socket that
     * most community Bluetooth SPP clients use for exactly this case.
     */
    @SuppressLint("MissingPermission")
    private fun openSocket(device: BluetoothDevice, attemptId: Long): BluetoothSocket {
        val standardSocket = device.createRfcommSocketToServiceRecord(SPP_UUID)
        setPendingSocket(standardSocket, attemptId)
        armConnectTimeout(attemptId)
        return try {
            standardSocket.connect()
            standardSocket
        } catch (standardError: IOException) {
            try { standardSocket.close() } catch (_: IOException) { }
            try {
                val fallback = (device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                    .invoke(device, 1) as BluetoothSocket)
                setPendingSocket(fallback, attemptId)
                fallback.connect()
                fallback
            } catch (_: Exception) {
                throw standardError
            }
        } catch (error: Exception) {
            try { standardSocket.close() } catch (_: IOException) { }
            throw IOException("Unable to open Bluetooth socket", error)
        }
    }

    /**
     * Some test tools (e.g. the AMD Tool) write a message without a trailing newline, which makes
     * a strict [BufferedReader.readLine] block forever waiting for a delimiter that never arrives.
     * Read whatever bytes are available instead: split on newlines when present (so a
     * newline-terminated protocol still works and multiple messages in one packet are separated),
     * and flush a non-terminated message after a short idle period. The idle period prevents a
     * message split across multiple Bluetooth packets from being parsed and discarded halfway
     * through.
     */
    private fun readLoop(connectedSocket: BluetoothSocket, sessionId: Long) {
        val buffer = ByteArray(1024)
        try {
            val input = connectedSocket.inputStream
            while (!closed && connectedSocket.isConnected) {
                val bytesRead = input.read(buffer)
                if (bytesRead == -1) break
                val chunk = String(buffer, 0, bytesRead, Charsets.UTF_8)
                mainHandler.post {
                    if (isCurrentSession(sessionId, connectedSocket)) consumeIncomingChunk(chunk)
                }
            }
        } catch (_: IOException) {
            // EOF and read errors both become a reconnect event below.
        } finally {
            mainHandler.post {
                if (isCurrentSession(sessionId, connectedSocket)) {
                    flushIncomingBuffer()
                    markConnectionLost(RobotMessages.BLUETOOTH_DEVICE_DISCONNECTED, sessionId, connectedSocket)
                }
            }
        }
    }

    private fun parseIncoming(line: String) {
        addRawReceived(line)
        val parts = line.split(",").map { it.trim().removePrefix("[").removeSuffix("]") }
        when (parts.firstOrNull()?.uppercase()) {
            RobotProtocol.ADD -> {
                val id = parts.getOrNull(1)?.let(::canonicalObstacleId) ?: return
                val point = parseCoordinateFrom(line) ?: return
                upsertRemoteObstacle(id, point)
            }
            RobotProtocol.SUBTRACT -> {
                val id = parts.getOrNull(1)?.let(::canonicalObstacleId) ?: return
                if (state.obstacles.any { it.id.equals(id, ignoreCase = true) }) {
                    state = state.copy(obstacles = state.obstacles.filterNot { it.id.equals(id, ignoreCase = true) })
                    addStatus(RobotMessages.obstacleRemovedRemotely(id))
                }
            }
            RobotProtocol.FACE -> {
                val id = parts.getOrNull(1)?.let(::canonicalObstacleId) ?: return
                val face = parts.getOrNull(2)?.uppercase()?.let { code -> Face.entries.firstOrNull { it.code == code } } ?: return
                if (state.obstacles.any { it.id.equals(id, ignoreCase = true) }) {
                    state = state.copy(obstacles = state.obstacles.map {
                        if (it.id.equals(id, ignoreCase = true)) it.copy(targetFace = face) else it
                    })
                    addStatus(RobotMessages.obstacleFaceSetRemotely(id, face))
                }
            }
            else -> when (val message = parseProtocolMessage(line)) {
                is ProtocolMessage.Text -> addStatus(message.text)
                is ProtocolMessage.Target -> state = state.copy(obstacles = applyTargetRecognition(state.obstacles, message))
                is ProtocolMessage.Robot -> {
                    val update = remoteRobotPose(message.x, message.y, message.direction, state.obstacles)
                    state = state.copy(robot = update.pose)
                    update.conflictingObstacle?.let { addStatus(RobotMessages.remoteRobotConflict(it)) }
                }
                null -> Unit
            }
        }
    }

    private fun parseCoordinateFrom(line: String): GridPoint? {
        val match = Regex("\\((-?\\d+)\\s*,\\s*(-?\\d+)\\)").find(line) ?: return null
        val x = match.groupValues[1].toIntOrNull() ?: return null
        val y = match.groupValues[2].toIntOrNull() ?: return null
        return GridPoint(x, y)
    }

    private fun upsertRemoteObstacle(id: String, point: GridPoint) {
        if (point.x !in 0 until MAP_COLUMNS || point.y !in 0 until MAP_ROWS) return
        if (state.robot.occupies(point.x, point.y)) {
            addStatus(RobotMessages.remoteObstacleIgnored(id, point))
            return
        }
        val exists = state.obstacles.any { it.id.equals(id, ignoreCase = true) }
        state = if (exists) {
            state.copy(obstacles = state.obstacles.map {
                if (it.id.equals(id, ignoreCase = true)) it.copy(x = point.x, y = point.y) else it
            })
        } else {
            state.copy(obstacles = state.obstacles + Obstacle(id, point.x, point.y))
        }
        addStatus(RobotMessages.remoteObstaclePlaced(id, point))
    }

    private fun handleConnectionFailure(attemptId: Long, detail: String, retry: Boolean = true) {
        mainHandler.post {
            val relevant = synchronized(socketLock) {
                connectionAttemptId == attemptId && socket == null
            }
            if (!relevant || closed) return@post
            synchronized(socketLock) {
                connectionAttemptId += 1
                activeSessionId += 1
                closeCurrentSocketsLocked()
            }
            cancelConnectTimeout()
            state = state.copy(
                connected = false,
                connectedAddress = null,
                connectionStatus = "Disconnected",
                connectionDetail = detail
            )
            addStatus(detail)
            if (retry) {
                addStatus(RobotMessages.RETRYING_CONNECTION)
                scheduleReconnect()
            }
        }
    }

    private fun markConnectionLost(detail: String, sessionId: Long, expectedSocket: BluetoothSocket) {
        val relevant = synchronized(socketLock) {
            if (activeSessionId != sessionId || socket !== expectedSocket) {
                false
            } else {
                activeSessionId += 1
                closeCurrentSocketsLocked()
                true
            }
        }
        if (!relevant || closed) return
        state = state.copy(connected = false, connectedAddress = null, connectionStatus = "Disconnected", connectionDetail = detail)
        addStatus(detail)
        scheduleReconnect()
    }

    private fun scheduleReconnect() {
        val device = lastDevice ?: return
        if (closed) return
        cancelReconnect()
        state = state.copy(connectionDetail = RobotMessages.waitingToReconnect(device.address))
        val retry = Runnable {
            if (!closed && lastDevice != null && !state.connected) {
                connect(state.selectedDevice ?: toInfo(device))
            }
        }
        reconnectAttempt += 1
        val backoffMultiplier = 1L shl (reconnectAttempt - 1).coerceAtMost(3)
        val delay = (RECONNECT_DELAY_MS * backoffMultiplier).coerceAtMost(MAX_RECONNECT_DELAY_MS)
        reconnectRunnable = retry
        mainHandler.postDelayed(retry, delay)
    }

    private fun cancelReconnect() {
        reconnectRunnable?.let(mainHandler::removeCallbacks)
        reconnectRunnable = null
    }

    private fun closeCurrentSocketsLocked() {
        try { pendingSocket?.close() } catch (_: IOException) { }
        try { socket?.close() } catch (_: IOException) { }
        pendingSocket = null
        socket = null
        output = null
    }

    private fun setPendingSocket(candidate: BluetoothSocket, attemptId: Long) {
        val accepted = synchronized(socketLock) {
            if (closed || connectionAttemptId != attemptId) {
                false
            } else {
                try { pendingSocket?.close() } catch (_: IOException) { }
                pendingSocket = candidate
                true
            }
        }
        if (!accepted) {
            try { candidate.close() } catch (_: IOException) { }
            throw IOException("Bluetooth connection attempt was superseded")
        }
    }

    private fun armConnectTimeout(attemptId: Long) {
        cancelConnectTimeout()
        val timeout = Runnable {
            val pending = synchronized(socketLock) {
                connectionAttemptId == attemptId && pendingSocket != null
            }
            if (pending) handleConnectionFailure(attemptId, RobotMessages.CONNECTION_TIMED_OUT)
        }
        connectTimeoutRunnable = timeout
        mainHandler.postDelayed(timeout, CONNECT_TIMEOUT_MS)
    }

    private fun cancelConnectTimeout() {
        connectTimeoutRunnable?.let(mainHandler::removeCallbacks)
        connectTimeoutRunnable = null
    }

    private fun finishScan(message: String) {
        cancelScanTimeout()
        if (state.scanning) state = state.copy(scanning = false)
        addStatus(message)
    }

    private fun cancelScanTimeout() {
        scanTimeoutRunnable?.let(mainHandler::removeCallbacks)
        scanTimeoutRunnable = null
    }

    private fun isCurrentSession(sessionId: Long, expectedSocket: BluetoothSocket): Boolean =
        synchronized(socketLock) { activeSessionId == sessionId && socket === expectedSocket }

    private fun consumeIncomingChunk(chunk: String) {
        incomingBuffer.append(chunk)
        while (true) {
            var delimiterIndex = -1
            for (index in 0 until incomingBuffer.length) {
                if (incomingBuffer[index] == '\n' || incomingBuffer[index] == '\r') {
                    delimiterIndex = index
                    break
                }
            }
            if (delimiterIndex < 0) break
            incomingBuffer.substring(0, delimiterIndex).trim().takeIf { it.isNotEmpty() }?.let(::parseIncoming)
            incomingBuffer.delete(0, delimiterIndex + 1)
            while (incomingBuffer.isNotEmpty() && (incomingBuffer[0] == '\n' || incomingBuffer[0] == '\r')) {
                incomingBuffer.deleteCharAt(0)
            }
        }
        if (incomingBuffer.isNotEmpty()) {
            incomingFlushRunnable?.let(mainHandler::removeCallbacks)
            val flush = Runnable { flushIncomingBuffer() }
            incomingFlushRunnable = flush
            mainHandler.postDelayed(flush, INCOMING_IDLE_FLUSH_MS)
        }
    }

    private fun flushIncomingBuffer() {
        incomingFlushRunnable = null
        incomingBuffer.toString().trim().takeIf { it.isNotEmpty() }?.let(::parseIncoming)
        incomingBuffer.clear()
    }

    private fun resetIncomingBuffer() {
        incomingFlushRunnable?.let(mainHandler::removeCallbacks)
        incomingFlushRunnable = null
        incomingBuffer.clear()
    }

    private fun registerReceiver() {
        if (registered) return
        val filter = IntentFilter().apply {
            addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED)
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        }
        ContextCompat.registerReceiver(context, receiver, filter, ContextCompat.RECEIVER_EXPORTED)
        registered = true
    }

    private fun addDevice(device: BluetoothDevice) {
        val info = try { toInfo(device) } catch (_: SecurityException) { return }
        state = state.copy(devices = mergeDevices(state.devices, listOf(info)))
    }

    @SuppressLint("MissingPermission")
    private fun toInfo(device: BluetoothDevice): BluetoothDeviceInfo = BluetoothDeviceInfo(device.address, device.name)

    private fun mergeDevices(vararg lists: List<BluetoothDeviceInfo>): List<BluetoothDeviceInfo> =
        lists.flatMap { it }.distinctBy { it.address }.sortedWith(compareBy({ it.name.isNullOrBlank() }, { it.name ?: it.address }))

    private fun hasBluetoothPermission(): Boolean = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
        ContextCompat.checkSelfPermission(context, android.Manifest.permission.BLUETOOTH_SCAN) == android.content.pm.PackageManager.PERMISSION_GRANTED &&
            ContextCompat.checkSelfPermission(context, android.Manifest.permission.BLUETOOTH_CONNECT) == android.content.pm.PackageManager.PERMISSION_GRANTED
    } else {
        ContextCompat.checkSelfPermission(context, android.Manifest.permission.ACCESS_FINE_LOCATION) == android.content.pm.PackageManager.PERMISSION_GRANTED
    }

    @Suppress("DEPRECATION")
    private fun Intent.parcelableBluetoothDevice(): BluetoothDevice? =
        if (android.os.Build.VERSION.SDK_INT >= 33) getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
        else getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
}
