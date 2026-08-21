package com.example.mdpandroid

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
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
import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
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
    val statusMessages: List<StatusMessage> = emptyList()
)

/** Classic Bluetooth SPP transport used by the AMD Tool and the robot-side serial bridge. */
class BluetoothController(private val context: Context) {
    companion object {
        private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        private const val RECONNECT_DELAY_MS = 3_000L
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val connectionExecutor = Executors.newCachedThreadPool()
    private val writeExecutor = Executors.newSingleThreadExecutor()
    private val adapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    private var socket: BluetoothSocket? = null
    private var output: OutputStream? = null
    private var lastDevice: BluetoothDevice? = null
    private var reconnectRunnable: Runnable? = null
    private var closed = false
    private var registered = false

    var state by mutableStateOf(AppState())
        private set

    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                BluetoothDevice.ACTION_FOUND -> {
                    val device = intent.parcelableBluetoothDevice() ?: return
                    addDevice(device)
                }
                BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                    state = state.copy(scanning = false)
                    addStatus("Bluetooth scan complete")
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
            addStatus("Bluetooth permission is required before scanning")
            return
        }
        val bluetoothAdapter = adapter
        if (bluetoothAdapter == null) {
            state = state.copy(connectionStatus = "Unavailable", connectionDetail = "This device has no Bluetooth adapter")
            return
        }
        try {
            if (!bluetoothAdapter.isEnabled) {
                addStatus("Turn on Bluetooth and scan again")
                return
            }
            if (bluetoothAdapter.isDiscovering) bluetoothAdapter.cancelDiscovery()
            state = state.copy(scanning = true, connectionDetail = "Scanning for nearby devices...")
            bluetoothAdapter.startDiscovery()
        } catch (_: SecurityException) {
            addStatus("Bluetooth permission denied")
        }
    }

    fun selectDevice(device: BluetoothDeviceInfo) {
        state = state.copy(selectedDevice = device, selectedDeviceName = device.name ?: device.address)
        addStatus("Selected ${device.name ?: device.address}")
    }

    fun selectObstacle(id: String) {
        if (state.obstacles.any { it.id == id }) state = state.copy(selectedObstacleId = id)
    }

    @SuppressLint("MissingPermission")
    fun connect(deviceInfo: BluetoothDeviceInfo) {
        if (!hasBluetoothPermission()) {
            addStatus("Bluetooth permission is required before connecting")
            return
        }
        val bluetoothDevice = try { adapter?.getRemoteDevice(deviceInfo.address) } catch (_: Exception) { null }
        if (bluetoothDevice == null) {
            addStatus("Could not find ${deviceInfo.address}")
            return
        }
        lastDevice = bluetoothDevice
        cancelReconnect()
        closeSocket()
        state = state.copy(
            selectedDevice = deviceInfo,
            selectedDeviceName = deviceInfo.name ?: deviceInfo.address,
            connectionStatus = "Connecting",
            connectionDetail = "Connecting to ${deviceInfo.name ?: deviceInfo.address}...",
            connected = false,
            connectedAddress = null
        )
        try { adapter?.cancelDiscovery() } catch (_: SecurityException) { }

        connectionExecutor.execute {
            try {
                val newSocket = bluetoothDevice.createRfcommSocketToServiceRecord(SPP_UUID)
                newSocket.connect()
                socket = newSocket
                output = newSocket.outputStream
                mainHandler.post {
                    state = state.copy(
                        connected = true,
                        connectedAddress = deviceInfo.address,
                        connectionStatus = "Connected",
                        connectionDetail = "Connected to ${deviceInfo.name ?: deviceInfo.address}"
                    )
                    addStatus("Bluetooth connection established")
                }
                readLoop(newSocket)
            } catch (error: IOException) {
                mainHandler.post {
                    state = state.copy(
                        connected = false,
                        connectionStatus = "Disconnected",
                        connectionDetail = "Connection failed: ${error.message ?: "device unavailable"}"
                    )
                    addStatus("Connection failed; retrying automatically")
                    scheduleReconnect()
                }
            } catch (_: SecurityException) {
                mainHandler.post {
                    addStatus("Bluetooth permission denied")
                    state = state.copy(connectionStatus = "Disconnected", connectionDetail = "Bluetooth permission denied")
                }
            }
        }
    }

    fun disconnect() {
        cancelReconnect()
        lastDevice = null
        closeSocket()
        state = state.copy(
            connected = false,
            connectedAddress = null,
            connectionStatus = "Disconnected",
            connectionDetail = "Disconnected by user"
        )
        addStatus("Bluetooth disconnected")
    }

    fun send(command: String) {
        val currentOutput = output
        if (!state.connected || currentOutput == null) {
            addStatus("Not connected; command not sent: $command")
            return
        }
        writeExecutor.execute {
            try {
                currentOutput.write((command.trim() + "\n").toByteArray(Charsets.UTF_8))
                currentOutput.flush()
                mainHandler.post { addStatus("Sent: $command") }
            } catch (_: IOException) {
                mainHandler.post { markConnectionLost("Connection lost while sending") }
            }
        }
    }

    fun addObstacleAt(point: GridPoint) {
        if (point.x !in 0 until MAP_COLUMNS || point.y !in 0 until MAP_ROWS) return
        if (state.obstacles.any { it.x == point.x && it.y == point.y }) return
        val nextNumber = (state.obstacles.mapNotNull { it.id.removePrefix("B").toIntOrNull() }.maxOrNull() ?: 0) + 1
        val obstacle = Obstacle("B$nextNumber", point.x, point.y)
        state = state.copy(obstacles = state.obstacles + obstacle, selectedObstacleId = obstacle.id)
        send("ADD,${obstacle.id},(${point.x},${point.y})")
    }

    fun moveObstacle(id: String, x: Int, y: Int) {
        val obstacle = state.obstacles.firstOrNull { it.id == id } ?: return
        if (x !in 0 until MAP_COLUMNS || y !in 0 until MAP_ROWS || state.obstacles.any { it.id != id && it.x == x && it.y == y }) {
            removeObstacle(id)
            return
        }
        state = state.copy(obstacles = state.obstacles.map { if (it.id == id) it.copy(x = x, y = y) else it }, selectedObstacleId = id)
        send("ADD,$id,($x,$y)")
        if (obstacle.x != x || obstacle.y != y) addStatus("$id placed at ($x,$y)")
    }

    fun removeObstacle(id: String) {
        if (state.obstacles.none { it.id == id }) return
        state = state.copy(obstacles = state.obstacles.filterNot { it.id == id }, selectedObstacleId = null)
        send("SUB,$id")
    }

    fun setObstacleFace(id: String, face: Face) {
        val obstacle = state.obstacles.firstOrNull { it.id == id } ?: return
        state = state.copy(obstacles = state.obstacles.map { if (it.id == id) it.copy(targetFace = face) else it }, selectedObstacleId = id)
        send("FACE,$id,${face.code},(${obstacle.x},${obstacle.y})")
    }

    fun clearObstacleTarget(id: String) {
        state = state.copy(obstacles = state.obstacles.map { if (it.id == id) it.copy(targetFace = null, targetId = null) else it })
        addStatus("Cleared target annotation for $id")
    }

    fun addStatus(message: String) {
        val timestamp = java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.getDefault()).format(java.util.Date())
        state = state.copy(statusMessages = (state.statusMessages + StatusMessage(timestamp, message)).takeLast(40))
    }

    fun deviceName(info: BluetoothDeviceInfo): String = info.name ?: info.address

    fun close() {
        closed = true
        cancelReconnect()
        closeSocket()
        if (registered) {
            try { context.unregisterReceiver(receiver) } catch (_: IllegalArgumentException) { }
            registered = false
        }
        connectionExecutor.shutdownNow()
        writeExecutor.shutdownNow()
    }

    private fun readLoop(connectedSocket: BluetoothSocket) {
        try {
            val reader = BufferedReader(InputStreamReader(connectedSocket.inputStream, Charsets.UTF_8))
            while (!closed && connectedSocket.isConnected) {
                val line = reader.readLine() ?: break
                val cleanLine = line.trim()
                if (cleanLine.isNotEmpty()) mainHandler.post { parseIncoming(cleanLine) }
            }
        } catch (_: IOException) {
            // EOF and read errors both become a reconnect event below.
        } finally {
            mainHandler.post { markConnectionLost("Bluetooth device disconnected") }
        }
    }

    private fun parseIncoming(line: String) {
        val parts = line.split(",").map { it.trim().removePrefix("[").removeSuffix("]") }
        when (parts.firstOrNull()?.uppercase()) {
            "MSG" -> parts.drop(1).joinToString(",").takeIf { it.isNotBlank() }?.let(::addStatus)
            "TARGET" -> {
                val id = parts.getOrNull(1) ?: return
                val target = parts.getOrNull(2) ?: return
                val face = parts.getOrNull(3)?.let { code -> Face.values().firstOrNull { it.code == code.uppercase() } }
                state = state.copy(obstacles = state.obstacles.map { obstacle ->
                    if (obstacle.id.equals(id, ignoreCase = true)) obstacle.copy(targetId = target, targetFace = face ?: obstacle.targetFace) else obstacle
                })
                addStatus("Target $target received for $id")
            }
            "ROBOT" -> {
                val x = parts.getOrNull(1)?.toIntOrNull() ?: return
                val y = parts.getOrNull(2)?.toIntOrNull() ?: return
                val direction = parts.getOrNull(3)?.uppercase()?.let { code -> Face.values().firstOrNull { it.code == code } } ?: return
                state = state.copy(robot = RobotState(x, y, direction))
                addStatus("Robot updated: ($x,$y) facing ${direction.code}")
            }
            else -> addStatus("Ignored unrecognised message")
        }
    }

    private fun markConnectionLost(detail: String) {
        if (!state.connected && state.connectionStatus == "Disconnected") return
        closeSocket()
        state = state.copy(connected = false, connectedAddress = null, connectionStatus = "Disconnected", connectionDetail = detail)
        addStatus(detail)
        scheduleReconnect()
    }

    private fun scheduleReconnect() {
        val device = lastDevice ?: return
        if (closed) return
        cancelReconnect()
        state = state.copy(connectionDetail = "Waiting to reconnect to ${device.address}...")
        val retry = Runnable {
            if (!closed && lastDevice != null && !state.connected) {
                connect(state.selectedDevice ?: toInfo(device))
            }
        }
        reconnectRunnable = retry
        mainHandler.postDelayed(retry, RECONNECT_DELAY_MS)
    }

    private fun cancelReconnect() {
        reconnectRunnable?.let(mainHandler::removeCallbacks)
        reconnectRunnable = null
    }

    private fun closeSocket() {
        try { socket?.close() } catch (_: IOException) { }
        socket = null
        output = null
    }

    private fun registerReceiver() {
        if (registered) return
        val filter = IntentFilter().apply {
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
