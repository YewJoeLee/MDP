package com.example.mdpandroid

import android.Manifest
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.BluetoothDisabled
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.KeyboardArrowLeft
import androidx.compose.material.icons.filled.KeyboardArrowRight
import androidx.compose.material.icons.filled.KeyboardArrowUp
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.FilterChip
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Tab
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChanged
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.ui.theme.MDPAndroidTheme
import kotlin.math.abs
import kotlin.math.floor

internal const val MAP_COLUMNS = 20
internal const val MAP_ROWS = 20

class MainActivity : ComponentActivity() {
    private lateinit var bluetoothController: BluetoothController

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        bluetoothController.refreshDevices()
        if (result.values.all { granted -> granted }) {
            bluetoothController.startScan()
            bluetoothController.startServerListening()
        } else {
            bluetoothController.addStatus("Bluetooth permission is required before scanning")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        bluetoothController = BluetoothController(applicationContext)
        setContent {
            MDPAndroidTheme {
                ARCMApp(bluetoothController, ::requestBluetoothPermissions)
            }
        }
    }

    private fun requestBluetoothPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        permissionLauncher.launch(permissions)
    }

    override fun onDestroy() {
        if (::bluetoothController.isInitialized) bluetoothController.close()
        super.onDestroy()
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun ARCMApp(
    controller: BluetoothController,
    requestBluetoothPermissions: () -> Unit
) {
    val state = controller.state
    var showDevices by remember { mutableStateOf(false) }
    var demoMode by remember { mutableStateOf(false) }
    var selectedTab by rememberSaveable { mutableStateOf(0) }

    DisposableEffect(controller) {
        controller.refreshDevices()
        controller.startServerListening()
        onDispose { }
    }

    Scaffold(
        topBar = {
            Column {
                TopAppBar(
                    title = {
                        Column {
                            Text("MDP Remote Controller", fontWeight = FontWeight.Bold)
                            Text("Android Remote Controller Module", fontSize = 11.sp)
                        }
                    },
                    colors = TopAppBarDefaults.topAppBarColors(
                        containerColor = MaterialTheme.colorScheme.primaryContainer
                    ),
                    actions = {
                        Icon(
                            imageVector = if (state.connected) Icons.Default.Bluetooth else Icons.Default.BluetoothDisabled,
                            contentDescription = null,
                            tint = if (state.connected) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.onSurfaceVariant,
                            modifier = Modifier.padding(end = 16.dp)
                        )
                    }
                )
                PrimaryTabRow(selectedTabIndex = selectedTab) {
                    AppTab.entries.forEachIndexed { index, tab ->
                        Tab(
                            selected = selectedTab == index,
                            onClick = { selectedTab = index },
                            text = { Text(tab.title) }
                        )
                    }
                }
            }
        }
    ) { padding ->
        val onCommand: (String) -> Unit = { command ->
            controller.moveRobot(command)
            if (demoMode) controller.addStatus("Demo command: $command")
        }
        when (AppTab.entries[selectedTab]) {
            AppTab.CONTROL -> ScreenColumn(padding) {
                ConnectionCard(
                    state = state,
                    demoMode = demoMode,
                    onDemoModeChange = { demoMode = it },
                    onScan = {
                        if (demoMode) controller.addStatus("Demo mode: Bluetooth scan skipped")
                        else requestBluetoothPermissions()
                        showDevices = true
                    },
                    onConnect = controller::connect,
                    onDisconnect = controller::disconnect,
                    onSelectDevice = { showDevices = true }
                )
                StatusCard(state.statusMessages)
                ReceivedRawCard(state.receivedRawLog)
            }
            AppTab.ARENA -> ScreenColumn(padding) {
                ArenaCard(
                    state = state,
                    onAddObstacle = controller::addObstacleAt,
                    onMoveObstacle = controller::moveObstacle,
                    onRemoveObstacle = controller::removeObstacle,
                    onSelectObstacle = controller::selectObstacle,
                    onSetFace = controller::setObstacleFace,
                    onClearTarget = controller::clearObstacleTarget,
                    onSendArena = controller::sendArenaSnapshot,
                    onMoveRobot = controller::setRobotStart,
                    onSetRobotFace = controller::setRobotFace
                )
                RobotStartCard(robot = state.robot, onSetStart = controller::setRobotStart)
                ControlCard(enabled = demoMode || state.connected, onCommand = onCommand)
            }
            AppTab.MANUAL -> ScreenColumn(padding) {
                ControlCard(enabled = demoMode || state.connected, onCommand = onCommand)
                StatusCard(state.statusMessages)
                AssessmentStatusCard(state)
            }
        }
    }

    if (showDevices) {
        DevicePickerDialog(
            devices = state.devices,
            scanning = state.scanning,
            connectedAddress = state.connectedAddress,
            onDismiss = { showDevices = false },
            onScan = {
                if (demoMode) controller.addStatus("Demo mode: no Bluetooth devices")
                else requestBluetoothPermissions()
            },
            onSelect = {
                controller.selectDevice(it)
                if (!demoMode) controller.connect(it)
                showDevices = false
            },
            deviceName = controller::deviceName
        )
    }
}

private enum class AppTab(val title: String) {
    CONTROL("Control"),
    ARENA("Arena"),
    MANUAL("Manual")
}

@Composable
private fun ScreenColumn(padding: PaddingValues, content: @Composable () -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(padding)
            .verticalScroll(rememberScrollState())
        .padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        content()
    }
}

@Composable
private fun AssessmentStatusCard(state: AppState) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Assessment readiness", fontWeight = FontWeight.Bold)
            Text("Task 1  •  Automatic movement and image recognition", style = MaterialTheme.typography.bodyMedium)
            Text("Monitor MSG, TARGET, and ROBOT updates during the run.", style = MaterialTheme.typography.bodySmall)
            HorizontalDivider()
            Text("Task 2  •  Fastest car using visual recognition", style = MaterialTheme.typography.bodyMedium)
            Text("Use STOP for emergency/manual testing and monitor robot status.", style = MaterialTheme.typography.bodySmall)
            Text(
                if (state.connected) "Bluetooth link ready for live integration." else "Connect Bluetooth before live assessment.",
                color = if (state.connected) Color(0xFF197A43) else MaterialTheme.colorScheme.error,
                fontWeight = FontWeight.SemiBold,
                style = MaterialTheme.typography.bodySmall
            )
        }
    }
}

@Composable
private fun ConnectionCard(
    state: AppState,
    demoMode: Boolean,
    onDemoModeChange: (Boolean) -> Unit,
    onScan: () -> Unit,
    onConnect: (BluetoothDeviceInfo) -> Unit,
    onDisconnect: () -> Unit,
    onSelectDevice: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
    ) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("Bluetooth connection", fontWeight = FontWeight.Bold, modifier = Modifier.weight(1f))
                Text(
                    text = if (demoMode) "DEMO" else if (state.connected) "CONNECTED" else state.connectionStatus.uppercase(),
                    color = if (demoMode || state.connected) Color(0xFF197A43) else MaterialTheme.colorScheme.error,
                    fontSize = 12.sp,
                    fontWeight = FontWeight.Bold
                )
            }
            Row(verticalAlignment = Alignment.CenterVertically) {
                FilterChip(
                    selected = demoMode,
                    onClick = { onDemoModeChange(!demoMode) },
                    label = { Text("Demo mode") }
                )
                Spacer(Modifier.width(8.dp))
                Text(
                    text = if (demoMode) "Test the map and protocol without hardware" else "Use AMD Tool or the robot SPP device",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
                OutlinedButton(onClick = onScan, enabled = !state.scanning && !demoMode) {
                    Icon(Icons.Default.Refresh, contentDescription = null)
                    Spacer(Modifier.width(4.dp))
                    Text(if (state.scanning) "Scanning..." else "Scan")
                }
                OutlinedButton(onClick = onSelectDevice, enabled = state.devices.isNotEmpty() && !demoMode) {
                    Text(state.selectedDeviceName ?: "Select device", maxLines = 1, overflow = TextOverflow.Ellipsis)
                    Icon(Icons.Default.KeyboardArrowDown, contentDescription = null)
                }
                if (state.connected) {
                    Button(onClick = onDisconnect) { Text("Disconnect") }
                } else {
                    Button(
                        onClick = { state.selectedDevice?.let(onConnect) },
                        enabled = state.selectedDevice != null && !demoMode
                    ) { Text("Connect") }
                }
            }
            Text(state.connectionDetail, style = MaterialTheme.typography.bodySmall)
        }
    }
}

@Composable
private fun ReceivedRawCard(messages: List<StatusMessage>) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
    ) {
        Column(Modifier.padding(12.dp)) {
            Text("Received text (raw, C.1 evidence)", fontWeight = FontWeight.Bold)
            Spacer(Modifier.height(6.dp))
            if (messages.isEmpty()) {
                Text("No raw text received yet.", style = MaterialTheme.typography.bodySmall)
            } else {
                LazyColumn(modifier = Modifier.height(160.dp), reverseLayout = true) {
                    items(messages.asReversed()) { message ->
                        Text("${message.time}  ${message.text}", style = MaterialTheme.typography.bodySmall, modifier = Modifier.padding(vertical = 2.dp))
                    }
                }
            }
        }
    }
}

@Composable
private fun ControlCard(enabled: Boolean, onCommand: (String) -> Unit) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp), horizontalAlignment = Alignment.CenterHorizontally) {
            Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                Text("Robot controls", fontWeight = FontWeight.Bold, modifier = Modifier.weight(1f))
                Text(if (enabled) "Ready" else "Connect or enable demo mode", style = MaterialTheme.typography.bodySmall)
            }
            Spacer(Modifier.height(8.dp))
            IconButton(onClick = { onCommand("f") }, enabled = enabled, modifier = Modifier.size(52.dp)) {
                Icon(Icons.Default.KeyboardArrowUp, contentDescription = "Forward", modifier = Modifier.size(36.dp))
            }
            Row(verticalAlignment = Alignment.CenterVertically) {
                IconButton(onClick = { onCommand("tl") }, enabled = enabled, modifier = Modifier.size(52.dp)) {
                    Icon(Icons.Default.KeyboardArrowLeft, contentDescription = "Left", modifier = Modifier.size(36.dp))
                }
                Button(onClick = { onCommand("STOP") }, enabled = enabled, modifier = Modifier.size(76.dp, 48.dp), contentPadding = PaddingValues(0.dp)) {
                    Icon(Icons.Default.Stop, contentDescription = "Stop")
                }
                IconButton(onClick = { onCommand("tr") }, enabled = enabled, modifier = Modifier.size(52.dp)) {
                    Icon(Icons.Default.KeyboardArrowRight, contentDescription = "Right", modifier = Modifier.size(36.dp))
                }
            }
            IconButton(onClick = { onCommand("r") }, enabled = enabled, modifier = Modifier.size(52.dp)) {
                Icon(Icons.Default.KeyboardArrowDown, contentDescription = "Backward", modifier = Modifier.size(36.dp))
            }
        }
    }
}

@Composable
private fun ArenaCard(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetFace: (String, Face) -> Unit,
    onClearTarget: (String) -> Unit,
    onSendArena: () -> Unit,
    onMoveRobot: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit
) {
    val selected = state.obstacles.firstOrNull { it.id == state.selectedObstacleId }
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("Arena map", fontWeight = FontWeight.Bold, modifier = Modifier.weight(1f))
                Text("Tap empty cell to add obstacle", style = MaterialTheme.typography.bodySmall)
            }
            ArenaGrid(
                state = state,
                onAddObstacle = onAddObstacle,
                onMoveObstacle = onMoveObstacle,
                onSelectObstacle = onSelectObstacle,
                onSetFace = onSetFace,
                onMoveRobot = onMoveRobot,
                onSetRobotFace = onSetRobotFace
            )
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    "Drag an obstacle or the robot a short nudge to set its facing side, further to move it, or (obstacles only) off the arena to remove it.",
                    style = MaterialTheme.typography.bodySmall,
                    modifier = Modifier.weight(1f)
                )
                OutlinedButton(onClick = onSendArena) { Text("Send arena") }
            }
            if (selected != null) {
                HorizontalDivider()
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text("Selected ${selected.id} at (${selected.x}, ${selected.y})", fontWeight = FontWeight.SemiBold, modifier = Modifier.weight(1f))
                    IconButton(onClick = { onRemoveObstacle(selected.id) }) {
                        Icon(Icons.Default.Delete, contentDescription = "Remove obstacle")
                    }
                }
                Text("Touch a face to annotate the target image (or nudge the obstacle on the map):", style = MaterialTheme.typography.bodySmall)
                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    Face.values().forEach { face ->
                        FilterChip(
                            selected = selected.targetFace == face,
                            onClick = { onSetFace(selected.id, face) },
                            label = { Text(face.code) }
                        )
                    }
                    if (selected.targetFace != null) {
                        TextButton(onClick = { onClearTarget(selected.id) }) { Text("Clear") }
                    }
                }
            }
        }
    }
}

/** Wraps [ArenaCanvas] with row/column index labels (0..19) along the left and bottom edges. */
@Composable
private fun ArenaGrid(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetFace: (String, Face) -> Unit,
    onMoveRobot: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit
) {
    val gutter = 16.dp
    val maxCanvasSpan = 360.dp
    val labelStyle = MaterialTheme.typography.bodySmall.copy(fontSize = 8.sp)
    BoxWithConstraints(modifier = Modifier.fillMaxWidth()) {
        // On a wide/tablet screen, filling the full available width would make the canvas
        // (forced to a 1:1 aspect ratio) taller than the viewport, which pushes the send/robot
        // start/control cards far below the fold and makes the grid itself hard to scroll past,
        // since its own drag gesture swallows swipes that land on it. Cap the span instead.
        val cellSize = minOf(maxWidth - gutter, maxCanvasSpan) / MAP_COLUMNS
        val canvasSize = cellSize * MAP_COLUMNS
        Column {
            Row {
                Column(modifier = Modifier.width(gutter).height(canvasSize)) {
                    for (y in MAP_ROWS - 1 downTo 0) {
                        Text(
                            text = "$y",
                            style = labelStyle,
                            textAlign = TextAlign.Center,
                            modifier = Modifier.height(cellSize).fillMaxWidth()
                        )
                    }
                }
                ArenaCanvas(
                    state = state,
                    onAddObstacle = onAddObstacle,
                    onMoveObstacle = onMoveObstacle,
                    onSelectObstacle = onSelectObstacle,
                    onSetFace = onSetFace,
                    onMoveRobot = onMoveRobot,
                    onSetRobotFace = onSetRobotFace,
                    modifier = Modifier.size(canvasSize)
                )
            }
            Row(modifier = Modifier.padding(start = gutter)) {
                for (x in 0 until MAP_COLUMNS) {
                    Text(
                        text = "$x",
                        style = labelStyle,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(cellSize)
                    )
                }
            }
        }
    }
}

@Composable
private fun RobotStartCard(robot: RobotState, onSetStart: (Int, Int) -> Unit) {
    var xText by remember(robot.x) { mutableStateOf(robot.x.toString()) }
    var yText by remember(robot.y) { mutableStateOf(robot.y.toString()) }
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Robot start position", fontWeight = FontWeight.Bold)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
                OutlinedTextField(
                    value = xText,
                    onValueChange = { xText = it.filter(Char::isDigit).take(2) },
                    label = { Text("X") },
                    singleLine = true,
                    modifier = Modifier.weight(1f)
                )
                OutlinedTextField(
                    value = yText,
                    onValueChange = { yText = it.filter(Char::isDigit).take(2) },
                    label = { Text("Y") },
                    singleLine = true,
                    modifier = Modifier.weight(1f)
                )
                Button(onClick = {
                    val x = xText.toIntOrNull()
                    val y = yText.toIntOrNull()
                    if (x != null && y != null) onSetStart(x, y)
                }) { Text("Set") }
            }
        }
    }
}

private enum class DragKind { OBSTACLE, ROBOT }

/**
 * A single obstacle or the robot can be tapped (select / add-obstacle-on-empty-cell), dragged a
 * short distance and released back near its own cell (sets its facing side from the drag
 * direction), or dragged further (moves it to the release cell; off-grid removes an obstacle,
 * or clamps the robot back onto the grid).
 *
 * This used to be two separate `pointerInput` blocks (one `detectDragGestures`, one
 * `detectTapGestures`). Two independent gesture detectors racing over the same touch stream on a
 * ~18dp cell is smaller than the default touch-slop distance, so by the time `onDragStart` fired
 * its reported offset had already drifted into a neighbouring (usually empty) cell, and the drag
 * silently found nothing to move. A single combined gesture loop that reads the raw pointer-down
 * position directly avoids that drift entirely.
 */
@Composable
private fun ArenaCanvas(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetFace: (String, Face) -> Unit,
    onMoveRobot: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    modifier: Modifier = Modifier
) {
    var dragKind by remember { mutableStateOf<DragKind?>(null) }
    var dragObstacleId by remember { mutableStateOf<String?>(null) }
    var dragOffset by remember { mutableStateOf<Offset?>(null) }

    Canvas(
        modifier = modifier
            .aspectRatio(MAP_COLUMNS.toFloat() / MAP_ROWS.toFloat())
            .background(Color(0xFFE8F3F8), RoundedCornerShape(8.dp))
            .border(1.dp, Color(0xFF7A9AA8), RoundedCornerShape(8.dp))
            .pointerInput(state.obstacles, state.robot.x, state.robot.y) {
                val cellSizePx = minOf(size.width, size.height) / MAP_COLUMNS.toFloat()
                val jitterPx = 6.dp.toPx()
                val nudgeLimitPx = cellSizePx * 0.55f
                awaitEachGesture {
                    val down = awaitFirstDown()
                    val downOffset = down.position
                    val downPoint = gridPoint(downOffset, size.width.toFloat(), size.height.toFloat())
                    val hitObstacleId = state.obstacles.firstOrNull { it.x == downPoint.x && it.y == downPoint.y }?.id
                    val hitRobot = hitObstacleId == null && downPoint.x == state.robot.x && downPoint.y == state.robot.y
                    var currentOffset = downOffset
                    var moved = false

                    while (true) {
                        val event = awaitPointerEvent()
                        val change = event.changes.firstOrNull { it.id == down.id } ?: break
                        if (change.positionChanged()) {
                            change.consume()
                            currentOffset = change.position
                            moved = true
                            when {
                                hitObstacleId != null -> {
                                    dragKind = DragKind.OBSTACLE
                                    dragObstacleId = hitObstacleId
                                    dragOffset = currentOffset
                                }
                                hitRobot -> {
                                    dragKind = DragKind.ROBOT
                                    dragOffset = currentOffset
                                }
                            }
                        }
                        if (!change.pressed) break
                    }

                    dragKind = null
                    dragObstacleId = null
                    dragOffset = null

                    val delta = currentOffset - downOffset
                    val distance = delta.getDistance()
                    val releasePoint = gridPoint(currentOffset, size.width.toFloat(), size.height.toFloat())

                    when {
                        !moved || distance < jitterPx -> when {
                            hitObstacleId != null -> onSelectObstacle(hitObstacleId)
                            hitRobot -> {}
                            downPoint.x in 0 until MAP_COLUMNS && downPoint.y in 0 until MAP_ROWS -> onAddObstacle(downPoint)
                        }
                        hitObstacleId != null -> if (distance < nudgeLimitPx) {
                            onSetFace(hitObstacleId, dominantFace(delta))
                        } else {
                            onMoveObstacle(hitObstacleId, releasePoint.x, releasePoint.y)
                        }
                        hitRobot -> if (distance < nudgeLimitPx) {
                            onSetRobotFace(dominantFace(delta))
                        } else {
                            onMoveRobot(releasePoint.x.coerceIn(0, MAP_COLUMNS - 1), releasePoint.y.coerceIn(0, MAP_ROWS - 1))
                        }
                    }
                }
            }
    ) {
        val cellWidth = size.width / MAP_COLUMNS
        val cellHeight = size.height / MAP_ROWS

        for (x in 0..MAP_COLUMNS) drawLine(Color(0xFFB7D1DB), Offset(x.toFloat() * cellWidth, 0f), Offset(x.toFloat() * cellWidth, size.height), 1f)
        for (y in 0..MAP_ROWS) drawLine(Color(0xFFB7D1DB), Offset(0f, y.toFloat() * cellHeight), Offset(size.width, y.toFloat() * cellHeight), 1f)

        state.obstacles.forEach { obstacle ->
            if (dragKind == DragKind.OBSTACLE && dragObstacleId == obstacle.id) return@forEach
            val left = obstacle.x * cellWidth + 2f
            val top = (MAP_ROWS - 1 - obstacle.y) * cellHeight + 2f
            val fill = if (obstacle.targetId != null) Color(0xFFD29B42) else Color(0xFF417A9D)
            drawRect(fill, Offset(left, top), Size(cellWidth - 4f, cellHeight - 4f))
            drawRect(Color.White, Offset(left, top), Size(cellWidth - 4f, cellHeight - 4f), style = Stroke(2f))
            obstacle.targetFace?.let { face ->
                val faceColor = Color(0xFFE33B35)
                when (face) {
                    Face.N -> drawLine(faceColor, Offset(left, top), Offset(left + cellWidth, top), 5f)
                    Face.S -> drawLine(faceColor, Offset(left, top + cellHeight - 4f), Offset(left + cellWidth, top + cellHeight - 4f), 5f)
                    Face.W -> drawLine(faceColor, Offset(left, top), Offset(left, top + cellHeight), 5f)
                    Face.E -> drawLine(faceColor, Offset(left + cellWidth - 4f, top), Offset(left + cellWidth - 4f, top + cellHeight), 5f)
                }
            }
            drawCenteredText(
                text = obstacle.targetId ?: obstacle.id.removePrefix("B"),
                center = Offset(left + cellWidth / 2, top + cellHeight / 2),
                color = Color.White,
                textSize = if (obstacle.targetId != null) 18f else 11f
            )
        }

        if (dragKind != DragKind.ROBOT) {
            val robotLeft = state.robot.x * cellWidth
            val robotTop = (MAP_ROWS - 1 - state.robot.y) * cellHeight
            val robotCenter = Offset(robotLeft + cellWidth / 2, robotTop + cellHeight / 2)
            drawCircle(Color(0xFF1B9E77), cellWidth.coerceAtMost(cellHeight) * 0.34f, robotCenter)
            drawCircle(Color.White, cellWidth.coerceAtMost(cellHeight) * 0.34f, robotCenter, style = Stroke(2f))
            val direction = when (state.robot.direction) {
                Face.N -> Offset(0f, -cellHeight * 0.27f)
                Face.S -> Offset(0f, cellHeight * 0.27f)
                Face.W -> Offset(-cellWidth * 0.27f, 0f)
                Face.E -> Offset(cellWidth * 0.27f, 0f)
            }
            drawLine(Color.White, robotCenter, robotCenter + direction, 3f, cap = StrokeCap.Round)
        }

        // Live preview: whatever is being dragged follows the finger instead of its committed position.
        val liveOffset = dragOffset
        when (dragKind) {
            DragKind.OBSTACLE -> {
                val obstacle = state.obstacles.firstOrNull { it.id == dragObstacleId }
                if (obstacle != null && liveOffset != null) {
                    val point = gridPoint(liveOffset, size.width, size.height)
                    val outOfBounds = point.x !in 0 until MAP_COLUMNS || point.y !in 0 until MAP_ROWS
                    val fill = when {
                        outOfBounds -> Color(0xFFD1453B)
                        obstacle.targetId != null -> Color(0xFFD29B42)
                        else -> Color(0xFF417A9D)
                    }
                    drawRect(
                        fill.copy(alpha = 0.85f),
                        Offset(liveOffset.x - cellWidth / 2 + 2f, liveOffset.y - cellHeight / 2 + 2f),
                        Size(cellWidth - 4f, cellHeight - 4f)
                    )
                    drawCenteredText(text = obstacle.id.removePrefix("B"), center = liveOffset, color = Color.White, textSize = 11f)
                }
            }
            DragKind.ROBOT -> if (liveOffset != null) {
                val radius = cellWidth.coerceAtMost(cellHeight) * 0.34f
                drawCircle(Color(0xFF1B9E77).copy(alpha = 0.85f), radius, liveOffset)
                drawCircle(Color.White, radius, liveOffset, style = Stroke(2f))
            }
            null -> {}
        }
    }
}

/** Compass direction whose axis dominates a screen-space drag vector (screen-down = grid South, since grid y increases upward). */
private fun dominantFace(delta: Offset): Face = if (abs(delta.x) > abs(delta.y)) {
    if (delta.x > 0) Face.E else Face.W
} else {
    if (delta.y > 0) Face.S else Face.N
}

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawCenteredText(
    text: String,
    center: Offset,
    color: Color,
    textSize: Float
) {
    val paint = android.graphics.Paint().apply {
        this.color = color.toArgb()
        this.textSize = textSize * density
        textAlign = android.graphics.Paint.Align.CENTER
        isAntiAlias = true
        typeface = android.graphics.Typeface.DEFAULT_BOLD
    }
    drawContext.canvas.nativeCanvas.drawText(text, center.x, center.y - (paint.ascent() + paint.descent()) / 2, paint)
}

private fun gridPoint(offset: Offset, width: Float, height: Float): GridPoint {
    val x = floor(offset.x / (width / MAP_COLUMNS)).toInt()
    val yFromTop = floor(offset.y / (height / MAP_ROWS)).toInt()
    return GridPoint(x, MAP_ROWS - 1 - yFromTop)
}

@Composable
private fun StatusCard(messages: List<StatusMessage>) {
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp)) {
            Text("Robot status", fontWeight = FontWeight.Bold)
            Spacer(Modifier.height(6.dp))
            if (messages.isEmpty()) {
                Text("No status messages received yet.", style = MaterialTheme.typography.bodySmall)
            } else {
                LazyColumn(modifier = Modifier.height(120.dp), reverseLayout = true) {
                    items(messages.asReversed()) { message ->
                        Text("${message.time}  ${message.text}", style = MaterialTheme.typography.bodySmall, modifier = Modifier.padding(vertical = 2.dp))
                    }
                }
            }
        }
    }
}

@Composable
private fun DevicePickerDialog(
    devices: List<BluetoothDeviceInfo>,
    scanning: Boolean,
    connectedAddress: String?,
    onDismiss: () -> Unit,
    onScan: () -> Unit,
    onSelect: (BluetoothDeviceInfo) -> Unit,
    deviceName: (BluetoothDeviceInfo) -> String
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Select Bluetooth device") },
        text = {
            Column {
                Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                    Text(if (scanning) "Scanning for devices..." else "Paired and discovered devices", modifier = Modifier.weight(1f))
                    IconButton(onClick = onScan, enabled = !scanning) { Icon(Icons.Default.Refresh, contentDescription = "Scan") }
                }
                if (devices.isEmpty()) {
                    Text("No devices found. Pair the robot or AMD Tool in Android settings, then scan again.")
                } else {
                    LazyColumn(modifier = Modifier.height(220.dp)) {
                        items(devices, key = { it.address }) { device ->
                            Row(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .clickable { onSelect(device) }
                                    .padding(vertical = 10.dp),
                                verticalAlignment = Alignment.CenterVertically
                            ) {
                                Column(Modifier.weight(1f)) {
                                    Text(deviceName(device), fontWeight = FontWeight.SemiBold)
                                    Text(device.address, style = MaterialTheme.typography.bodySmall)
                                }
                                if (connectedAddress == device.address) Text("Connected", color = Color(0xFF197A43), fontSize = 12.sp)
                            }
                        }
                    }
                }
            }
        },
        confirmButton = { TextButton(onClick = onDismiss) { Text("Close") } }
    )
}
