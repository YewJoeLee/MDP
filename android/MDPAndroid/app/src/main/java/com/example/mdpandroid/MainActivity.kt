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
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
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
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowLeft
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowRight
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.BluetoothDisabled
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.KeyboardArrowUp
import androidx.compose.material.icons.filled.Refresh
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
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRowDefaults
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.mutableIntStateOf
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
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.ui.theme.MDPAndroidTheme
import com.example.mdpandroid.ui.theme.MissionError
import com.example.mdpandroid.ui.theme.MissionTeal
import com.example.mdpandroid.ui.theme.ObstacleBlue
import com.example.mdpandroid.ui.theme.RobotGreen
import com.example.mdpandroid.ui.theme.SuccessContainer
import com.example.mdpandroid.ui.theme.TargetAmber
import kotlin.math.floor

internal const val MAP_COLUMNS = 20
internal const val MAP_ROWS = 20
private val SpaceXs = 4.dp
private val SpaceSm = 8.dp
private val PageGutter = 12.dp
private val CardInset = 12.dp
private val SectionGap = 12.dp
private val ArenaTopGutter = SpaceSm

class MainActivity : ComponentActivity() {
    private lateinit var bluetoothController: BluetoothController

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) {
        bluetoothController.refreshDevices()
        if (it.values.all { granted -> granted }) {
            bluetoothController.startScan()
            bluetoothController.startServerListening()
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
        val missing = permissions.filter {
            androidx.core.content.ContextCompat.checkSelfPermission(this, it) !=
                android.content.pm.PackageManager.PERMISSION_GRANTED
        }
        if (missing.isEmpty()) {
            bluetoothController.refreshDevices()
            bluetoothController.startScan()
            bluetoothController.startServerListening()
        } else {
            permissionLauncher.launch(missing.toTypedArray())
        }
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
    var selectedTab by rememberSaveable { mutableIntStateOf(0) }

    DisposableEffect(controller) {
        controller.refreshDevices()
        controller.startServerListening()
        onDispose { }
    }

    Scaffold(
        containerColor = MaterialTheme.colorScheme.background,
        topBar = {
            Column {
                TopAppBar(
                    title = {
                        Column {
                            Text("MDP Robot Console", fontWeight = FontWeight.Bold)
                            Text("Mission control and assessment workspace", fontSize = 11.sp)
                        }
                    },
                    colors = TopAppBarDefaults.topAppBarColors(
                        containerColor = MaterialTheme.colorScheme.primaryContainer,
                        titleContentColor = MaterialTheme.colorScheme.onPrimaryContainer
                    ),
                    actions = {
                        ConnectionIndicator(connected = state.connected, demoMode = demoMode)
                    }
                )
                PrimaryTabRow(
                    selectedTabIndex = selectedTab,
                    containerColor = MaterialTheme.colorScheme.primaryContainer,
                    contentColor = MaterialTheme.colorScheme.onPrimaryContainer,
                    indicator = {
                        TabRowDefaults.PrimaryIndicator(color = MaterialTheme.colorScheme.primary)
                    }
                ) {
                    AppTab.entries.forEachIndexed { index, tab ->
                        Tab(
                            selected = selectedTab == index,
                            onClick = { selectedTab = index },
                            selectedContentColor = MaterialTheme.colorScheme.primary,
                            unselectedContentColor = MaterialTheme.colorScheme.onPrimaryContainer.copy(alpha = 0.72f),
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
            AppTab.OVERVIEW -> ScreenColumn(padding) {
                MissionOverviewCard(state = state, demoMode = demoMode)
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
                RobotActivityCard(
                    statusMessages = state.statusMessages,
                    receivedRawMessages = state.receivedRawLog
                )
            }
            AppTab.ARENA -> ArenaScreen(
                padding = padding,
                state = state,
                controlsEnabled = demoMode || state.connected,
                onCommand = onCommand,
                onAddObstacle = controller::addObstacleAt,
                onMoveObstacle = controller::moveObstacle,
                onRemoveObstacle = controller::removeObstacle,
                onSelectObstacle = controller::selectObstacle,
                onSetFace = controller::setObstacleFace,
                onClearTarget = controller::clearObstacleTarget,
                onCloseFaceSelection = controller::clearObstacleSelection
            )
            AppTab.CONTROLS -> ScreenColumn(padding) {
                ControlAvailabilityCard(enabled = demoMode || state.connected, demoMode = demoMode)
                ControlCard(enabled = demoMode || state.connected, onCommand = onCommand)
                AssessmentCommandCard(enabled = demoMode || state.connected, onCommand = onCommand)
                RobotActivityCard(
                    statusMessages = state.statusMessages,
                    receivedRawMessages = state.receivedRawLog
                )
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
    OVERVIEW("Overview"),
    ARENA("Arena"),
    CONTROLS("Controls")
}

@Composable
private fun ConnectionIndicator(connected: Boolean, demoMode: Boolean) {
    val active = connected || demoMode
    Surface(
        shape = RoundedCornerShape(50),
        color = if (active) SuccessContainer else MaterialTheme.colorScheme.surface,
        modifier = Modifier.padding(end = 16.dp)
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            Icon(
                imageVector = if (active) Icons.Default.Bluetooth else Icons.Default.BluetoothDisabled,
                contentDescription = null,
                tint = if (active) MissionTeal else MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.size(16.dp)
            )
            Text(
                if (demoMode) "Demo" else if (connected) "Live" else "Offline",
                style = MaterialTheme.typography.labelMedium,
                fontWeight = FontWeight.SemiBold
            )
        }
    }
}

@Composable
private fun ScreenColumn(padding: PaddingValues, content: @Composable () -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(padding)
            .verticalScroll(rememberScrollState())
        .padding(PageGutter),
        verticalArrangement = Arrangement.spacedBy(SectionGap),
    ) {
        content()
    }
}

@Composable
private fun ArenaScreen(
    padding: PaddingValues,
    state: AppState,
    controlsEnabled: Boolean,
    onCommand: (String) -> Unit,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetFace: (String, Face) -> Unit,
    onClearTarget: (String) -> Unit,
    onCloseFaceSelection: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(padding)
            .verticalScroll(rememberScrollState())
            .padding(
                start = PageGutter,
                top = ArenaTopGutter,
                end = PageGutter,
                bottom = PageGutter
            ),
        verticalArrangement = Arrangement.spacedBy(SectionGap)
    ) {
        ArenaCard(
            state = state,
            onAddObstacle = onAddObstacle,
            onMoveObstacle = onMoveObstacle,
            onRemoveObstacle = onRemoveObstacle,
            onSelectObstacle = onSelectObstacle,
            onSetFace = onSetFace,
            onClearTarget = onClearTarget,
            onCloseFaceSelection = onCloseFaceSelection
        )
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = CardInset)
                .height(198.dp),
            horizontalArrangement = Arrangement.spacedBy(SectionGap)
        ) {
            ArenaDriveCard(
                enabled = controlsEnabled,
                onCommand = onCommand,
                modifier = Modifier.weight(0.9f)
            )
            RobotActivityCard(
                statusMessages = state.statusMessages,
                receivedRawMessages = state.receivedRawLog,
                modifier = Modifier.weight(1.3f),
                compact = true,
                fillHeight = true
            )
        }
    }
}

@Composable
private fun MissionOverviewCard(state: AppState, demoMode: Boolean) {
    val targetCount = state.obstacles.count { it.targetId != null }
    val connectionLabel = when {
        demoMode -> "Demo mode"
        state.connected -> "Robot connected"
        else -> "Robot offline"
    }
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(24.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer)
    ) {
        Column(Modifier.padding(CardInset), verticalArrangement = Arrangement.spacedBy(SectionGap)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text("Mission overview", fontWeight = FontWeight.Bold)
                    Text(connectionLabel, style = MaterialTheme.typography.bodySmall)
                }
                Text(
                    text = if (demoMode || state.connected) "READY" else "SETUP",
                    color = if (demoMode || state.connected) MissionTeal else MaterialTheme.colorScheme.error,
                    fontSize = 12.sp,
                    fontWeight = FontWeight.Bold
                )
            }
            HorizontalDivider()
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                OverviewMetric("Arena", "${state.obstacles.size} obstacles")
                OverviewMetric("Targets", "$targetCount identified")
                OverviewMetric("Robot", "(${state.robot.x}, ${state.robot.y})")
            }
            Text(
                "Prepare and observe the field in Arena. Use Controls only when you are ready to drive or launch an agreed assessment run.",
                style = MaterialTheme.typography.bodySmall
            )
        }
    }
}

@Composable
private fun OverviewMetric(label: String, value: String) {
    Column {
        Text(label, style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, style = MaterialTheme.typography.bodyMedium, fontWeight = FontWeight.SemiBold)
    }
}

@Composable
private fun ControlAvailabilityCard(enabled: Boolean, demoMode: Boolean) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(
            containerColor = if (enabled) MaterialTheme.colorScheme.secondaryContainer else MaterialTheme.colorScheme.errorContainer
        )
    ) {
        Row(
            modifier = Modifier.padding(CardInset),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column(Modifier.weight(1f)) {
                Text(if (enabled) "Controls are armed" else "Controls are unavailable", fontWeight = FontWeight.Bold)
                Text(
                    if (demoMode) "Commands will be recorded locally in demo mode."
                    else if (enabled) "Commands are sent to the active Bluetooth connection."
                    else "Connect a robot on Overview, or enable demo mode to practise.",
                    style = MaterialTheme.typography.bodySmall
                )
            }
            Text(
                if (enabled) "READY" else "OFFLINE",
                fontSize = 12.sp,
                fontWeight = FontWeight.Bold,
                color = if (enabled) MissionTeal else MaterialTheme.colorScheme.error
            )
        }
    }
}

@Composable
private fun ArenaLegendItem(color: Color, label: String) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(4.dp)) {
        Surface(color = color, shape = RoundedCornerShape(50), modifier = Modifier.size(8.dp)) {}
        Text(label, style = MaterialTheme.typography.labelSmall)
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
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, MaterialTheme.colorScheme.outlineVariant, RoundedCornerShape(20.dp)),
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)
    ) {
        Column(Modifier.padding(CardInset), verticalArrangement = Arrangement.spacedBy(SectionGap)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("Bluetooth connection", fontWeight = FontWeight.Bold, modifier = Modifier.weight(1f))
                Text(
                    text = if (demoMode) "DEMO" else if (state.connected) "CONNECTED" else state.connectionStatus.uppercase(),
                    color = if (demoMode || state.connected) MissionTeal else MaterialTheme.colorScheme.error,
                    fontSize = 12.sp,
                    fontWeight = FontWeight.Bold
                )
            }
            if (state.connected) {
                Text(
                    "Connected to ${state.selectedDeviceName ?: state.connectedAddress ?: "robot"}",
                    style = MaterialTheme.typography.bodyMedium,
                    fontWeight = FontWeight.SemiBold
                )
                Button(onClick = onDisconnect, modifier = Modifier.fillMaxWidth()) { Text("Disconnect robot") }
            } else {
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
                    OutlinedButton(
                        onClick = onScan,
                        enabled = !state.scanning && !demoMode,
                        modifier = Modifier.weight(1f)
                    ) {
                        Icon(Icons.Default.Refresh, contentDescription = null)
                        Spacer(Modifier.width(4.dp))
                        Text(if (state.scanning) "Scanning..." else "Scan")
                    }
                    OutlinedButton(
                        onClick = onSelectDevice,
                        enabled = !demoMode,
                        modifier = Modifier.weight(1f)
                    ) {
                        Text(state.selectedDeviceName ?: "Choose robot", maxLines = 1, overflow = TextOverflow.Ellipsis)
                        Icon(Icons.Default.KeyboardArrowDown, contentDescription = null)
                    }
                }
                Button(
                    onClick = { state.selectedDevice?.let(onConnect) },
                    enabled = state.selectedDevice != null && !demoMode,
                    modifier = Modifier.fillMaxWidth()
                ) { Text("Connect robot") }
            }
            Text(state.connectionDetail, style = MaterialTheme.typography.bodySmall)
        }
    }
}

@Composable
private fun RobotActivityCard(
    statusMessages: List<StatusMessage>,
    receivedRawMessages: List<StatusMessage>,
    modifier: Modifier = Modifier,
    compact: Boolean = false,
    fillHeight: Boolean = false
) {
    val logEntries = mergeActivityLog(statusMessages, receivedRawMessages)
    Card(
        modifier = modifier
            .fillMaxWidth()
            .then(if (fillHeight) Modifier.fillMaxHeight() else Modifier)
            .border(1.dp, MaterialTheme.colorScheme.outlineVariant, RoundedCornerShape(20.dp)),
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)
    ) {
        Column(Modifier.padding(CardInset), verticalArrangement = Arrangement.spacedBy(SectionGap)) {
            Text("Robot activity", fontWeight = FontWeight.Bold)
            if (!compact) {
                Text(
                    "One chronological log for run status and incoming Bluetooth text.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            if (logEntries.isEmpty()) {
                Text("No robot activity yet.", style = MaterialTheme.typography.bodySmall)
            } else {
                LazyColumn(modifier = Modifier.height(if (compact) 124.dp else 180.dp), reverseLayout = true) {
                    items(logEntries.asReversed()) { entry ->
                        Row(modifier = Modifier.padding(vertical = 2.dp), verticalAlignment = Alignment.Top) {
                            Text(
                                entry.source.name.lowercase().replaceFirstChar(Char::uppercase),
                                style = MaterialTheme.typography.labelSmall,
                                color = if (entry.source == ActivitySource.RECEIVED) MaterialTheme.colorScheme.tertiary else MaterialTheme.colorScheme.primary,
                                fontWeight = FontWeight.Bold,
                                modifier = Modifier.width(64.dp)
                            )
                            Text(
                                "${entry.message.time}  ${entry.message.text}",
                                style = MaterialTheme.typography.bodySmall,
                                modifier = Modifier.weight(1f),
                                maxLines = if (compact) 1 else Int.MAX_VALUE,
                                overflow = TextOverflow.Ellipsis
                            )
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun ControlCard(enabled: Boolean, onCommand: (String) -> Unit) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(24.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
    ) {
        Column(Modifier.padding(CardInset), horizontalAlignment = Alignment.CenterHorizontally) {
            Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text("Manual drive", fontWeight = FontWeight.Bold)
                    Text("Use for positioning and controlled testing", style = MaterialTheme.typography.bodySmall)
                }
                Text(if (enabled) "READY" else "OFFLINE", style = MaterialTheme.typography.labelSmall, fontWeight = FontWeight.Bold)
            }
            Text(
                "The map follows confirmed ROBOT updates; it does not predict a movement after a button press.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Spacer(Modifier.height(8.dp))
            DrivePad(enabled = enabled, onCommand = onCommand, buttonSize = 64.dp)
        }
    }
}

@Composable
private fun ArenaDriveCard(enabled: Boolean, onCommand: (String) -> Unit, modifier: Modifier = Modifier) {
    Card(
        modifier = modifier.fillMaxSize(),
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
    ) {
        Column(
            modifier = Modifier.padding(CardInset).fillMaxSize(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.SpaceBetween
        ) {
            Text("Manual control", style = MaterialTheme.typography.labelLarge, fontWeight = FontWeight.Bold)
            DrivePad(enabled = enabled, onCommand = onCommand, buttonSize = 42.dp)
        }
    }
}

@Composable
private fun DrivePad(enabled: Boolean, onCommand: (String) -> Unit, buttonSize: androidx.compose.ui.unit.Dp) {
    Column(horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(4.dp)) {
        DriveCommandButton(
            icon = Icons.Default.KeyboardArrowUp,
            contentDescription = "Move forward",
            enabled = enabled,
            buttonSize = buttonSize,
            onClick = { onCommand("f") }
        )
        Row(horizontalArrangement = Arrangement.spacedBy(buttonSize / 6), verticalAlignment = Alignment.CenterVertically) {
            DriveCommandButton(
                icon = Icons.AutoMirrored.Filled.KeyboardArrowLeft,
                contentDescription = "Turn left",
                enabled = enabled,
                buttonSize = buttonSize,
                onClick = { onCommand("tl") }
            )
            DriveCommandButton(
                icon = Icons.Default.KeyboardArrowDown,
                contentDescription = "Reverse",
                enabled = enabled,
                buttonSize = buttonSize,
                onClick = { onCommand("r") }
            )
            DriveCommandButton(
                icon = Icons.AutoMirrored.Filled.KeyboardArrowRight,
                contentDescription = "Turn right",
                enabled = enabled,
                buttonSize = buttonSize,
                onClick = { onCommand("tr") }
            )
        }
    }
}

@Composable
private fun DriveCommandButton(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    contentDescription: String,
    enabled: Boolean,
    buttonSize: androidx.compose.ui.unit.Dp,
    onClick: () -> Unit
) {
    Button(
        onClick = onClick,
        enabled = enabled,
        modifier = Modifier.size(buttonSize),
        shape = RoundedCornerShape(buttonSize / 3),
        contentPadding = PaddingValues(0.dp)
    ) {
        Icon(icon, contentDescription = contentDescription, modifier = Modifier.size(buttonSize * 0.56f))
    }
}

@Composable
private fun AssessmentCommandCard(enabled: Boolean, onCommand: (String) -> Unit) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer)
    ) {
        Column(Modifier.padding(CardInset), verticalArrangement = Arrangement.spacedBy(SectionGap)) {
            Text("Assessment runs", fontWeight = FontWeight.Bold)
            Text(
                "Start a run only after the team agrees the robot-side protocol and the arena is ready.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp), modifier = Modifier.fillMaxWidth()) {
                Button(onClick = { onCommand("beginExplore") }, enabled = enabled, modifier = Modifier.weight(1f)) {
                    Text("Task 1\nExplore")
                }
                Button(onClick = { onCommand("beginFastest") }, enabled = enabled, modifier = Modifier.weight(1f)) {
                    Text("Task 2\nFastest path")
                }
            }
            OutlinedButton(onClick = { onCommand("sendArena") }, enabled = enabled, modifier = Modifier.fillMaxWidth()) {
                Text("Send current arena to robot")
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
    onCloseFaceSelection: () -> Unit
) {
    val selected = state.obstacles.firstOrNull { it.id == state.selectedObstacleId }
    val preferredMapSide = 432.dp
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(24.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)
    ) {
        Column(Modifier.padding(CardInset), verticalArrangement = Arrangement.spacedBy(SectionGap)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text("Arena workspace", fontWeight = FontWeight.Bold)
                    Text("20 × 20 field • ${state.obstacles.size} obstacles placed", style = MaterialTheme.typography.bodySmall)
                }
                Text(
                    "EDIT MODE",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.primary,
                    fontWeight = FontWeight.Bold
                )
            }
            BoxWithConstraints(modifier = Modifier.fillMaxWidth()) {
                val sidePanelWidth = 128.dp
                val mapSide = minOf(
                    preferredMapSide,
                    (maxWidth - sidePanelWidth - SectionGap).coerceAtLeast(1.dp)
                )
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(SectionGap)
                ) {
                    ArenaCanvas(
                        state = state,
                        onAddObstacle = onAddObstacle,
                        onMoveObstacle = onMoveObstacle,
                        onSelectObstacle = onSelectObstacle,
                        modifier = Modifier.size(mapSide)
                    )
                    ArenaSidePanel(
                        state = state,
                        modifier = Modifier
                            .width(sidePanelWidth)
                            .height(mapSide)
                    )
                }
            }
            if (selected != null) {
                TargetFaceSelection(
                    obstacle = selected,
                    onSetFace = onSetFace,
                    onClearTarget = onClearTarget,
                    onRemoveObstacle = onRemoveObstacle,
                    onDone = onCloseFaceSelection
                )
            }
        }
    }
}

@Composable
private fun TargetFaceSelection(
    obstacle: Obstacle,
    onSetFace: (String, Face) -> Unit,
    onClearTarget: (String) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onDone: () -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, MaterialTheme.colorScheme.outlineVariant, RoundedCornerShape(16.dp)),
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)
    ) {
        BoxWithConstraints(Modifier.fillMaxWidth()) {
            if (maxWidth >= 480.dp) {
                Row(
                    modifier = Modifier.padding(horizontal = CardInset, vertical = SpaceSm),
                    horizontalArrangement = Arrangement.spacedBy(SpaceXs),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    ObstacleDetails(obstacle, modifier = Modifier.width(112.dp))
                    Text("Face", style = MaterialTheme.typography.labelSmall)
                    FaceChoices(obstacle, onSetFace)
                    if (obstacle.targetFace != null) {
                        IconButton(onClick = { onClearTarget(obstacle.id) }) {
                            Icon(Icons.Default.Refresh, contentDescription = "Clear selected face")
                        }
                    }
                    Spacer(Modifier.weight(1f))
                    IconButton(onClick = onDone) {
                        Icon(Icons.Default.Check, contentDescription = "Done selecting target face")
                    }
                    IconButton(onClick = { onRemoveObstacle(obstacle.id) }) {
                        Icon(Icons.Default.Delete, contentDescription = "Remove obstacle")
                    }
                }
            } else {
                Column(
                    modifier = Modifier.padding(CardInset),
                    verticalArrangement = Arrangement.spacedBy(SpaceXs)
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        ObstacleDetails(obstacle, modifier = Modifier.weight(1f))
                        TextButton(onClick = onDone) { Text("Done") }
                        IconButton(onClick = { onRemoveObstacle(obstacle.id) }) {
                            Icon(Icons.Default.Delete, contentDescription = "Remove obstacle")
                        }
                    }
                    Row(
                        horizontalArrangement = Arrangement.spacedBy(SpaceXs),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text("Face", style = MaterialTheme.typography.labelSmall)
                        FaceChoices(obstacle, onSetFace)
                        if (obstacle.targetFace != null) {
                            IconButton(onClick = { onClearTarget(obstacle.id) }) {
                                Icon(Icons.Default.Refresh, contentDescription = "Clear selected face")
                            }
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun ObstacleDetails(obstacle: Obstacle, modifier: Modifier = Modifier) {
    Column(modifier) {
        Text("Obstacle ${obstacle.id}", fontWeight = FontWeight.SemiBold)
        Text("(${obstacle.x}, ${obstacle.y})", style = MaterialTheme.typography.bodySmall)
    }
}

@Composable
private fun FaceChoices(obstacle: Obstacle, onSetFace: (String, Face) -> Unit) {
    Row(horizontalArrangement = Arrangement.spacedBy(SpaceXs)) {
        Face.entries.forEach { face ->
            FilterChip(
                selected = obstacle.targetFace == face,
                onClick = { onSetFace(obstacle.id, face) },
                label = { Text(face.code, fontWeight = FontWeight.Bold) }
            )
        }
    }
}

@Composable
private fun ArenaSidePanel(state: AppState, modifier: Modifier = Modifier) {
    val identifiedTargets = state.obstacles.count { it.targetId != null }
    Card(
        modifier = modifier,
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.secondaryContainer)
    ) {
        Column(
            modifier = Modifier.padding(CardInset),
            verticalArrangement = Arrangement.spacedBy(SpaceSm)
        ) {
            Column(verticalArrangement = Arrangement.spacedBy(SpaceXs)) {
                Text("Live field", style = MaterialTheme.typography.labelLarge, fontWeight = FontWeight.Bold)
                Surface(
                    shape = RoundedCornerShape(50),
                    color = if (state.connected) SuccessContainer else MaterialTheme.colorScheme.errorContainer
                ) {
                    Text(
                        if (state.connected) "LIVE" else "NO LINK",
                        modifier = Modifier.padding(horizontal = 8.dp, vertical = 5.dp),
                        style = MaterialTheme.typography.labelSmall,
                        color = if (state.connected) MissionTeal else MaterialTheme.colorScheme.error,
                        fontWeight = FontWeight.Bold
                    )
                }
            }
            HorizontalDivider()
            ArenaSideMetric("Position", "(${state.robot.x}, ${state.robot.y})")
            ArenaSideMetric("Heading", state.robot.direction.code)
            ArenaSideMetric("Obstacles", state.obstacles.size.toString())
            ArenaSideMetric("Targets", "$identifiedTargets found")
            HorizontalDivider()
            Text("Arena guide", style = MaterialTheme.typography.labelLarge, fontWeight = FontWeight.Bold)
            ArenaLegendItem(ObstacleBlue, "Blue: obstacle")
            ArenaLegendItem(TargetAmber, "Gold: target")
            ArenaLegendItem(RobotGreen, "Green: robot")
            Text(
                "Tap to add. Drag to move or remove.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }
    }
}

@Composable
private fun ArenaSideMetric(label: String, value: String, modifier: Modifier = Modifier) {
    Column(modifier) {
        Text(label, style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, style = MaterialTheme.typography.bodyMedium, fontWeight = FontWeight.SemiBold)
    }
}

@Composable
private fun ArenaCanvas(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onSelectObstacle: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    var dragId by remember { mutableStateOf<String?>(null) }
    var dragPoint by remember { mutableStateOf<GridPoint?>(null) }

    Canvas(
        modifier = modifier
            .background(Color(0xFFEAF4F5), RoundedCornerShape(16.dp))
            .border(1.dp, Color(0xFFA9BEC9), RoundedCornerShape(16.dp))
            .pointerInput(state.obstacles) {
                detectDragGestures(
                    onDragStart = { offset ->
                        val point = gridPoint(offset, size.width.toFloat(), size.height.toFloat())
                        dragId = state.obstacles.firstOrNull { it.x == point.x && it.y == point.y }?.id
                        dragPoint = point
                    },
                    onDrag = { change, _ ->
                        change.consume()
                        dragPoint = gridPoint(change.position, size.width.toFloat(), size.height.toFloat())
                    },
                    onDragEnd = {
                        val id = dragId
                        val point = dragPoint
                        if (id != null && point != null) onMoveObstacle(id, point.x, point.y)
                        dragId = null
                        dragPoint = null
                    },
                    onDragCancel = {
                        dragId = null
                        dragPoint = null
                    }
                )
            }
            .pointerInput(state.obstacles) {
                detectTapGestures { offset ->
                    val point = gridPoint(offset, size.width.toFloat(), size.height.toFloat())
                    val obstacle = state.obstacles.firstOrNull { it.x == point.x && it.y == point.y }
                    if (obstacle != null) {
                        onSelectObstacle(obstacle.id)
                    } else if (point.x in 0 until MAP_COLUMNS && point.y in 0 until MAP_ROWS) {
                        onAddObstacle(point)
                    }
                }
            }
    ) {
        val cellWidth = size.width / MAP_COLUMNS
        val cellHeight = size.height / MAP_ROWS

        for (x in 0..MAP_COLUMNS) drawLine(Color(0xFFD5E3E8), Offset(x.toFloat() * cellWidth, 0f), Offset(x.toFloat() * cellWidth, size.height), 1f)
        for (y in 0..MAP_ROWS) drawLine(Color(0xFFD5E3E8), Offset(0f, y.toFloat() * cellHeight), Offset(size.width, y.toFloat() * cellHeight), 1f)

        state.obstacles.forEach { obstacle ->
            val left = obstacle.x * cellWidth + 2f
            val top = (MAP_ROWS - 1 - obstacle.y) * cellHeight + 2f
            val obstacleWidth = cellWidth - 4f
            val obstacleHeight = cellHeight - 4f
            val right = left + obstacleWidth
            val bottom = top + obstacleHeight
            val fill = if (obstacle.targetId != null) TargetAmber else ObstacleBlue
            drawRect(fill, Offset(left, top), Size(obstacleWidth, obstacleHeight))
            drawRect(Color.White, Offset(left, top), Size(obstacleWidth, obstacleHeight), style = Stroke(2f))
            obstacle.targetFace?.let { face ->
                val faceColor = MissionError
                when (face) {
                    Face.N -> drawLine(faceColor, Offset(left, top), Offset(right, top), 5f)
                    Face.S -> drawLine(faceColor, Offset(left, bottom), Offset(right, bottom), 5f)
                    Face.W -> drawLine(faceColor, Offset(left, top), Offset(left, bottom), 5f)
                    Face.E -> drawLine(faceColor, Offset(right, top), Offset(right, bottom), 5f)
                }
            }
            drawCenteredText(
                text = obstacle.targetId ?: obstacle.id.removePrefix("B"),
                center = Offset(left + obstacleWidth / 2, top + obstacleHeight / 2),
                color = Color.White,
                textSize = cellWidth.coerceAtMost(cellHeight) * if (obstacle.targetId != null) 0.42f else 0.38f
            )
        }

        val robotLeft = state.robot.x * cellWidth
        val robotTop = (MAP_ROWS - 1 - state.robot.y) * cellHeight
        val robotCenter = Offset(robotLeft + cellWidth / 2, robotTop + cellHeight / 2)
        drawCircle(RobotGreen, cellWidth.coerceAtMost(cellHeight) * 0.34f, robotCenter)
        drawCircle(Color.White, cellWidth.coerceAtMost(cellHeight) * 0.34f, robotCenter, style = Stroke(2f))
        val direction = when (state.robot.direction) {
            Face.N -> Offset(0f, -cellHeight * 0.27f)
            Face.S -> Offset(0f, cellHeight * 0.27f)
            Face.W -> Offset(-cellWidth * 0.27f, 0f)
            Face.E -> Offset(cellWidth * 0.27f, 0f)
        }
        drawLine(Color.White, robotCenter, robotCenter + direction, 3f, cap = StrokeCap.Round)
    }
}

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawCenteredText(
    text: String,
    center: Offset,
    color: Color,
    textSize: Float
) {
    val paint = android.graphics.Paint().apply {
        this.color = color.toArgb()
        this.textSize = textSize
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
                                if (connectedAddress == device.address) Text("Connected", color = MissionTeal, fontSize = 12.sp)
                            }
                        }
                    }
                }
            }
        },
        confirmButton = { TextButton(onClick = onDismiss) { Text("Close") } }
    )
}
