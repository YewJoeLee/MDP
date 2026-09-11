package com.example.mdpandroid.ui

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.BluetoothDisabled
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.PrimaryTabRow
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRowDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.BluetoothController
import com.example.mdpandroid.BluetoothDeviceInfo
import com.example.mdpandroid.ui.theme.MissionTeal
import com.example.mdpandroid.ui.theme.SuccessContainer

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ARCMApp(
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
        val onSendArena: () -> Unit = {
            if (demoMode) controller.addStatus("Demo arena sync prepared")
            else controller.sendArenaSnapshot()
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
                onAddObstacle = controller::addObstacle,
                onMoveObstacle = controller::moveObstacle,
                onRemoveObstacle = controller::removeObstacle,
                onSelectObstacle = controller::selectObstacle,
                onSetObstacleFace = controller::setObstacleFace,
                onSetRobotStart = controller::setRobotStart,
                onSetRobotFace = controller::setRobotFace,
                onSetRobotPose = controller::setRobotPose,
                onSendArena = onSendArena,
                onClearObstacleTarget = controller::clearObstacleTarget,
                onClearObstacleSelection = controller::clearObstacleSelection
            )
            AppTab.CONTROLS -> ScreenColumn(padding) {
                ControlAvailabilityCard(enabled = demoMode || state.connected, demoMode = demoMode)
                ControlCard(enabled = demoMode || state.connected, onCommand = onCommand)
                AssessmentCommandCard(
                    enabled = demoMode || state.connected,
                    onCommand = onCommand,
                    onSendArena = onSendArena
                )
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

internal enum class AppTab(val title: String) {
    OVERVIEW("Overview"),
    ARENA("Arena"),
    CONTROLS("Controls")
}

@Composable
internal fun ConnectionIndicator(connected: Boolean, demoMode: Boolean) {
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
internal fun ScreenColumn(padding: PaddingValues, content: @Composable () -> Unit) {
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
internal fun DevicePickerDialog(
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
