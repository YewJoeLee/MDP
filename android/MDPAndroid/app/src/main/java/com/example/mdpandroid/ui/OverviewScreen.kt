package com.example.mdpandroid.ui

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.FilterChip
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.AppState
import com.example.mdpandroid.BluetoothDeviceInfo
import com.example.mdpandroid.ui.theme.MissionTeal

@Composable
internal fun MissionOverviewCard(state: AppState, demoMode: Boolean) {
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
internal fun OverviewMetric(label: String, value: String) {
    Column {
        Text(label, style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, style = MaterialTheme.typography.bodyMedium, fontWeight = FontWeight.SemiBold)
    }
}

@Composable
internal fun ConnectionCard(
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
