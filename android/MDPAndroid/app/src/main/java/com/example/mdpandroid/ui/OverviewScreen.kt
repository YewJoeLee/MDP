package com.example.mdpandroid.ui

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
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
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.AppState
import com.example.mdpandroid.BluetoothDeviceInfo
import com.example.mdpandroid.ui.theme.*

@Composable
internal fun MissionOverviewCard(state: AppState, demoMode: Boolean) {
    val targetCount = state.obstacles.count { it.targetId != null }
    Column(verticalArrangement = Arrangement.spacedBy(SectionGap)) {
        Card(Modifier.fillMaxWidth(), shape = RoundedCornerShape(20.dp),
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.primaryContainer)) {
            Box {
                GameTexture(Modifier.matchParentSize())
                GameStairs(Modifier.align(Alignment.TopEnd).width(156.dp).height(139.dp).alpha(.2f))
                Column(Modifier.padding(20.dp), verticalArrangement = Arrangement.spacedBy(14.dp)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Column(Modifier.weight(1f)) {
                            Text("GROUP 5 / ROBOT CONTROLLER", style = MaterialTheme.typography.labelSmall, color = GamePink)
                            Text("ROBOT CONTROL", style = MaterialTheme.typography.titleLarge)
                            Text("Mission overview", style = MaterialTheme.typography.bodySmall,
                                color = MaterialTheme.colorScheme.onSurfaceVariant)
                        }
                        Surface(shape = RoundedCornerShape(12.dp), color = GamePink) {
                            Column(Modifier.padding(horizontal = 15.dp, vertical = 10.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                                Text("PLAYER", style = MaterialTheme.typography.labelSmall, color = GameNavy)
                                Text("005", fontFamily = GameDisplayFont, fontSize = 29.sp, color = GameNavy)
                            }
                        }
                    }
                    Surface(shape = RoundedCornerShape(12.dp), color = MaterialTheme.colorScheme.surface.copy(alpha = .95f)) {
                        Row(Modifier.fillMaxWidth().padding(14.dp), horizontalArrangement = Arrangement.SpaceBetween) {
                            OverviewMetric("Obstacles", state.obstacles.size.toString())
                            Column {
                                Text("Image IDs received", style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
                                Text("$targetCount / ${state.obstacles.size}", modifier = Modifier.testTag("recognition-total"),
                                    style = MaterialTheme.typography.bodyMedium, fontWeight = FontWeight.Bold, color = GameMint)
                            }
                            OverviewMetric("Robot / ${state.robot.direction.code}", "(${state.robot.x}, ${state.robot.y})")
                        }
                    }
                    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                        Text(if (demoMode) "DEMO SESSION" else if (state.connected) "LIVE SESSION" else "OFFLINE SESSION",
                            style = MaterialTheme.typography.labelSmall, fontWeight = FontWeight.Bold,
                            color = if (demoMode || state.connected) GameMint else MaterialTheme.colorScheme.error)
                        Text(if (demoMode) "Local practice · no Bluetooth commands" else "Prepare in Arena · launch in Controls",
                            style = MaterialTheme.typography.bodySmall)
                    }
                }
            }
        }
        BoxWithConstraints(Modifier.fillMaxWidth()) {
            if (maxWidth >= 560.dp) {
                Row(horizontalArrangement = Arrangement.spacedBy(SectionGap)) {
                    GameBriefing("01", "Task 1", "Image recognition · explore the arena", Modifier.weight(1f))
                    GameBriefing("02", "Task 2", "Fastest path · follow arrows and return", Modifier.weight(1f))
                }
            } else {
                Column(verticalArrangement = Arrangement.spacedBy(SpaceSm)) {
                    GameBriefing("01", "Task 1", "Image recognition · explore the arena", Modifier.fillMaxWidth())
                    GameBriefing("02", "Task 2", "Fastest path · follow arrows and return", Modifier.fillMaxWidth())
                }
            }
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
                        text = if (demoMode) "Practise locally without a robot" else "Choose a paired robot to connect",
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
            Text(if (demoMode) "Robot connection is disabled in Demo mode. Turn Demo mode off to connect." else state.connectionDetail,
                style = MaterialTheme.typography.bodySmall)
        }
    }
}
