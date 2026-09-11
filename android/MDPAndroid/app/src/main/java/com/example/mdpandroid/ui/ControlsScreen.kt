package com.example.mdpandroid.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowLeft
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowRight
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.KeyboardArrowUp
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.RobotCommand
import com.example.mdpandroid.ui.theme.MissionTeal

@Composable
internal fun ControlAvailabilityCard(enabled: Boolean, demoMode: Boolean) {
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
internal fun ControlCard(enabled: Boolean, onCommand: (RobotCommand) -> Unit) {
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
                "The map follows manual input and is corrected by confirmed ROBOT updates.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Spacer(Modifier.height(8.dp))
            DrivePad(enabled = enabled, onCommand = onCommand, buttonSize = 64.dp)
        }
    }
}

@Composable
internal fun ArenaDriveCard(enabled: Boolean, onCommand: (RobotCommand) -> Unit, modifier: Modifier = Modifier) {
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
internal fun DrivePad(enabled: Boolean, onCommand: (RobotCommand) -> Unit, buttonSize: Dp) {
    Column(horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(4.dp)) {
        DriveCommandButton(
            icon = Icons.Default.KeyboardArrowUp,
            contentDescription = "Move forward",
            enabled = enabled,
            buttonSize = buttonSize,
            onClick = { onCommand(RobotCommand.FORWARD) }
        )
        Row(horizontalArrangement = Arrangement.spacedBy(buttonSize / 6), verticalAlignment = Alignment.CenterVertically) {
            DriveCommandButton(
                icon = Icons.AutoMirrored.Filled.KeyboardArrowLeft,
                contentDescription = "Turn left",
                enabled = enabled,
                buttonSize = buttonSize,
                onClick = { onCommand(RobotCommand.TURN_LEFT) }
            )
            DriveCommandButton(
                icon = Icons.Default.KeyboardArrowDown,
                contentDescription = "Reverse",
                enabled = enabled,
                buttonSize = buttonSize,
                onClick = { onCommand(RobotCommand.REVERSE) }
            )
            DriveCommandButton(
                icon = Icons.AutoMirrored.Filled.KeyboardArrowRight,
                contentDescription = "Turn right",
                enabled = enabled,
                buttonSize = buttonSize,
                onClick = { onCommand(RobotCommand.TURN_RIGHT) }
            )
        }
        OutlinedButton(
            onClick = { onCommand(RobotCommand.STOP) },
            enabled = enabled,
            contentPadding = PaddingValues(horizontal = 10.dp, vertical = 0.dp)
        ) {
            Icon(Icons.Default.Stop, contentDescription = "Stop robot", modifier = Modifier.size(16.dp))
            Spacer(Modifier.width(4.dp))
            Text("Stop")
        }
    }
}

@Composable
internal fun DriveCommandButton(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    contentDescription: String,
    enabled: Boolean,
    buttonSize: Dp,
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
internal fun AssessmentCommandCard(
    enabled: Boolean,
    onCommand: (RobotCommand) -> Unit,
    onSendArena: () -> Unit
) {
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
                Button(onClick = { onCommand(RobotCommand.BEGIN_EXPLORE) }, enabled = enabled, modifier = Modifier.weight(1f)) {
                    Text("Task 1\nExplore")
                }
                Button(onClick = { onCommand(RobotCommand.BEGIN_FASTEST) }, enabled = enabled, modifier = Modifier.weight(1f)) {
                    Text("Task 2\nFastest path")
                }
            }
            OutlinedButton(onClick = onSendArena, enabled = enabled, modifier = Modifier.fillMaxWidth()) {
                Text("Send current arena to robot")
            }
        }
    }
}
