package com.example.mdpandroid.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.FilterChip
import androidx.compose.material3.FilterChipDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChanged
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.AppState
import com.example.mdpandroid.Face
import com.example.mdpandroid.GridPoint
import com.example.mdpandroid.MAP_COLUMNS
import com.example.mdpandroid.MAP_ROWS
import com.example.mdpandroid.Obstacle
import com.example.mdpandroid.ROBOT_FOOTPRINT_RADIUS
import com.example.mdpandroid.RobotState
import com.example.mdpandroid.RobotCommand
import com.example.mdpandroid.faceFromDrag
import com.example.mdpandroid.gridPoint
import com.example.mdpandroid.occupies
import com.example.mdpandroid.ui.theme.MissionError
import com.example.mdpandroid.ui.theme.MissionTeal
import com.example.mdpandroid.ui.theme.ObstacleBlue
import com.example.mdpandroid.ui.theme.RobotGreen
import com.example.mdpandroid.ui.theme.SuccessContainer
import com.example.mdpandroid.ui.theme.TargetAmber

@Composable
internal fun ArenaScreen(
    padding: PaddingValues,
    state: AppState,
    controlsEnabled: Boolean,
    onCommand: (RobotCommand) -> Unit,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    onSetRobotPose: (Int, Int, Face) -> Unit,
    onSendArena: () -> Unit,
    onClearObstacleTarget: (String) -> Unit,
    onClearObstacleSelection: () -> Unit
) {
    // A landscape tablet is wider than it is tall: lay the workspace out side-by-side so the
    // whole tab (grid, robot start, drive pad, activity log) fits in view without scrolling,
    // instead of stacking everything into a column taller than the screen.
    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .padding(padding)
    ) {
        if (maxWidth > maxHeight) {
            ArenaScreenLandscape(
                state = state,
                onAddObstacle = onAddObstacle,
                onMoveObstacle = onMoveObstacle,
                onRemoveObstacle = onRemoveObstacle,
                onSelectObstacle = onSelectObstacle,
                onSetObstacleFace = onSetObstacleFace,
                onSetRobotStart = onSetRobotStart,
                onSetRobotFace = onSetRobotFace,
                onSetRobotPose = onSetRobotPose,
                onSendArena = onSendArena,
                onClearObstacleTarget = onClearObstacleTarget,
                onClearObstacleSelection = onClearObstacleSelection
            )
        } else {
            ArenaScreenPortrait(
                state = state,
                controlsEnabled = controlsEnabled,
                onCommand = onCommand,
                onAddObstacle = onAddObstacle,
                onMoveObstacle = onMoveObstacle,
                onRemoveObstacle = onRemoveObstacle,
                onSelectObstacle = onSelectObstacle,
                onSetObstacleFace = onSetObstacleFace,
                onSetRobotStart = onSetRobotStart,
                onSetRobotFace = onSetRobotFace,
                onSetRobotPose = onSetRobotPose,
                onSendArena = onSendArena,
                onClearObstacleTarget = onClearObstacleTarget,
                onClearObstacleSelection = onClearObstacleSelection
            )
        }
    }
}

@Composable
private fun ArenaScreenPortrait(
    state: AppState,
    controlsEnabled: Boolean,
    onCommand: (RobotCommand) -> Unit,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    onSetRobotPose: (Int, Int, Face) -> Unit,
    onSendArena: () -> Unit,
    onClearObstacleTarget: (String) -> Unit,
    onClearObstacleSelection: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
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
            onSetObstacleFace = onSetObstacleFace,
            onSetRobotStart = onSetRobotStart,
            onSetRobotFace = onSetRobotFace,
            onSetRobotPose = onSetRobotPose,
            onSendArena = onSendArena,
            onClearObstacleTarget = onClearObstacleTarget,
            onClearObstacleSelection = onClearObstacleSelection
        )
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = CardInset),
            verticalArrangement = Arrangement.spacedBy(SpaceSm)
        ) {
            ManualControlSection(enabled = controlsEnabled, onCommand = onCommand)
            RobotActivityCard(
                statusMessages = state.statusMessages,
                receivedRawMessages = state.receivedRawLog,
                modifier = Modifier.fillMaxWidth(),
                boxed = false
            )
        }
    }
}

@Composable
private fun ArenaScreenLandscape(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    onSetRobotPose: (Int, Int, Face) -> Unit,
    onSendArena: () -> Unit,
    onClearObstacleTarget: (String) -> Unit,
    onClearObstacleSelection: () -> Unit
) {
    val selected = state.obstacles.firstOrNull { it.id == state.selectedObstacleId }
    // Flush to the screen's own dimensions — no outer margin — so the map claims every bit of
    // available space instead of floating inset from the device edges.
    Row(
        modifier = Modifier.fillMaxSize(),
        horizontalArrangement = Arrangement.spacedBy(SectionGap)
    ) {
        ArenaWorkspaceCard(
            state = state,
            onAddObstacle = onAddObstacle,
            onMoveObstacle = onMoveObstacle,
            onSelectObstacle = onSelectObstacle,
            onSetObstacleFace = onSetObstacleFace,
            onSetRobotStart = onSetRobotStart,
            onSetRobotFace = onSetRobotFace,
            onSendArena = onSendArena,
            modifier = Modifier
                .weight(2.4f)
                .fillMaxHeight()
        )
        // A narrow fixed width so the map (above) claims the majority of the screen. Below
        // TargetFaceSelection's and RobotStartCard's 480dp/560dp breakpoints, so they render
        // their stacked (not single-row) layouts — still compact, just taller.
        //
        // No scroll here: TargetFaceSelection and RobotStartCard size to their content, and the
        // drive pad / activity row below takes weight(1f) to soak up whatever height remains, so
        // this column always fits the fixed fillMaxHeight() exactly instead of overflowing it.
        // (verticalScroll + weight() cannot be combined on the same axis in Compose.)
        Column(
            modifier = Modifier
                .weight(1f)
                .fillMaxHeight()
                .padding(top = ArenaTopGutter, end = PageGutter, bottom = PageGutter),
            verticalArrangement = Arrangement.spacedBy(SectionGap)
        ) {
            if (selected != null) {
                TargetFaceSelection(
                    obstacle = selected,
                    onSetObstacleFace = onSetObstacleFace,
                    onClearObstacleTarget = onClearObstacleTarget,
                    onRemoveObstacle = onRemoveObstacle,
                    onDone = onClearObstacleSelection
                )
            }
            RobotStartCard(robot = state.robot, onSetPose = onSetRobotPose)
            RobotActivityCard(
                statusMessages = state.statusMessages,
                receivedRawMessages = state.receivedRawLog,
                modifier = Modifier
                    .fillMaxWidth()
                    .weight(1f),
                compact = true,
                fillHeight = true,
                boxed = false
            )
        }
    }
}

/** Grid-only variant of [ArenaCard] for the landscape layout: sized to fit the available height
 * (not just width) so the whole Arena tab fits on screen without scrolling. */
@Composable
private fun ArenaWorkspaceCard(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    onSendArena: () -> Unit,
    modifier: Modifier = Modifier
) {
    // No card/background — sits directly on the page so the grid can claim the maximum space.
    Row(
        modifier = modifier.fillMaxSize(),
        horizontalArrangement = Arrangement.spacedBy(SpaceSm)
    ) {
        ArenaStatusSidebar(state = state, modifier = Modifier.fillMaxHeight())
        Column(
            modifier = Modifier.weight(1f).fillMaxHeight(),
            verticalArrangement = Arrangement.spacedBy(0.dp)
        ) {
            // Its own row above the grid — outside the map — rather than floating on top of it.
            // No gap to the grid below: TextButton's own 40dp minimum height (much taller than
            // the pill) was what made that gap, not the spacing between them.
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(SpaceXs, Alignment.End),
                verticalAlignment = Alignment.CenterVertically
            ) {
                ConnectionPill(connected = state.connected)
                Surface(
                    shape = RoundedCornerShape(50),
                    color = MaterialTheme.colorScheme.secondaryContainer
                ) {
                    TextButton(
                        onClick = onSendArena,
                        modifier = Modifier.height(28.dp),
                        contentPadding = PaddingValues(horizontal = 10.dp, vertical = 0.dp)
                    ) {
                        Text("SEND ARENA", style = MaterialTheme.typography.labelSmall.copy(fontSize = 9.sp))
                    }
                }
            }
            ArenaGrid(
                state = state,
                onAddObstacle = onAddObstacle,
                onMoveObstacle = onMoveObstacle,
                onSelectObstacle = onSelectObstacle,
                onSetObstacleFace = onSetObstacleFace,
                onSetRobotStart = onSetRobotStart,
                onSetRobotFace = onSetRobotFace,
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
            )
        }
    }
}

/** The vertical counterpart of [ArenaStatusStrip] plus the legend, stacked in a narrow, centered
 * sidebar to the left of the map instead of a horizontal strip above it, so the grid can use the
 * full height. */
@Composable
private fun ArenaStatusSidebar(state: AppState, modifier: Modifier = Modifier) {
    val identifiedTargets = state.obstacles.count { it.targetId != null }
    Surface(
        modifier = modifier.width(96.dp),
        shape = RoundedCornerShape(18.dp),
        color = MaterialTheme.colorScheme.secondaryContainer
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 8.dp, vertical = 12.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(SpaceSm)
        ) {
            ArenaMetric("Position", "(${state.robot.x}, ${state.robot.y})", modifier = Modifier.fillMaxWidth())
            ArenaMetric("Heading", state.robot.direction.code, modifier = Modifier.fillMaxWidth())
            ArenaMetric("Obstacles", state.obstacles.size.toString(), modifier = Modifier.fillMaxWidth())
            ArenaMetric("Targets", "$identifiedTargets found", modifier = Modifier.fillMaxWidth())
            HorizontalDivider()
            Column(
                modifier = Modifier.fillMaxWidth(),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(SpaceXs)
            ) {
                ArenaLegendItem(ObstacleBlue, "Obstacle")
                ArenaLegendItem(TargetAmber, "Target")
                ArenaLegendItem(RobotGreen, "Robot")
            }
        }
    }
}

@Composable
internal fun ArenaLegendItem(color: Color, label: String) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(4.dp)) {
        Surface(color = color, shape = RoundedCornerShape(50), modifier = Modifier.size(8.dp)) {}
        Text(label, style = MaterialTheme.typography.labelSmall)
    }
}

@Composable
internal fun ArenaCard(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    onSetRobotPose: (Int, Int, Face) -> Unit,
    onSendArena: () -> Unit,
    onClearObstacleTarget: (String) -> Unit,
    onClearObstacleSelection: () -> Unit
) {
    val selected = state.obstacles.firstOrNull { it.id == state.selectedObstacleId }
    // No card/background — sits directly on the page so the grid can claim the maximum space.
    Column(Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(SpaceSm)) {
        ArenaStatusStrip(state = state)
        BoxWithConstraints(modifier = Modifier.fillMaxWidth()) {
            // Fill the full card width so the grid is as large and touch-friendly as the
            // tablet allows.
            val mapSide = maxWidth
            ArenaGrid(
                state = state,
                onAddObstacle = onAddObstacle,
                onMoveObstacle = onMoveObstacle,
                onSelectObstacle = onSelectObstacle,
                onSetObstacleFace = onSetObstacleFace,
                onSetRobotStart = onSetRobotStart,
                onSetRobotFace = onSetRobotFace,
                modifier = Modifier.size(mapSide)
            )
        }
        ArenaLegendRow(connected = state.connected, onSendArena = onSendArena)
        if (selected != null) {
            TargetFaceSelection(
                obstacle = selected,
                onSetObstacleFace = onSetObstacleFace,
                onClearObstacleTarget = onClearObstacleTarget,
                onRemoveObstacle = onRemoveObstacle,
                onDone = onClearObstacleSelection
            )
        }
        RobotStartCard(robot = state.robot, onSetPose = onSetRobotPose)
    }
}

@Composable
internal fun TargetFaceSelection(
    obstacle: Obstacle,
    onSetObstacleFace: (String, Face) -> Unit,
    onClearObstacleTarget: (String) -> Unit,
    onRemoveObstacle: (String) -> Unit,
    onDone: () -> Unit
) {
    BoxWithConstraints(Modifier.fillMaxWidth()) {
        if (maxWidth >= 480.dp) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(SpaceXs),
                verticalAlignment = Alignment.CenterVertically
            ) {
                ObstacleDetails(obstacle, modifier = Modifier.width(112.dp))
                Text("Face", style = MaterialTheme.typography.labelSmall)
                FaceChoices(obstacle, onSetObstacleFace)
                if (obstacle.targetFace != null) {
                    CompactIconButton(
                        onClick = { onClearObstacleTarget(obstacle.id) },
                        icon = Icons.Default.Refresh,
                        contentDescription = "Clear selected face"
                    )
                }
                Spacer(Modifier.weight(1f))
                CompactIconButton(onClick = onDone, icon = Icons.Default.Check, contentDescription = "Done selecting target face")
                CompactIconButton(
                    onClick = { onRemoveObstacle(obstacle.id) },
                    icon = Icons.Default.Delete,
                    contentDescription = "Remove obstacle"
                )
            }
        } else {
            // Flush: no dead space from full-size (48dp) TextButton/IconButton touch targets, and
            // the two rows sit right on top of each other (spacedBy(SpaceXs), not SectionGap).
            Column(
                modifier = Modifier.fillMaxWidth(),
                verticalArrangement = Arrangement.spacedBy(SpaceXs)
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    ObstacleDetails(obstacle, modifier = Modifier.weight(1f))
                    CompactIconButton(onClick = onDone, icon = Icons.Default.Check, contentDescription = "Done selecting target face")
                    CompactIconButton(
                        onClick = { onRemoveObstacle(obstacle.id) },
                        icon = Icons.Default.Delete,
                        contentDescription = "Remove obstacle"
                    )
                }
                Row(
                    horizontalArrangement = Arrangement.spacedBy(SpaceXs),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Face", style = MaterialTheme.typography.labelSmall)
                    FaceChoices(obstacle, onSetObstacleFace)
                    if (obstacle.targetFace != null) {
                        CompactIconButton(
                            onClick = { onClearObstacleTarget(obstacle.id) },
                            icon = Icons.Default.Refresh,
                            contentDescription = "Clear selected face"
                        )
                    }
                }
            }
        }
    }
}

/** [IconButton] enforces a 48dp minimum touch target, which is a lot of dead space around a
 * small glyph in these already-cramped panels — this shrinks that to a still-tappable 32dp. */
@Composable
private fun CompactIconButton(
    onClick: () -> Unit,
    icon: ImageVector,
    contentDescription: String,
    modifier: Modifier = Modifier
) {
    IconButton(onClick = onClick, modifier = modifier.size(32.dp)) {
        Icon(icon, contentDescription = contentDescription, modifier = Modifier.size(18.dp))
    }
}

@Composable
internal fun ObstacleDetails(obstacle: Obstacle, modifier: Modifier = Modifier) {
    // A Box + contentAlignment, not a Row's horizontalArrangement group-alignment — guarantees
    // centering regardless of how much width the caller's weight()/fixed-width modifier grants.
    Box(modifier.fillMaxWidth(), contentAlignment = Alignment.Center) {
        Row(horizontalArrangement = Arrangement.spacedBy(SpaceXs), verticalAlignment = Alignment.Bottom) {
            Text("Obstacle ${obstacle.id}", fontWeight = FontWeight.SemiBold)
            Text(
                "(${obstacle.x}, ${obstacle.y})",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
        }
    }
}

@Composable
internal fun FaceChoices(obstacle: Obstacle, onSetObstacleFace: (String, Face) -> Unit) {
    Row(horizontalArrangement = Arrangement.spacedBy(SpaceXs)) {
        Face.entries.forEach { face ->
            FilterChip(
                selected = obstacle.targetFace == face,
                onClick = { onSetObstacleFace(obstacle.id, face) },
                label = { Text(face.code, fontWeight = FontWeight.Bold) }
            )
        }
    }
}

@Composable
internal fun ConnectionPill(connected: Boolean) {
    Surface(
        shape = RoundedCornerShape(50),
        color = if (connected) SuccessContainer else MaterialTheme.colorScheme.errorContainer
    ) {
        Text(
            if (connected) "LIVE" else "NO LINK",
            modifier = Modifier.padding(horizontal = 7.dp, vertical = 3.dp),
            style = MaterialTheme.typography.labelSmall.copy(fontSize = 9.sp),
            color = if (connected) MissionTeal else MaterialTheme.colorScheme.error,
            fontWeight = FontWeight.Bold
        )
    }
}

/** The "position window": a bigger, more readable status strip of key metrics, shown above the
 * map instead of a side panel so the map itself can use the full width. */
@Composable
internal fun ArenaStatusStrip(state: AppState, modifier: Modifier = Modifier) {
    val identifiedTargets = state.obstacles.count { it.targetId != null }
    Surface(
        modifier = modifier.fillMaxWidth(),
        shape = RoundedCornerShape(18.dp),
        color = MaterialTheme.colorScheme.secondaryContainer
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 18.dp, vertical = 8.dp),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            ArenaMetric("Position", "(${state.robot.x}, ${state.robot.y})")
            ArenaMetric("Heading", state.robot.direction.code)
            ArenaMetric("Obstacles", state.obstacles.size.toString())
            ArenaMetric("Targets", "$identifiedTargets found")
        }
    }
}

@Composable
internal fun ArenaMetric(label: String, value: String, modifier: Modifier = Modifier) {
    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Text(
            label,
            style = MaterialTheme.typography.labelMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            textAlign = TextAlign.Center
        )
        Text(value, style = MaterialTheme.typography.titleMedium, fontWeight = FontWeight.Bold, textAlign = TextAlign.Center)
    }
}

/** Legend plus the connection/sync status that used to sit in a header row above the map — moved
 * here so the map itself can claim that vertical space. Falls back to two lines when the card is
 * too narrow (the landscape Arena layout's grid column) for everything to fit on one. */
@Composable
internal fun ArenaLegendRow(
    connected: Boolean,
    onSendArena: () -> Unit,
    modifier: Modifier = Modifier
) {
    BoxWithConstraints(modifier.fillMaxWidth()) {
        if (maxWidth >= 520.dp) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(SpaceSm)
            ) {
                Row(modifier = Modifier.weight(1f), horizontalArrangement = Arrangement.spacedBy(SpaceSm)) {
                    ArenaLegendItem(ObstacleBlue, "Obstacle")
                    ArenaLegendItem(TargetAmber, "Target")
                    ArenaLegendItem(RobotGreen, "Robot")
                }
                ConnectionPill(connected = connected)
                TextButton(onClick = onSendArena, contentPadding = PaddingValues(horizontal = 6.dp, vertical = 0.dp)) {
                    Text("SEND ARENA", style = MaterialTheme.typography.labelSmall.copy(fontSize = 9.sp))
                }
            }
        } else {
            Column(verticalArrangement = Arrangement.spacedBy(SpaceXs)) {
                Row(horizontalArrangement = Arrangement.spacedBy(SpaceSm)) {
                    ArenaLegendItem(ObstacleBlue, "Obstacle")
                    ArenaLegendItem(TargetAmber, "Target")
                    ArenaLegendItem(RobotGreen, "Robot")
                }
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(SpaceSm)) {
                    ConnectionPill(connected = connected)
                    TextButton(onClick = onSendArena, contentPadding = PaddingValues(horizontal = 6.dp, vertical = 0.dp)) {
                        Text("SYNC ARENA", style = MaterialTheme.typography.labelSmall.copy(fontSize = 9.sp))
                    }
                }
            }
        }
    }
}

/** Sized by the caller to fill the available card width for a large, touch-friendly grid. */
@Composable
internal fun ArenaGrid(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    modifier: Modifier = Modifier
) {
    val gutter = 18.dp
    val labelStyle = MaterialTheme.typography.labelSmall.copy(fontSize = 9.sp, textAlign = TextAlign.Center)
    BoxWithConstraints(modifier = modifier) {
        // Independent width/height (not forced to a square) so the grid fills whatever space the
        // caller gives it edge-to-edge instead of centering a smaller square inside it.
        val canvasWidth = (maxWidth - gutter).coerceAtLeast(1.dp)
        val canvasHeight = (maxHeight - gutter).coerceAtLeast(1.dp)
        val cellWidth = canvasWidth / MAP_COLUMNS
        val cellHeight = canvasHeight / MAP_ROWS
        Column {
            Row(Modifier.height(canvasHeight)) {
                Column(Modifier.width(gutter).height(canvasHeight)) {
                    for (y in MAP_ROWS - 1 downTo 0) {
                        Text(
                            text = y.toString(),
                            style = labelStyle,
                            modifier = Modifier.height(cellHeight).fillMaxWidth()
                        )
                    }
                }
                ArenaCanvas(
                    state = state,
                    onAddObstacle = onAddObstacle,
                    onMoveObstacle = onMoveObstacle,
                    onSelectObstacle = onSelectObstacle,
                    onSetObstacleFace = onSetObstacleFace,
                    onSetRobotStart = onSetRobotStart,
                    onSetRobotFace = onSetRobotFace,
                    modifier = Modifier.size(canvasWidth, canvasHeight)
                )
            }
            Row(Modifier.padding(start = gutter).height(gutter)) {
                for (x in 0 until MAP_COLUMNS) {
                    Text(x.toString(), style = labelStyle, modifier = Modifier.width(cellWidth))
                }
            }
        }
    }
}

/** A bordered [BasicTextField] instead of Material3's OutlinedTextField — no built-in minimum
 * touch target, so it hugs the digit instead of leaving a lot of empty space in the box. */
@Composable
private fun CompactNumberField(
    value: String,
    onValueChange: (String) -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .background(MaterialTheme.colorScheme.surface, RoundedCornerShape(8.dp))
            .border(1.dp, MaterialTheme.colorScheme.outline, RoundedCornerShape(8.dp))
            .padding(horizontal = 10.dp, vertical = 6.dp),
        contentAlignment = Alignment.Center
    ) {
        BasicTextField(
            value = value,
            onValueChange = onValueChange,
            singleLine = true,
            textStyle = MaterialTheme.typography.titleMedium.copy(
                color = MaterialTheme.colorScheme.onSurface,
                textAlign = TextAlign.Center
            ),
            cursorBrush = SolidColor(MaterialTheme.colorScheme.primary),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
            modifier = Modifier.fillMaxWidth()
        )
    }
}

@Composable
internal fun RobotStartCard(robot: RobotState, onSetPose: (Int, Int, Face) -> Unit) {
    var xText by remember(robot.x) { mutableStateOf(robot.x.toString()) }
    var yText by remember(robot.y) { mutableStateOf(robot.y.toString()) }
    var direction by remember(robot.direction) { mutableStateOf(robot.direction) }
    val setPose = {
        val x = xText.toIntOrNull()
        val y = yText.toIntOrNull()
        if (x != null && y != null) onSetPose(x, y, direction)
    }
    // OutlinedTextField can't shrink below its own ~56dp minimum touch target even with the
    // label removed, which is what left the box mostly empty around the digit — a bordered
    // BasicTextField has no such floor, so it hugs the text instead.
    val positionFields: @Composable RowScope.() -> Unit = {
        Text("X", style = MaterialTheme.typography.labelSmall, modifier = Modifier.align(Alignment.CenterVertically))
        CompactNumberField(
            value = xText,
            onValueChange = { xText = it.filter(Char::isDigit).take(2) },
            modifier = Modifier.weight(1f)
        )
        Text("Y", style = MaterialTheme.typography.labelSmall, modifier = Modifier.align(Alignment.CenterVertically))
        CompactNumberField(
            value = yText,
            onValueChange = { yText = it.filter(Char::isDigit).take(2) },
            modifier = Modifier.weight(1f)
        )
    }
    val setButton = @Composable {
        Button(
            onClick = setPose,
            modifier = Modifier.height(32.dp),
            contentPadding = PaddingValues(horizontal = SpaceSm, vertical = 0.dp)
        ) {
            Text("Set", style = MaterialTheme.typography.labelSmall, fontWeight = FontWeight.Bold)
        }
    }
    // Chips stretch to evenly fill whatever width the caller gives this row, rather than
    // clumping to the left, so "Robot  N E S W" spans the full card width.
    val directionChips: @Composable RowScope.() -> Unit = {
        Face.entries.forEach { face ->
            FilterChip(
                selected = direction == face,
                onClick = { direction = face },
                label = { Text(face.code, fontWeight = FontWeight.Bold) },
                modifier = Modifier.weight(1f),
                // The card's own surfaceVariant background matches FilterChip's default selected
                // fill closely enough that the selection becomes invisible; force real contrast.
                colors = FilterChipDefaults.filterChipColors(
                    selectedContainerColor = MaterialTheme.colorScheme.primary,
                    selectedLabelColor = MaterialTheme.colorScheme.onPrimary
                )
            )
        }
    }
    Column(
        modifier = Modifier.fillMaxWidth(),
        verticalArrangement = Arrangement.spacedBy(SpaceXs)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(SpaceSm),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text("Robot", style = MaterialTheme.typography.labelLarge, fontWeight = FontWeight.SemiBold)
            positionFields()
            setButton()
        }
        // Flush against the row above (spacedBy(SpaceXs), not a full SectionGap).
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(SpaceXs)
        ) { directionChips() }
    }
}

private enum class DragKind { OBSTACLE, ROBOT }

@Composable
internal fun ArenaCanvas(
    state: AppState,
    onAddObstacle: (GridPoint) -> Unit,
    onMoveObstacle: (String, Int, Int) -> Unit,
    onSelectObstacle: (String) -> Unit,
    onSetObstacleFace: (String, Face) -> Unit,
    onSetRobotStart: (Int, Int) -> Unit,
    onSetRobotFace: (Face) -> Unit,
    modifier: Modifier = Modifier
) {
    var dragKind by remember { mutableStateOf<DragKind?>(null) }
    var dragObstacleId by remember { mutableStateOf<String?>(null) }
    var dragOffset by remember { mutableStateOf<Offset?>(null) }
    Canvas(
        modifier = modifier
            .background(Color(0xFFEAF4F5), RoundedCornerShape(16.dp))
            .border(1.dp, Color(0xFFA9BEC9), RoundedCornerShape(16.dp))
            .pointerInput(state.obstacles, state.robot) {
                awaitEachGesture {
                    val down = awaitFirstDown()
                    val start = down.position
                    val startPoint = gridPoint(start, size.width.toFloat(), size.height.toFloat())
                    val obstacleId = state.obstacles.firstOrNull { it.x == startPoint.x && it.y == startPoint.y }?.id
                    val robotSelected = obstacleId == null && state.robot.occupies(startPoint.x, startPoint.y)
                    var current = start
                    var moved = false

                    while (true) {
                        val event = awaitPointerEvent()
                        val change = event.changes.firstOrNull { it.id == down.id } ?: break
                        if (change.positionChanged()) {
                            change.consume()
                            current = change.position
                            moved = true
                            when {
                                obstacleId != null -> {
                                    dragKind = DragKind.OBSTACLE
                                    dragObstacleId = obstacleId
                                    dragOffset = current
                                }
                                robotSelected -> {
                                    dragKind = DragKind.ROBOT
                                    dragOffset = current
                                }
                            }
                        }
                        if (!change.pressed) break
                    }

                    val releasedAt = gridPoint(current, size.width.toFloat(), size.height.toFloat())
                    val distance = (current - start).getDistance()
                    val cellSize = minOf(size.width, size.height).toFloat() / MAP_COLUMNS
                    val isTap = !moved || distance < 6.dp.toPx()
                    val isNudge = !isTap && distance < cellSize * 0.65f
                    dragKind = null
                    dragObstacleId = null
                    dragOffset = null
                    when {
                        isTap -> {
                            when {
                                obstacleId != null -> onSelectObstacle(obstacleId)
                                !robotSelected && startPoint.x in 0 until MAP_COLUMNS && startPoint.y in 0 until MAP_ROWS -> onAddObstacle(startPoint)
                            }
                        }
                        obstacleId != null && isNudge -> onSetObstacleFace(obstacleId, faceFromDrag(current - start))
                        obstacleId != null -> onMoveObstacle(obstacleId, releasedAt.x, releasedAt.y)
                        robotSelected && isNudge -> onSetRobotFace(faceFromDrag(current - start))
                        robotSelected -> onSetRobotStart(releasedAt.x, releasedAt.y)
                    }
                }
            }
    ) {
        val cellWidth = size.width / MAP_COLUMNS
        val cellHeight = size.height / MAP_ROWS

        for (x in 0..MAP_COLUMNS) drawLine(Color(0xFFD5E3E8), Offset(x.toFloat() * cellWidth, 0f), Offset(x.toFloat() * cellWidth, size.height), 1f)
        for (y in 0..MAP_ROWS) drawLine(Color(0xFFD5E3E8), Offset(0f, y.toFloat() * cellHeight), Offset(size.width, y.toFloat() * cellHeight), 1f)

        state.obstacles.forEach { obstacle ->
            if (dragKind == DragKind.OBSTACLE && dragObstacleId == obstacle.id) return@forEach
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

        if (dragKind != DragKind.ROBOT) {
            val footprintSpan = 2 * ROBOT_FOOTPRINT_RADIUS + 1
            val footprintLeft = (state.robot.x - ROBOT_FOOTPRINT_RADIUS) * cellWidth + 2f
            val footprintTop = (MAP_ROWS - 1 - (state.robot.y + ROBOT_FOOTPRINT_RADIUS)) * cellHeight + 2f
            val footprintWidth = footprintSpan * cellWidth - 4f
            val footprintHeight = footprintSpan * cellHeight - 4f
            val robotCenter = Offset(footprintLeft + footprintWidth / 2, footprintTop + footprintHeight / 2)
            val robotRadius = minOf(footprintWidth, footprintHeight) / 2
            drawCircle(RobotGreen, robotRadius, robotCenter)
            drawCircle(Color.White, robotRadius, robotCenter, style = Stroke(2f))
            val direction = when (state.robot.direction) {
                Face.N -> Offset(0f, -robotRadius * 0.85f)
                Face.S -> Offset(0f, robotRadius * 0.85f)
                Face.W -> Offset(-robotRadius * 0.85f, 0f)
                Face.E -> Offset(robotRadius * 0.85f, 0f)
            }
            drawLine(Color.White, robotCenter, robotCenter + direction, 4f, cap = StrokeCap.Round)
        }

        val liveOffset = dragOffset
        when (dragKind) {
            DragKind.OBSTACLE -> {
                val obstacle = state.obstacles.firstOrNull { it.id == dragObstacleId }
                if (obstacle != null && liveOffset != null) {
                    val point = gridPoint(liveOffset, size.width, size.height)
                    val outOfBounds = point.x !in 0 until MAP_COLUMNS || point.y !in 0 until MAP_ROWS
                    val fill = if (outOfBounds) MissionError else if (obstacle.targetId != null) TargetAmber else ObstacleBlue
                    drawRect(
                        fill.copy(alpha = 0.85f),
                        Offset(liveOffset.x - cellWidth / 2 + 2f, liveOffset.y - cellHeight / 2 + 2f),
                        Size(cellWidth - 4f, cellHeight - 4f)
                    )
                    drawCenteredText(
                        obstacle.targetId ?: obstacle.id.removePrefix("B"),
                        liveOffset,
                        Color.White,
                        cellWidth.coerceAtMost(cellHeight) * 0.38f
                    )
                }
            }
            DragKind.ROBOT -> if (liveOffset != null) {
                val point = gridPoint(liveOffset, size.width, size.height)
                val outOfBounds = point.x !in ROBOT_FOOTPRINT_RADIUS until (MAP_COLUMNS - ROBOT_FOOTPRINT_RADIUS) ||
                    point.y !in ROBOT_FOOTPRINT_RADIUS until (MAP_ROWS - ROBOT_FOOTPRINT_RADIUS)
                val footprintSpan = 2 * ROBOT_FOOTPRINT_RADIUS + 1
                val w = footprintSpan * cellWidth - 4f
                val h = footprintSpan * cellHeight - 4f
                val radius = minOf(w, h) / 2
                val fill = if (outOfBounds) MissionError else RobotGreen
                drawCircle(fill.copy(alpha = 0.85f), radius, liveOffset)
                drawCircle(Color.White, radius, liveOffset, style = Stroke(2f))
            }
            null -> Unit
        }
    }
}

private fun DrawScope.drawCenteredText(
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
