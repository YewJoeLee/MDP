package com.example.mdpandroid.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.BoxWithConstraints
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
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
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
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
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
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChanged
import androidx.compose.ui.text.font.FontWeight
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
            onSetObstacleFace = onSetObstacleFace,
            onSetRobotStart = onSetRobotStart,
            onSetRobotFace = onSetRobotFace,
            onSetRobotPose = onSetRobotPose,
            onSendArena = onSendArena,
            onClearObstacleTarget = onClearObstacleTarget,
            onClearObstacleSelection = onClearObstacleSelection
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
                ConnectionPill(connected = state.connected)
                TextButton(onClick = onSendArena, contentPadding = PaddingValues(horizontal = SpaceSm, vertical = 0.dp)) {
                    Text("SYNC ARENA", style = MaterialTheme.typography.labelSmall)
                }
            }
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
            ArenaLegendRow()
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
}

@Composable
internal fun TargetFaceSelection(
    obstacle: Obstacle,
    onSetObstacleFace: (String, Face) -> Unit,
    onClearObstacleTarget: (String) -> Unit,
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
                    FaceChoices(obstacle, onSetObstacleFace)
                    if (obstacle.targetFace != null) {
                        IconButton(onClick = { onClearObstacleTarget(obstacle.id) }) {
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
                        FaceChoices(obstacle, onSetObstacleFace)
                        if (obstacle.targetFace != null) {
                            IconButton(onClick = { onClearObstacleTarget(obstacle.id) }) {
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
internal fun ObstacleDetails(obstacle: Obstacle, modifier: Modifier = Modifier) {
    Column(modifier) {
        Text("Obstacle ${obstacle.id}", fontWeight = FontWeight.SemiBold)
        Text("(${obstacle.x}, ${obstacle.y})", style = MaterialTheme.typography.bodySmall)
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
        color = if (connected) SuccessContainer else MaterialTheme.colorScheme.errorContainer,
        modifier = Modifier.padding(end = SpaceSm)
    ) {
        Text(
            if (connected) "LIVE" else "NO LINK",
            modifier = Modifier.padding(horizontal = 8.dp, vertical = 5.dp),
            style = MaterialTheme.typography.labelSmall,
            color = if (connected) MissionTeal else MaterialTheme.colorScheme.error,
            fontWeight = FontWeight.Bold
        )
    }
}

/** A full-width status strip of key metrics, shown above the map instead of a side panel so the map itself can use the full width. */
@Composable
internal fun ArenaStatusStrip(state: AppState, modifier: Modifier = Modifier) {
    val identifiedTargets = state.obstacles.count { it.targetId != null }
    Surface(
        modifier = modifier.fillMaxWidth(),
        shape = RoundedCornerShape(16.dp),
        color = MaterialTheme.colorScheme.secondaryContainer
    ) {
        Row(
            modifier = Modifier.padding(horizontal = CardInset, vertical = SpaceSm),
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
    Column(modifier) {
        Text(label, style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, style = MaterialTheme.typography.bodyMedium, fontWeight = FontWeight.SemiBold)
    }
}

/** Legend and interaction hint, shown as a single row under the map instead of a side panel. */
@Composable
internal fun ArenaLegendRow(modifier: Modifier = Modifier) {
    Column(modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(SpaceXs)) {
        Row(horizontalArrangement = Arrangement.spacedBy(SpaceSm)) {
            ArenaLegendItem(ObstacleBlue, "Obstacle")
            ArenaLegendItem(TargetAmber, "Target")
            ArenaLegendItem(RobotGreen, "Robot")
        }
        Text(
            "Tap to add an obstacle. Short-drag to set a face; drag the robot to set its start.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
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
        val canvasSide = (minOf(maxWidth, maxHeight) - gutter).coerceAtLeast(1.dp)
        val cellSize = canvasSide / MAP_COLUMNS
        Column {
            Row(Modifier.height(canvasSide)) {
                Column(Modifier.width(gutter).height(canvasSide)) {
                    for (y in MAP_ROWS - 1 downTo 0) {
                        Text(
                            text = y.toString(),
                            style = labelStyle,
                            modifier = Modifier.height(cellSize).fillMaxWidth()
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
                    modifier = Modifier.size(canvasSide)
                )
            }
            Row(Modifier.padding(start = gutter).height(gutter)) {
                for (x in 0 until MAP_COLUMNS) {
                    Text(x.toString(), style = labelStyle, modifier = Modifier.width(cellSize))
                }
            }
        }
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
    val positionFields = @Composable {
        OutlinedTextField(
            value = xText,
            onValueChange = { xText = it.filter(Char::isDigit).take(2) },
            label = { Text("X") },
            singleLine = true,
            modifier = Modifier.width(68.dp)
        )
        OutlinedTextField(
            value = yText,
            onValueChange = { yText = it.filter(Char::isDigit).take(2) },
            label = { Text("Y") },
            singleLine = true,
            modifier = Modifier.width(68.dp)
        )
    }
    val directionChips = @Composable {
        Face.entries.forEach { face ->
            FilterChip(
                selected = direction == face,
                onClick = { direction = face },
                label = { Text(face.code, fontWeight = FontWeight.Bold) },
                // The card's own surfaceVariant background matches FilterChip's default selected
                // fill closely enough that the selection becomes invisible; force real contrast.
                colors = FilterChipDefaults.filterChipColors(
                    selectedContainerColor = MaterialTheme.colorScheme.primary,
                    selectedLabelColor = MaterialTheme.colorScheme.onPrimary
                )
            )
        }
    }
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
    ) {
        BoxWithConstraints(Modifier.fillMaxWidth()) {
            if (maxWidth >= 560.dp) {
                Row(
                    modifier = Modifier.padding(horizontal = CardInset, vertical = SpaceSm),
                    horizontalArrangement = Arrangement.spacedBy(SpaceSm),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column(Modifier.weight(1f)) {
                        Text("Robot start", fontWeight = FontWeight.SemiBold)
                        Text("Set grid position and facing", style = MaterialTheme.typography.bodySmall)
                    }
                    positionFields()
                    Row(horizontalArrangement = Arrangement.spacedBy(SpaceXs)) { directionChips() }
                    Button(onClick = setPose) { Text("Set") }
                }
            } else {
                Column(
                    modifier = Modifier.padding(CardInset),
                    verticalArrangement = Arrangement.spacedBy(SpaceXs)
                ) {
                    Text("Robot start", fontWeight = FontWeight.SemiBold)
                    Text("Set grid position and facing", style = MaterialTheme.typography.bodySmall)
                    Row(
                        horizontalArrangement = Arrangement.spacedBy(SpaceSm),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        positionFields()
                        Spacer(Modifier.weight(1f))
                        Button(onClick = setPose) { Text("Set") }
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(SpaceXs)) { directionChips() }
                }
            }
        }
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
