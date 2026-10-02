package com.example.mdpandroid.ui

import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.DeleteOutline
import androidx.compose.material.icons.filled.FileDownload
import androidx.compose.material.icons.filled.VerticalAlignBottom
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Constraints
import androidx.compose.ui.unit.dp
import com.example.mdpandroid.ActivityLogEntry
import com.example.mdpandroid.ActivityLogHistory
import com.example.mdpandroid.ActivitySource
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@Composable
internal fun RobotActivityCard(
    history: ActivityLogHistory,
    modifier: Modifier = Modifier,
    compact: Boolean = false,
    fillHeight: Boolean = false,
    boxed: Boolean = true
) {
    val state = history.state
    val listState = rememberLazyListState()
    var selected by remember { mutableStateOf<ActivityLogEntry?>(null) }
    var confirmClear by remember { mutableStateOf(false) }
    var jumping by remember { mutableStateOf(false) }
    val export = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("text/plain")) { uri ->
        uri?.let(history::export)
    }
    LaunchedEffect(history, listState) {
        snapshotFlow {
            Triple(listState.isScrollInProgress, listState.firstVisibleItemIndex to listState.firstVisibleItemScrollOffset,
                listState.layoutInfo.visibleItemsInfo.lastOrNull()?.index ?: 0)
        }.collect { (scrolling, position, last) ->
            val (first, offset) = position
            val current = history.state
            if (jumping) return@collect
            if (scrolling && (first > 0 || offset > 0)) history.followLatest(false)
            if (!scrolling && first == 0 && offset == 0 && !current.hasNewer) history.followLatest(true)
            if (scrolling && !current.loading) {
                if (last >= current.entries.size - 10 && current.hasOlder) history.loadOlder()
                else if (first <= 10 && current.hasNewer) history.loadNewer()
            }
        }
    }
    LaunchedEffect(state.entries.firstOrNull()?.message?.order, state.followingLatest, state.loading) {
        if (state.followingLatest && !state.loading && state.entries.isNotEmpty()) {
            jumping = true
            try { listState.scrollToItem(0) } finally { jumping = false }
        }
    }
    val content = @Composable {
        Column(
            modifier = Modifier
                .then(if (boxed) Modifier.padding(CardInset) else Modifier)
                .then(if (fillHeight) Modifier.fillMaxHeight() else Modifier),
            verticalArrangement = Arrangement.spacedBy(if (boxed) SectionGap else SpaceXs)
        ) {
            Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                Text("Robot activity (${state.totalCount})", modifier = Modifier.weight(1f), maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    style = if (boxed) MaterialTheme.typography.bodyLarge else MaterialTheme.typography.labelLarge,
                    fontWeight = FontWeight.Bold)
                IconButton(onClick = history::showLatest, enabled = !state.loading) {
                    Icon(Icons.Default.VerticalAlignBottom, contentDescription = "Jump to latest logs")
                }
                IconButton(onClick = {
                    val date = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
                    export.launch("robot-logs-$date.txt")
                }) { Icon(Icons.Default.FileDownload, contentDescription = "Export logs") }
                IconButton(onClick = { confirmClear = true }, enabled = state.totalCount > 0) {
                    Icon(Icons.Default.DeleteOutline, contentDescription = "Clear history")
                }
            }
            if (!compact) Text("Saved history. Tap a message to read it in full.",
                style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
            if (state.entries.isEmpty()) {
                Text(if (state.loading) "Loading history…" else "No robot activity yet.", style = MaterialTheme.typography.bodySmall)
            } else {
                val listModifier = if (fillHeight) Modifier.weight(1f) else Modifier.height(if (compact) 124.dp else 180.dp)
                LazyColumn(modifier = listModifier.testTag("activity-history"), state = listState, reverseLayout = true) {
                    items(state.entries, key = { it.message.order }) { entry ->
                        ActivityLogRow(entry, compact, onClick = { selected = entry })
                    }
                    if (state.loading) item { Text("Loading history…", style = MaterialTheme.typography.bodySmall) }
                }
            }
        }
    }
    if (boxed) {
        Card(modifier.fillMaxWidth().then(if (fillHeight) Modifier.fillMaxHeight() else Modifier)
            .border(1.dp, MaterialTheme.colorScheme.outlineVariant, RoundedCornerShape(20.dp)),
            shape = RoundedCornerShape(20.dp), colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)) { content() }
    } else {
        Box(modifier.fillMaxWidth().then(if (fillHeight) Modifier.fillMaxHeight() else Modifier)) { content() }
    }
    selected?.let { entry ->
        AlertDialog(onDismissRequest = { selected = null }, title = { Text("Log message") },
            text = {
                SelectionContainer {
                    Column(Modifier.heightIn(max = 320.dp).verticalScroll(rememberScrollState())) {
                        Text("${entry.message.time} · ${if (entry.source == ActivitySource.STATUS) "Status" else "Received"}",
                            style = MaterialTheme.typography.labelSmall)
                        Spacer(Modifier.height(8.dp))
                        Text(entry.message.text)
                    }
                }
            }, confirmButton = { TextButton(onClick = { selected = null }) { Text("Close") } })
    }
    if (confirmClear) {
        AlertDialog(onDismissRequest = { confirmClear = false }, title = { Text("Clear log history?") },
            text = { Text("This deletes all saved log entries from this device.") },
            confirmButton = { TextButton(onClick = { confirmClear = false; selected = null; history.clear() }) { Text("Clear") } },
            dismissButton = { TextButton(onClick = { confirmClear = false }) { Text("Cancel") } })
    }
    state.notice?.let { notice ->
        AlertDialog(onDismissRequest = history::dismissNotice, title = { Text("Log history") },
            text = { Text(notice) }, confirmButton = { TextButton(onClick = history::dismissNotice) { Text("OK") } })
    }
}

@Composable
private fun ActivityLogRow(entry: ActivityLogEntry, compact: Boolean, onClick: () -> Unit) {
    val source = if (entry.source == ActivitySource.RECEIVED) "Received" else "Status"
    // Keep the full saved timestamp in details and exports.
    val time = entry.message.time.substringAfterLast(' ').substringBefore('.')
    val sourceStyle = MaterialTheme.typography.labelSmall.copy(fontWeight = FontWeight.Bold)
    val timeStyle = MaterialTheme.typography.labelSmall
    val messageStyle = MaterialTheme.typography.bodySmall
    val sourceColor = if (entry.source == ActivitySource.RECEIVED) MaterialTheme.colorScheme.tertiary
        else MaterialTheme.colorScheme.primary
    val timeColor = MaterialTheme.colorScheme.onSurfaceVariant
    val measurer = rememberTextMeasurer()
    val density = LocalDensity.current

    BoxWithConstraints(Modifier.fillMaxWidth().clickable(onClick = onClick).padding(vertical = 6.dp)) {
        val messageWidth = with(density) { maxWidth.roundToPx() - 16.dp.roundToPx() } -
            measurer.measure(source, sourceStyle, softWrap = false).size.width -
            measurer.measure(time, timeStyle, softWrap = false).size.width
        val fitsInline = messageWidth > 0 && '\n' !in entry.message.text && '\r' !in entry.message.text &&
            !measurer.measure(entry.message.text, messageStyle, softWrap = false, maxLines = 1,
                constraints = Constraints(maxWidth = messageWidth)).hasVisualOverflow

        if (fitsInline) {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(source, style = sourceStyle, color = sourceColor, modifier = Modifier.alignByBaseline())
                Text(time, style = timeStyle, color = timeColor, modifier = Modifier.alignByBaseline())
                Text(entry.message.text, style = messageStyle, maxLines = 1,
                    modifier = Modifier.alignByBaseline())
            }
        } else {
            Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically) {
                    Text(source, style = sourceStyle, color = sourceColor)
                    Text(time, style = timeStyle, color = timeColor)
                }
                Text(entry.message.text, style = messageStyle, modifier = Modifier.fillMaxWidth(),
                    maxLines = if (compact) 3 else Int.MAX_VALUE, overflow = TextOverflow.Ellipsis)
            }
        }
    }
}
