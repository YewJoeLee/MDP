package com.example.mdpandroid.ui

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import com.example.mdpandroid.ActivitySource
import com.example.mdpandroid.StatusMessage
import com.example.mdpandroid.mergeActivityLog

@Composable
internal fun RobotActivityCard(
    statusMessages: List<StatusMessage>,
    receivedRawMessages: List<StatusMessage>,
    modifier: Modifier = Modifier,
    compact: Boolean = false,
    fillHeight: Boolean = false,
    boxed: Boolean = true
) {
    val logEntries = mergeActivityLog(statusMessages, receivedRawMessages)
    val content = @Composable {
        Column(
            modifier = Modifier
                .then(if (boxed) Modifier.padding(CardInset) else Modifier)
                .then(if (fillHeight) Modifier.fillMaxHeight() else Modifier),
            verticalArrangement = Arrangement.spacedBy(if (boxed) SectionGap else SpaceXs)
        ) {
            Text(
                "Robot activity",
                style = if (boxed) MaterialTheme.typography.bodyLarge else MaterialTheme.typography.labelLarge,
                fontWeight = FontWeight.Bold
            )
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
                // When the card's own height is constrained by a sibling (fillHeight), let the
                // list take whatever room remains instead of forcing a fixed height that could
                // overflow the card's bounds and get clipped.
                val listModifier = if (fillHeight) Modifier.weight(1f) else Modifier.height(if (compact) 124.dp else 180.dp)
                LazyColumn(modifier = listModifier, reverseLayout = true) {
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
    if (boxed) {
        Card(
            modifier = modifier
                .fillMaxWidth()
                .then(if (fillHeight) Modifier.fillMaxHeight() else Modifier)
                .border(1.dp, MaterialTheme.colorScheme.outlineVariant, RoundedCornerShape(20.dp)),
            shape = RoundedCornerShape(20.dp),
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface)
        ) {
            content()
        }
    } else {
        Box(
            modifier = modifier
                .fillMaxWidth()
                .then(if (fillHeight) Modifier.fillMaxHeight() else Modifier)
        ) {
            content()
        }
    }
}
