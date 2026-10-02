package com.example.mdpandroid

import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.width
import androidx.compose.ui.Modifier
import androidx.compose.ui.semantics.SemanticsActions
import androidx.compose.ui.text.TextLayoutResult
import androidx.compose.ui.test.*
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.compose.ui.unit.dp
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.RobotActivityCard
import com.example.mdpandroid.ui.theme.MDPAndroidTheme
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import java.util.UUID
import java.util.concurrent.TimeUnit

@RunWith(AndroidJUnit4::class)
class ActivityLogUiTest {
    @get:Rule val compose = createComposeRule()
    private val context get() = ApplicationProvider.getApplicationContext<android.content.Context>()

    @Test fun shortenedEntryOpensFullTextAndClearingRequiresConfirmation() = withHistory(1) { history ->
        val fullText = "full text: " + "robot detail ".repeat(80)
        compose.runOnIdle { history.append(ActivitySource.RECEIVED, StatusMessage("2026-10-02 12:00:00", fullText)) }
        compose.waitUntil(10_000) { history.state.totalCount == 2L }
        compose.onNodeWithText(fullText, substring = true).performClick()
        compose.onNodeWithText("Log message").assertIsDisplayed()
        compose.onNode(hasText(fullText) and hasAnyAncestor(isDialog())).assertExists()
        compose.onNodeWithText("Close").performClick()
        compose.onNodeWithContentDescription("Clear history").performClick()
        compose.onNodeWithText("Cancel").performClick()
        assertEquals(2L, history.state.totalCount)
        compose.onNodeWithContentDescription("Clear history").performClick()
        compose.onNodeWithText("Clear", substring = false).performClick()
        compose.waitUntil(10_000) { !history.state.loading && history.state.totalCount == 0L }
        compose.onNodeWithText("No robot activity yet.").assertIsDisplayed()
    }

    @Test fun scrollingLoadsOlderLogsAndLatestReturnsToIncomingMessages() = withHistory(220) { history ->
        val list = compose.onNodeWithTag("activity-history")
        repeat(30) {
            if (history.state.entries.last().message.text != "entry 0") {
                list.performTouchInput { swipeDown(durationMillis = 150) }
                compose.waitForIdle()
            }
        }
        compose.waitUntil(10_000) { history.state.entries.last().message.text == "entry 0" }
        assertFalse(history.state.followingLatest)
        val viewed = history.state.entries
        compose.runOnIdle { history.append(ActivitySource.STATUS, StatusMessage("time", "new live entry")) }
        compose.waitUntil(10_000) { history.state.totalCount == 221L }
        assertEquals(viewed, history.state.entries)
        compose.onNodeWithContentDescription("Jump to latest logs").performClick()
        compose.waitUntil(10_000) { !history.state.loading && history.state.entries.first().message.text == "new live entry" }
        compose.onNodeWithText("new live entry", substring = true).assertIsDisplayed()
    }

    @Test fun narrowPanelWrapsMessageBelowShortTimeAndKeepsFullTimestampInDetails() = withHistory(0, modifier = Modifier.width(280.dp), boxed = false) { history ->
        val timestamp = "2026-10-02 18:01:03.462"
        val message = "Arena not sent: set a face for B2, B3 and B4 before starting the robot."
        compose.runOnIdle { history.append(ActivitySource.STATUS, StatusMessage(timestamp, message)) }
        compose.waitUntil(10_000) { history.state.totalCount == 1L }
        compose.onNodeWithText("18:01:03").assertIsDisplayed()
        val text = compose.onNodeWithText(message, useUnmergedTree = true)
        text.assertIsDisplayed()
        val layouts = mutableListOf<TextLayoutResult>()
        text.performSemanticsAction(SemanticsActions.GetTextLayoutResult) { it(layouts) }
        assertTrue("Message must wrap instead of being restricted to one line", layouts.single().lineCount > 1)
        assertFalse(layouts.single().isLineEllipsized(0))
        text.performClick()
        compose.onNodeWithText(timestamp, substring = true).assertIsDisplayed()
        compose.onNode(hasText(message) and hasAnyAncestor(isDialog())).assertIsDisplayed()
    }

    @Test fun shortMessageFitsBesideSourceAndTimeWithoutTruncation() = withHistory(0, modifier = Modifier.width(280.dp), boxed = false) { history ->
        compose.runOnIdle {
            history.append(ActivitySource.STATUS, StatusMessage("2026-10-02 18:01:03.462", "Bluetooth disconnected"))
        }
        compose.waitUntil(10_000) { history.state.totalCount == 1L }
        val message = compose.onNodeWithText("Bluetooth disconnected", useUnmergedTree = true)
        val messageBounds = message.fetchSemanticsNode().boundsInRoot
        val timeBounds = compose.onNodeWithText("18:01:03", useUnmergedTree = true).fetchSemanticsNode().boundsInRoot
        assertTrue("Short message and time should share one line",
            messageBounds.top < timeBounds.bottom && timeBounds.top < messageBounds.bottom)
        val layouts = mutableListOf<TextLayoutResult>()
        message.performSemanticsAction(SemanticsActions.GetTextLayoutResult) { it(layouts) }
        assertEquals(1, layouts.single().lineCount)
        assertFalse(layouts.single().isLineEllipsized(0))
    }

    private fun withHistory(count: Int, modifier: Modifier = Modifier.fillMaxWidth(), boxed: Boolean = true, test: (ActivityLogHistory) -> Unit) {
        val name = "ui-log-test-${UUID.randomUUID()}.db"
        ActivityLogStore(context, name).use { store ->
            repeat(count) { store.append(ActivitySource.STATUS, "time", "entry $it") }
        }
        lateinit var history: ActivityLogHistory
        compose.runOnIdle { history = ActivityLogHistory(context, name) }
        try {
            compose.setContent { MDPAndroidTheme { RobotActivityCard(history, modifier = modifier, compact = true, boxed = boxed) } }
            compose.waitUntil(10_000) { !history.state.loading }
            test(history)
        } finally { history.close().get(10, TimeUnit.SECONDS); context.deleteDatabase(name) }
    }
}
