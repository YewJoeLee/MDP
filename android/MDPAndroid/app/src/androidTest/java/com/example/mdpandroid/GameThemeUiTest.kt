package com.example.mdpandroid

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.width
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.asAndroidBitmap
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.unit.dp
import androidx.compose.ui.test.*
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.AssessmentCommandCard
import com.example.mdpandroid.ui.MissionOverviewCard
import com.example.mdpandroid.ui.ARCMApp
import com.example.mdpandroid.ui.ControlCard
import com.example.mdpandroid.ui.theme.MDPAndroidTheme
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import androidx.test.core.app.ApplicationProvider
import java.io.File
import java.util.UUID
import java.util.concurrent.TimeUnit

/** Theme changes must not merge the two start gates or imply fabricated results. */
@RunWith(AndroidJUnit4::class)
class GameThemeUiTest {
    @get:Rule val compose = createComposeRule()

    @Test fun themedGamesKeepIndependentReadinessAndActualCommands() {
        var state by mutableStateOf(AppState(demoMode = true))
        val commands = mutableListOf<RobotCommand>()
        compose.setContent {
            MDPAndroidTheme {
                AssessmentCommandCard(state, onCommand = {
                    commands += it
                    state = state.copy(taskRun = TaskRun(it, TaskPhase.RUNNING, simulated = true))
                }, onSendArena = {}, onCancelTask = { state = state.copy(taskRun = null) })
            }
        }
        compose.onNodeWithText("TASK 1").assertExists()
        compose.onNodeWithText("TASK 2").assertExists()
        compose.onNodeWithText("Image recognition").assertExists()
        compose.onNodeWithText("Fastest path", substring = false).assertExists()
        compose.onNodeWithText("Red Light, Green Light").assertDoesNotExist()
        compose.onNodeWithText("Glass Bridge").assertDoesNotExist()
        compose.onNodeWithText("Task 1\nExplore").assertIsNotEnabled()
        compose.onNodeWithText("Task 2\nFastest path").assertIsEnabled().performClick()
        compose.runOnIdle { assertEquals(listOf(RobotCommand.BEGIN_FASTEST), commands) }
        compose.onNodeWithText("Task 1\nExplore").assertIsNotEnabled()
        compose.onNodeWithText("Task 2\nFastest path").assertIsNotEnabled()
        compose.onNodeWithText("Finish demo task").performClick()
        compose.runOnIdle { assertNull(state.taskRun) }
    }

    @Test fun overviewCountsReceivedIdsAndLabelsDemoAsDemo() {
        val state = AppState(demoMode = true, obstacles = listOf(
            Obstacle("B1", 4, 7, targetId = "12", targetFace = Face.S),
            Obstacle("B2", 8, 16, targetFace = Face.W)
        ))
        compose.setContent { MDPAndroidTheme { MissionOverviewCard(state, demoMode = true) } }
        compose.onNodeWithTag("recognition-total").assertTextEquals("1 / 2")
        compose.onNodeWithText("DEMO SESSION").assertExists()
        compose.onNodeWithText("LIVE SESSION").assertDoesNotExist()
    }

    @Test fun activeTaskLocksManualDriveWithoutClaimingToBeOffline() {
        compose.setContent {
            MDPAndroidTheme { ControlCard(enabled = false, onCommand = {}, taskActive = true) }
        }
        compose.onNodeWithText("TASK ACTIVE").assertIsDisplayed()
        compose.onNodeWithText("OFFLINE").assertDoesNotExist()
        compose.onNodeWithContentDescription("Move forward").assertIsNotEnabled()
    }

    @Test fun themedTabsRemainUsableAtTabletAndCompactWidths() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val name = "theme-ui-${UUID.randomUUID()}"
        val store = ArenaStore(context, name)
        val history = ActivityLogHistory(context, "$name.db")
        lateinit var controller: BluetoothController
        var compact by mutableStateOf(false)
        compose.runOnIdle {
            controller = BluetoothController(context, history, store)
            controller.setDemoMode(true)
            controller.restoreArena(ArenaSnapshot(selectedObstacleId = null, robot = RobotState(), obstacles = listOf(
                Obstacle("B1", 12, 12, targetFace = Face.S), Obstacle("B2", 8, 16, targetFace = Face.W),
                Obstacle("B3", 4, 7, targetFace = Face.S), Obstacle("B4", 13, 4, targetId = "11", targetFace = Face.N),
                Obstacle("B5", 8, 3, targetFace = Face.W), Obstacle("B6", 9, 8, targetFace = Face.E),
                Obstacle("B7", 3, 12, targetFace = Face.E), Obstacle("B8", 16, 16, targetFace = Face.S)
            )))
        }
        try {
            compose.setContent { MDPAndroidTheme {
                Box(Modifier.width(if (compact) 400.dp else 960.dp).fillMaxHeight().testTag("theme-app")) {
                    ARCMApp(controller, requestBluetoothPermissions = {})
                }
            } }
            fun capture(file: String) {
                compose.waitForIdle()
                val dir = File(context.getExternalFilesDir(null), "theme-captures").apply { mkdirs() }
                File(dir, file).outputStream().use {
                    compose.onNodeWithTag("theme-app").captureToImage().asAndroidBitmap()
                        .compress(android.graphics.Bitmap.CompressFormat.PNG, 100, it)
                }
            }
            compose.onNodeWithText("DEMO SESSION").assertIsDisplayed()
            capture("01-overview-tablet.png")
            compose.onNode(hasText("Arena") and hasClickAction()).performClick().assertIsSelected()
            compose.onNodeWithText("Task 1\nExplore").assertIsNotEnabled()
            capture("02-arena-tablet.png")
            compose.runOnIdle { controller.selectObstacle("B4") }
            compose.onNodeWithText("Image face: North (N)").assertIsDisplayed()
            compose.onNodeWithText("Image ID: 11").assertIsDisplayed()
            compose.onNodeWithContentDescription("Image face: North (N)").assertIsSelected()
            capture("06-arena-selected-face.png")
            compose.onNode(hasText("Controls") and hasClickAction()).performClick().assertIsSelected()
            compose.onNodeWithText("TASK 2").assertIsDisplayed()
            compose.onNodeWithText("Task 2\nFastest path").assertIsEnabled()
            capture("03-controls-tablet.png")
            compose.runOnIdle { compact = true }
            compose.onNode(hasText("Overview") and hasClickAction()).performClick().assertIsSelected()
            compose.onNodeWithText("Bluetooth connection").performScrollTo().assertIsDisplayed()
            capture("04-overview-compact.png")
            compose.onNode(hasText("Controls") and hasClickAction()).performClick().assertIsSelected()
            compose.onNodeWithText("Task 2\nFastest path").performScrollTo().assertIsEnabled()
            capture("05-controls-compact.png")
        } finally {
            compose.runOnIdle { controller.close() }
            store.close().get(10, TimeUnit.SECONDS); history.close().get(10, TimeUnit.SECONDS)
            context.deleteSharedPreferences(name); context.deleteDatabase("$name.db")
        }
    }
}
