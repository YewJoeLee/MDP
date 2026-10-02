package com.example.mdpandroid

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.size
import androidx.compose.ui.Modifier
import androidx.compose.ui.test.*
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.compose.ui.unit.dp
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.ArenaScreen
import com.example.mdpandroid.ui.AssessmentCommandCard
import com.example.mdpandroid.ui.theme.MDPAndroidTheme
import java.util.UUID
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class AssessmentButtonsTest {
    @get:Rule val compose = createComposeRule()

    @Test fun arenaExploreIsBelowSendAndUsesTheSameReadinessChecks() = withController { controller ->
        renderArena(controller)
        val explore = compose.onNodeWithText("Task 1\nExplore")
        val send = compose.onNodeWithText("Send\narena")
        explore.assertIsNotEnabled()
        compose.runOnIdle {
            controller.addObstacle(GridPoint(5, 5))
            controller.setObstacleFace("B1", Face.N)
        }
        explore.assertIsNotEnabled()
        send.performClick()
        explore.assertIsEnabled()
        assertTrue(explore.fetchSemanticsNode().boundsInRoot.top >= send.fetchSemanticsNode().boundsInRoot.bottom)
        explore.performClick()
        compose.runOnIdle {
            assertEquals(RobotCommand.BEGIN_EXPLORE, controller.state.taskRun?.command)
            assertEquals(TaskPhase.RUNNING, controller.state.taskRun?.phase)
            assertEquals("Demo command: beginExplore", controller.state.statusMessages.last().text)
        }
        explore.assertIsNotEnabled()
        send.assertIsNotEnabled()
    }

    @Test fun portraitArenaAlsoStartsExploreAfterSending() = withController { controller ->
        compose.runOnIdle {
            controller.addObstacle(GridPoint(5, 5))
            controller.setObstacleFace("B1", Face.N)
        }
        renderArena(controller, portrait = true)
        compose.onNodeWithText("Send arena").performScrollTo().performClick()
        compose.onNodeWithText("Task 1\nExplore").performScrollTo().performClick()
        compose.runOnIdle { assertEquals(RobotCommand.BEGIN_EXPLORE, controller.state.taskRun?.command) }
    }

    @Test fun controlsSendExploreAndFastestHaveDistinctCommands() = withController { controller ->
        compose.runOnIdle {
            controller.addObstacle(GridPoint(5, 5))
            controller.setObstacleFace("B1", Face.S)
        }
        compose.setContent {
            MDPAndroidTheme {
                AssessmentCommandCard(controller.state, controller::moveRobot, controller::sendArenaSnapshot, controller::finishDemoTask)
            }
        }
        val explore = compose.onNodeWithText("Task 1\nExplore")
        val fastest = compose.onNodeWithText("Task 2\nFastest path")
        val send = compose.onNodeWithText("Send current arena to robot")
        explore.assertIsNotEnabled()
        fastest.assertIsNotEnabled()
        send.performClick()
        compose.runOnIdle { assertEquals("[(1, 5, 5, \"S\")]", controller.state.sentArena) }
        fastest.assertIsEnabled().performClick()
        compose.runOnIdle {
            assertEquals(RobotCommand.BEGIN_FASTEST, controller.state.taskRun?.command)
            assertEquals("Demo command: beginFastest", controller.state.statusMessages.last().text)
        }
        explore.assertIsNotEnabled()
        fastest.assertIsNotEnabled()
        compose.onNodeWithText("Finish demo task").performClick()
        explore.assertIsNotEnabled()
        send.performClick()
        explore.performClick()
        compose.runOnIdle { assertEquals(RobotCommand.BEGIN_EXPLORE, controller.state.taskRun?.command) }
    }

    private fun renderArena(controller: BluetoothController, portrait: Boolean = false) {
        compose.setContent {
            MDPAndroidTheme {
                Box(if (portrait) Modifier.size(360.dp, 640.dp) else Modifier) {
                    ArenaScreen(
                        padding = PaddingValues(), state = controller.state, logHistory = controller.logHistory,
                        controlsEnabled = controller.state.canDrive, onCommand = controller::moveRobot,
                        onAddObstacle = controller::addObstacle, onMoveObstacle = controller::moveObstacle,
                        onRemoveObstacle = controller::removeObstacle, onSelectObstacle = controller::selectObstacle,
                        onSetObstacleFace = controller::setObstacleFace, onSetRobotStart = controller::setRobotStart,
                        onSetRobotFace = controller::setRobotFace, onSetRobotPose = controller::setRobotPose,
                        onSendArena = controller::sendArenaSnapshot, onResetArena = controller::resetArena,
                        onClearObstacleTarget = controller::clearObstacleTarget,
                        onClearObstacleSelection = controller::clearObstacleSelection
                    )
                }
            }
        }
    }

    private fun withController(test: (BluetoothController) -> Unit) {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val name = "assessment-buttons-${UUID.randomUUID()}"
        val store = ArenaStore(context, name)
        val history = ActivityLogHistory(context, "$name.db")
        lateinit var controller: BluetoothController
        compose.runOnIdle { controller = BluetoothController(context, history, store); controller.setDemoMode(true) }
        try { test(controller) }
        finally {
            compose.runOnIdle { controller.close() }
            store.close().get(10, TimeUnit.SECONDS)
            history.close().get(10, TimeUnit.SECONDS)
            context.deleteSharedPreferences(name)
            context.deleteDatabase("$name.db")
        }
    }
}
