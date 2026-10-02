package com.example.mdpandroid

import androidx.compose.foundation.layout.size
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.toPixelMap
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.test.captureToImage
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.performTouchInput
import androidx.compose.ui.unit.dp
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.ArenaCanvas
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class ArenaGestureRegressionTest {
    @get:Rule
    val compose = createComposeRule()

    private val initial = AppState(obstacles = listOf(Obstacle("B1", 5, 10)))
    private val arena get() = compose.onNodeWithTag("arena")

    @Test
    fun heldObstacleSurvivesRobotAndTargetTelemetryUntilRelease() {
        var state by mutableStateOf(initial)
        val moves = mutableListOf<Triple<String, Int, Int>>()
        compose.setContent { TestArena(state, onMove = { id, x, y -> moves += Triple(id, x, y) }) }

        holdObstacle()
        compose.runOnIdle {
            state = state.copy(
                robot = state.robot.copy(x = 7),
                obstacles = state.obstacles.map { it.copy(targetId = "11") }
            )
        }
        arena.performTouchInput { up() }

        compose.runOnIdle { assertEquals(listOf(Triple("B1", 8, 10)), moves) }
    }

    @Test
    fun releaseUsesCallbackFromCurrentMode() {
        var demo by mutableStateOf(false)
        val liveMoves = mutableListOf<String>()
        val demoMoves = mutableListOf<String>()
        compose.setContent {
            val move: (String, Int, Int) -> Unit = if (demo) {
                { id, _, _ -> demoMoves += id }
            } else {
                { id, _, _ -> liveMoves += id }
            }
            TestArena(initial, onMove = move)
        }

        holdObstacle()
        compose.runOnIdle { demo = true }
        arena.performTouchInput { up() }

        compose.runOnIdle {
            assertTrue(liveMoves.isEmpty())
            assertEquals(listOf("B1"), demoMoves)
        }
    }

    @Test
    fun obstacleRemovedDuringHoldDoesNotCommitRelease() {
        var state by mutableStateOf(initial)
        val moves = mutableListOf<String>()
        compose.setContent { TestArena(state, onMove = { id, _, _ -> moves += id }) }

        holdObstacle()
        compose.runOnIdle { state = state.copy(obstacles = emptyList()) }
        arena.performTouchInput { up() }

        compose.runOnIdle { assertTrue(moves.isEmpty()) }
    }

    @Test
    fun taskStartingDuringHoldStillCommitsRelease() {
        var state by mutableStateOf(initial)
        val moves = mutableListOf<String>()
        compose.setContent { TestArena(state, onMove = { id, _, _ -> moves += id }) }
        holdObstacle()
        compose.runOnIdle { state = state.copy(taskRun = TaskRun(RobotCommand.BEGIN_EXPLORE, TaskPhase.STARTING)) }
        arena.performTouchInput { up() }
        compose.runOnIdle { assertEquals(listOf("B1"), moves) }
    }

    @Test
    fun runningTaskAllowsObstacleDrags() {
        val moves = mutableListOf<String>()
        val running = initial.copy(taskRun = TaskRun(RobotCommand.BEGIN_EXPLORE, TaskPhase.RUNNING))
        compose.setContent { TestArena(running, onMove = { id, _, _ -> moves += id }) }
        holdObstacle()
        arena.performTouchInput { up() }
        compose.runOnIdle { assertEquals(listOf("B1"), moves) }
    }

    @Test
    fun cancelledDragRestoresOriginalWithoutCommittingMove() {
        val moves = mutableListOf<String>()
        compose.setContent { TestArena(initial, onMove = { id, _, _ -> moves += id }) }
        val before = arena.captureToImage()
        val sampleX = (before.width * 5.25f / 20).toInt()
        val sampleY = (before.height * 9.25f / 20).toInt()
        val originalColor = before.toPixelMap()[sampleX, sampleY]

        holdObstacle()
        assertNotEquals(originalColor, arena.captureToImage().toPixelMap()[sampleX, sampleY])
        arena.performTouchInput { cancel() }

        compose.runOnIdle { assertTrue(moves.isEmpty()) }
        assertEquals(originalColor, arena.captureToImage().toPixelMap()[sampleX, sampleY])
    }

    private fun holdObstacle() {
        arena.performTouchInput {
            down(Offset(width * 5.5f / 20, height * 9.5f / 20))
            moveTo(Offset(width * 8.5f / 20, height * 9.5f / 20))
        }
    }

    @androidx.compose.runtime.Composable
    private fun TestArena(state: AppState, onMove: (String, Int, Int) -> Unit) {
        ArenaCanvas(
            state = state,
            onAddObstacle = {},
            onMoveObstacle = onMove,
            onSelectObstacle = {},
            onSetObstacleFace = { _, _ -> },
            onSetRobotStart = { _, _ -> },
            onSetRobotFace = {},
            onSelectRobot = {},
            modifier = Modifier.size(300.dp).testTag("arena")
        )
    }
}
