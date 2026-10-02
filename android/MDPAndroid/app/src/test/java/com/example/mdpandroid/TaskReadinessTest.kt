package com.example.mdpandroid

import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class TaskReadinessTest {
    private val ready = AppState(
        connected = true,
        obstacles = listOf(Obstacle("B1", 5, 5, targetFace = Face.N)),
        sentArena = "[(1, 5, 5, \"N\")]"
    )

    @Test fun sentCurrentArenaEnablesTaskStarts() {
        assertNull(ready.taskStartIssue())
    }

    @Test fun changingCoordinatesFacesOrRemovingObstaclesRequiresAnotherSend() {
        assertNotNull(ready.copy(obstacles = listOf(Obstacle("B1", 6, 5, targetFace = Face.N))).taskStartIssue())
        assertNotNull(ready.copy(obstacles = listOf(Obstacle("B1", 5, 5, targetFace = Face.S))).taskStartIssue())
        assertNotNull(ready.copy(obstacles = emptyList()).taskStartIssue())
    }

    @Test fun RecognitionResultsDoNotInvalidateTheSentLayout() {
        assertNull(ready.copy(obstacles = listOf(Obstacle("B1", 5, 5, targetId = "11", targetFace = Face.N))).taskStartIssue())
    }

    @Test fun reconnectRequiresResendingTheArena() {
        val reconnected = ready.connectedTo(BluetoothDeviceInfo("00:11:22:33:44:55", "Pi"))
        assertNull(reconnected.sentArena)
        assertNotNull(reconnected.taskStartIssue())
    }

    @Test fun pendingWritesCannotRaceATaskStart() {
        assertNotNull(ready.copy(arenaSendPending = true).taskStartIssue())
        assertNotNull(ready.copy(driveCommandPending = true).taskStartIssue())
    }

    @Test fun robotMustHaveAValidFootprintBeforeStarting() {
        assertNotNull(ready.copy(robot = RobotState(5, 5)).taskStartIssue())
        assertNotNull(ready.copy(robot = RobotState(0, 0)).taskStartIssue())
    }

    @Test fun everyActivePhaseBlocksDriveSendAndFurtherStarts() {
        TaskPhase.entries.forEach { phase ->
            val running = ready.copy(taskRun = TaskRun(RobotCommand.BEGIN_EXPLORE, phase))
            assertTrue(running.taskActive)
            assertFalse(running.canDrive)
            assertFalse(running.canSendArena)
            assertNotNull(running.taskStartIssue())
        }
    }

    @Test fun processRestorationRetainsTaskWithoutRestoringSendReadiness() {
        val running = ready.copy(taskRun = TaskRun(RobotCommand.BEGIN_EXPLORE, TaskPhase.RUNNING))
        val restored = AppState().withArenaSnapshot(running.arenaSnapshot())
        assertTrue(restored.taskActive)
        assertNull(restored.sentArena)
        assertNotNull(restored.taskStartIssue())
    }
}
