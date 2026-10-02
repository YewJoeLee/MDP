package com.example.mdpandroid

import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class TaskReliabilityRegressionTest {
    @Test fun taskCannotStartWithoutAnArena() = withController { controller ->
        controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
        assertNoStart(controller, RobotCommand.BEGIN_EXPLORE)
    }

    @Test fun taskCannotStartWithMissingFaces() = withController { controller ->
        controller.addObstacle(GridPoint(5, 5))
        controller.sendArenaSnapshot()
        controller.moveRobot(RobotCommand.BEGIN_FASTEST)
        assertNoStart(controller, RobotCommand.BEGIN_FASTEST)
    }

    @Test fun arenaSendRejectsMissingFacesAndListsTheAffectedObstacles() = withController { controller ->
        controller.addObstacle(GridPoint(5, 5))
        controller.setObstacleFace("B1", Face.N)
        controller.addObstacle(GridPoint(9, 9))
        controller.addObstacle(GridPoint(12, 12))

        controller.sendArenaSnapshot()

        assertEquals(null, controller.state.sentArena)
        assertFalse(controller.state.statusMessages.any { it.text.startsWith("Demo only: [") })
        assertTrue(controller.state.statusMessages.last().text.contains("B2, B3"))
    }

    @Test fun settingMissingFacesAllowsTheArenaToBeSent() = withController { controller ->
        controller.addObstacle(GridPoint(5, 5))
        controller.sendArenaSnapshot()
        controller.setObstacleFace("B1", Face.S)
        controller.sendArenaSnapshot()

        assertEquals("[(1, 5, 5, \"S\")]", controller.state.sentArena)
        assertEquals(1, controller.state.statusMessages.count { it.text.startsWith("Demo only: [") })
    }

    @Test fun taskCannotStartBeforeSendingTheCurrentLayout() = withController { controller ->
        controller.addObstacle(GridPoint(5, 5))
        controller.setObstacleFace("B1", Face.N)
        controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
        assertNoStart(controller, RobotCommand.BEGIN_EXPLORE)
    }

    @Test fun runningTaskAllowsLocalArenaEdits() = withController { controller ->
        controller.addObstacle(GridPoint(5, 5))
        controller.setObstacleFace("B1", Face.N)
        controller.sendArenaSnapshot()
        controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
        val run = controller.state.taskRun

        controller.addObstacle(GridPoint(9, 9))
        controller.moveObstacle("B1", 7, 7)
        assertEquals(GridPoint(7, 7), controller.state.obstacles.first().let { GridPoint(it.x, it.y) })
        controller.setObstacleFace("B1", Face.S)
        assertEquals(Face.S, controller.state.obstacles.first().targetFace)
        controller.clearObstacleTarget("B1")
        assertEquals(null, controller.state.obstacles.first().targetFace)
        controller.removeObstacle("B1")
        controller.setRobotStart(2, 2)
        controller.setRobotFace(Face.E)
        controller.setRobotPose(3, 3, Face.W)

        assertEquals(run, controller.state.taskRun)
        assertEquals(listOf(Obstacle("B2", 9, 9)), controller.state.obstacles)
        assertEquals(RobotState(3, 3, Face.W), controller.state.robot)
    }

    @Test fun runningTaskRejectsDuplicateStartsDriveAndArenaSends() = withController { controller ->
        prepareAndStart(controller)
        controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
        controller.moveRobot(RobotCommand.BEGIN_FASTEST)
        controller.moveRobot(RobotCommand.FORWARD)
        controller.sendArenaSnapshot()
        assertEquals(1, controller.state.statusMessages.count { it.text == RobotMessages.demoCommand(RobotCommand.BEGIN_EXPLORE) })
        assertNoStart(controller, RobotCommand.BEGIN_FASTEST)
        assertNoStart(controller, RobotCommand.FORWARD)
        assertEquals(RobotState(1, 1, Face.N), controller.state.robot)
        assertEquals(1, controller.state.statusMessages.count { it.text.startsWith("Demo only: [") })
    }

    @Test fun finishingDemoRequiresAResend() = withController { controller ->
        prepareAndStart(controller)
        controller.finishDemoTask()
        controller.addObstacle(GridPoint(9, 9))
        assertEquals(2, controller.state.obstacles.size)
        assertFalse(controller.state.taskActive)
        assertTrue(controller.state.taskStartIssue() != null)
    }

    @Test fun remoteTelemetryCanUpdateTheArenaDuringATask() = withController { controller ->
        prepareAndStart(controller)
        receive(controller, "ADD,2,(9,9)")
        receive(controller, "SUB,2")
        receive(controller, "FACE,1,S")
        receive(controller, "TARGET,1,11,S")
        receive(controller, "ROBOT,8,8,E")
        assertEquals(listOf(Obstacle("B1", 5, 5, targetId = "11", targetFace = Face.S)), controller.state.obstacles)
        assertEquals(RobotState(8, 8, Face.E), controller.state.robot)
    }

    @Test fun splitTaskCompletionEndsTheTaskOnlyAfterTheNewline() = withController { controller ->
        prepareAndStart(controller)
        // Restore a live task whose delivery became uncertain after process recreation.
        controller.restoreArena(controller.state.arenaSnapshot().copy(
            taskRun = TaskRun(RobotCommand.BEGIN_EXPLORE, TaskPhase.RUNNING)
        ))
        receive(controller, "TARGET,1,11")
        receive(controller, "ROBOT,8,8,E")
        val arena = controller.state.obstacles
        receiveChunk(controller, "taskCom")
        receiveChunk(controller, "plete")
        assertTrue(controller.state.taskActive)
        receiveChunk(controller, "\n")
        assertFalse(controller.state.taskActive)
        assertEquals(arena, controller.state.obstacles)
        assertEquals(RobotState(8, 8, Face.E), controller.state.robot)
        assertEquals(null, controller.state.sentArena)
        controller.addObstacle(GridPoint(12, 12))
        assertEquals(2, controller.state.obstacles.size)
    }

    @Test fun splitPiCompletionEndsEitherTaskAndRequiresAFreshArenaSend() = withController { controller ->
        controller.addObstacle(GridPoint(5, 5))
        controller.setObstacleFace("B1", Face.N)
        for (command in listOf(RobotCommand.BEGIN_EXPLORE, RobotCommand.BEGIN_FASTEST)) {
            controller.sendArenaSnapshot()
            controller.moveRobot(command)
            assertTrue(controller.state.taskActive)
            receiveChunk(controller, "MSG,Run com")
            receiveChunk(controller, "plete. Done !")
            assertTrue(controller.state.taskActive)
            receiveChunk(controller, "\n")
            assertFalse(controller.state.taskActive)
            assertEquals(null, controller.state.sentArena)
            assertTrue(controller.state.taskStartIssue() != null)
            assertEquals(listOf(Obstacle("B1", 5, 5, targetFace = Face.N)), controller.state.obstacles)
            assertEquals("MSG,Run complete. Done !", controller.state.receivedRawLog.last().text)
        }
    }

    @Test fun otherMessagesCannotUnlockARunningTask() = withController { controller ->
        prepareAndStart(controller)
        receive(controller, "MSG,taskComplete")
        receive(controller, "MSG,Run complete")
        receive(controller, "MSG,Run complete. Done ! Waiting")
        receive(controller, "taskComplete,1")
        receive(controller, "taskCompleteLater")
        receive(controller, "TARGET,1,bb")
        assertTrue(controller.state.taskActive)
    }

    @Test fun repeatedCompletionDoesNotInvalidateArenaSentForTheNextRun() = withController { controller ->
        prepareAndStart(controller)
        receive(controller, "MSG,Run complete. Done !")
        assertFalse(controller.state.taskActive)
        controller.sendArenaSnapshot()
        receive(controller, "MSG,Run complete. Done !")
        assertEquals("[(1, 5, 5, \"N\")]", controller.state.sentArena)
        assertEquals(null, controller.state.taskStartIssue())
        controller.moveRobot(RobotCommand.BEGIN_FASTEST)
        assertTrue(controller.state.taskActive)
    }

    private fun prepareAndStart(controller: BluetoothController) {
        controller.addObstacle(GridPoint(5, 5))
        controller.setObstacleFace("B1", Face.N)
        controller.sendArenaSnapshot()
        controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
        assertTrue(controller.state.taskActive)
    }

    private fun receive(controller: BluetoothController, line: String) {
        // Feed a complete transport record into the actual controller handler.
        val handler = BluetoothController::class.java.getDeclaredMethod("parseIncoming", String::class.java)
        handler.isAccessible = true
        handler.invoke(controller, line)
    }

    private fun receiveChunk(controller: BluetoothController, chunk: String) {
        val handler = BluetoothController::class.java.getDeclaredMethod("consumeIncomingChunk", String::class.java)
        handler.isAccessible = true
        handler.invoke(controller, chunk)
    }

    private fun assertNoStart(controller: BluetoothController, command: RobotCommand) {
        assertFalse(controller.state.statusMessages.any { it.text == RobotMessages.demoCommand(command) })
    }

    private fun withController(test: (BluetoothController) -> Unit) {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val databaseName = "task-test-${java.util.UUID.randomUUID()}.db"
        var history: ActivityLogHistory? = null
        val arenaStore = ArenaStore(context, databaseName)
        try {
            InstrumentationRegistry.getInstrumentation().runOnMainSync {
                val logs = ActivityLogHistory(context, databaseName).also { history = it }
                val controller = BluetoothController(context, logs, arenaStore)
                try {
                    controller.setDemoMode(true)
                    test(controller)
                } finally { controller.close() }
            }
        } finally {
            history?.close()?.get(10, java.util.concurrent.TimeUnit.SECONDS)
            arenaStore.close().get(10, java.util.concurrent.TimeUnit.SECONDS)
            context.deleteSharedPreferences(databaseName)
            context.deleteDatabase(databaseName)
        }
    }
}
