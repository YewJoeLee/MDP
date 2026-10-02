package com.example.mdpandroid

import android.content.Context
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import java.util.UUID
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class ArenaPersistenceTest {
    @Test fun restartRestoresCompleteMapAndResetStaysCleared() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val name = "arena-test-${UUID.randomUUID()}"
        val stores = mutableListOf<ArenaStore>()
        val histories = mutableListOf<ActivityLogHistory>()
        fun session(block: (BluetoothController) -> Unit) {
            val store = ArenaStore(context, name).also { stores += it }
            val history = ActivityLogHistory(context, "$name.db").also { histories += it }
            InstrumentationRegistry.getInstrumentation().runOnMainSync {
                val controller = BluetoothController(context, history, store)
                try { block(controller) } finally { controller.close() }
            }
            store.close().get(10, TimeUnit.SECONDS)
            history.close().get(10, TimeUnit.SECONDS)
        }
        try {
            session { controller ->
                controller.setDemoMode(true)
                controller.addObstacle(GridPoint(5, 5))
                controller.setObstacleFace("B1", Face.S)
                controller.restoreArena(controller.state.arenaSnapshot().copy(
                    obstacles = listOf(Obstacle("B1", 5, 5, "11", Face.S)),
                    robot = RobotState(10, 10, Face.W)
                ))
                controller.sendArenaSnapshot()
                controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
            }
            session { controller ->
                assertEquals(listOf(Obstacle("B1", 5, 5, "11", Face.S)), controller.state.obstacles)
                assertEquals(RobotState(10, 10, Face.W), controller.state.robot)
                assertNull(controller.state.selectedObstacleId)
                assertFalse(controller.state.connected)
                assertNull(controller.state.sentArena)
                assertNull(controller.state.taskRun)
                controller.setDemoMode(true)
                controller.sendArenaSnapshot()
                controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
                val run = controller.state.taskRun
                val logCount = controller.state.statusMessages.size
                controller.resetArena()
                assertEquals(run, controller.state.taskRun)
                assertNull(controller.state.sentArena)
                assertNull(controller.state.selectedObstacleId)
                assertTrue(controller.state.obstacles.isEmpty())
                assertEquals(RobotState(), controller.state.robot)
                assertEquals(logCount + 1, controller.state.statusMessages.size)
                assertFalse(controller.state.statusMessages.last().text.startsWith("Demo only:"))
            }
            session { controller ->
                // Android can restore an older Activity bundle after a later map reset.
                controller.restoreSession(ArenaSnapshot(
                    listOf(Obstacle("B9", 9, 9, targetFace = Face.N)), "B9", RobotState(4, 4, Face.E)
                ))
                assertTrue(controller.state.obstacles.isEmpty())
                assertEquals(RobotState(1, 1, Face.N), controller.state.robot)
                assertNull(controller.state.selectedObstacleId)
            }
        } finally {
            stores.forEach { it.close().get(10, TimeUnit.SECONDS) }
            histories.forEach { it.close().get(10, TimeUnit.SECONDS) }
            context.deleteSharedPreferences(name)
            context.deleteDatabase("$name.db")
        }
    }

    @Test fun invalidSavedMapFallsBackWithoutCrashing() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val name = "invalid-arena-${UUID.randomUUID()}"
        context.getSharedPreferences(name, Context.MODE_PRIVATE).edit().putString("map", "invalid json").commit()
        val store = ArenaStore(context, name)
        try {
            assertNull(store.load())
            assertNotNull(store.loadError)
        } finally {
            store.close().get(10, TimeUnit.SECONDS)
            context.deleteSharedPreferences(name)
        }
    }
}
