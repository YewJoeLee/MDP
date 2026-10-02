package com.example.mdpandroid

import android.Manifest
import android.bluetooth.BluetoothManager
import android.content.Context
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import java.io.ByteArrayOutputStream
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

/** Captures the controller's real queued writes without connecting to a physical robot. */
@RunWith(AndroidJUnit4::class)
class AssessmentTransmissionTest {
    private val instrumentation get() = InstrumentationRegistry.getInstrumentation()
    private fun main(action: () -> Unit) = instrumentation.runOnMainSync(action)

    @Test fun sendArenaAndBothTasksEmitExactPiPayloadsAndCompleteIndependently() = withTransport { controller, output ->
        val payload = "[(1, 5, 5, \"N\"), (2, 9, 9, \"W\")]"
        main {
            controller.addObstacle(GridPoint(5, 5))
            controller.setObstacleFace("B1", Face.N)
            controller.addObstacle(GridPoint(9, 9))
            controller.setObstacleFace("B2", Face.W)
            controller.sendArenaSnapshot()
        }
        await { controller.state.sentArena == payload && !controller.state.arenaSendPending }
        main { controller.moveRobot(RobotCommand.BEGIN_EXPLORE) }
        await { controller.state.taskRun?.phase == TaskPhase.RUNNING }
        assertEquals(listOf(payload, "beginExplore"), output.records())
        main {
            controller.moveRobot(RobotCommand.BEGIN_FASTEST)
            controller.sendArenaSnapshot()
        }
        assertEquals(listOf(payload, "beginExplore"), output.records())
        main {
            receive(controller, "taskComplete\n")
            assertNull(controller.state.taskRun)
            assertNull(controller.state.sentArena)
            controller.sendArenaSnapshot()
        }
        await { controller.state.sentArena == payload && !controller.state.arenaSendPending }
        main { controller.moveRobot(RobotCommand.BEGIN_FASTEST) }
        await { controller.state.taskRun?.phase == TaskPhase.RUNNING }
        assertEquals(listOf(payload, "beginExplore", payload, "beginFastest"), output.records())
        main { receive(controller, "taskComplete\n"); assertNull(controller.state.taskRun) }
    }

    @Test fun missingFacesAndPendingArenaWritesCannotStartEitherTask() = withTransport { controller, output ->
        main {
            controller.addObstacle(GridPoint(5, 5))
            controller.sendArenaSnapshot()
            controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
            controller.moveRobot(RobotCommand.BEGIN_FASTEST)
            assertNull(controller.state.taskRun)
        }
        assertTrue(output.records().isEmpty())
        output.release = CountDownLatch(1)
        try {
            main {
                controller.setObstacleFace("B1", Face.S)
                controller.sendArenaSnapshot()
                assertTrue(controller.state.arenaSendPending)
                controller.moveRobot(RobotCommand.BEGIN_EXPLORE)
                controller.moveRobot(RobotCommand.BEGIN_FASTEST)
                assertNull(controller.state.taskRun)
            }
        } finally { output.release!!.countDown() }
        await { !controller.state.arenaSendPending }
        assertEquals(listOf("[(1, 5, 5, \"S\")]"), output.records())
    }

    @Test fun failedTaskWriteKeepsDeliveryUncertainAndRequiresReconnection() = withTransport { controller, output ->
        main {
            controller.addObstacle(GridPoint(5, 5))
            controller.setObstacleFace("B1", Face.N)
            controller.sendArenaSnapshot()
        }
        await { controller.state.sentArena != null && !controller.state.arenaSendPending }
        output.reject = "beginFastest"
        main { controller.moveRobot(RobotCommand.BEGIN_FASTEST) }
        await { !controller.state.connected && controller.state.taskRun?.phase == TaskPhase.DELIVERY_UNKNOWN }
        assertEquals(listOf("[(1, 5, 5, \"N\")]"), output.records())
        main {
            assertNull(controller.state.sentArena)
            assertNotNull(controller.state.taskStartIssue())
            assertFalse(controller.state.canSendArena)
        }
    }

    private fun await(condition: () -> Boolean) {
        val deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(5)
        var satisfied = false
        while (!satisfied && System.nanoTime() < deadline) {
            main { satisfied = condition() }
            if (!satisfied) Thread.sleep(10)
        }
        assertTrue("Controller did not finish the expected send", satisfied)
    }

    private fun receive(controller: BluetoothController, chunk: String) {
        BluetoothController::class.java.getDeclaredMethod("consumeIncomingChunk", String::class.java).apply {
            isAccessible = true
            invoke(controller, chunk)
        }
    }

    private fun withTransport(test: (BluetoothController, Capture) -> Unit) {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val name = "assessment-transport-${UUID.randomUUID()}"
        val store = ArenaStore(context, name)
        val history = ActivityLogHistory(context, "$name.db")
        lateinit var controller: BluetoothController
        if (Build.VERSION.SDK_INT >= 31) instrumentation.uiAutomation.adoptShellPermissionIdentity(Manifest.permission.BLUETOOTH_CONNECT)
        try {
            val socket = (context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
                .getRemoteDevice("00:11:22:33:44:55")
                .createRfcommSocketToServiceRecord(UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"))
            val output = Capture()
            main {
                controller = BluetoothController(context, history, store)
                // Keep production session checks, queueing, serialization and success callbacks.
                // Substitute only the disconnected socket identity and its external output stream.
                listOf("socket" to socket, "output" to output).forEach { (field, value) ->
                    BluetoothController::class.java.getDeclaredField(field).apply { isAccessible = true; set(controller, value) }
                }
                BluetoothController::class.java.getDeclaredMethod("setState", AppState::class.java).apply {
                    isAccessible = true
                    invoke(controller, controller.state.copy(connected = true))
                }
            }
            try { test(controller, output) } finally { main { controller.close() } }
        } finally {
            store.close().get(10, TimeUnit.SECONDS)
            history.close().get(10, TimeUnit.SECONDS)
            context.deleteSharedPreferences(name)
            context.deleteDatabase("$name.db")
            if (Build.VERSION.SDK_INT >= 31) instrumentation.uiAutomation.dropShellPermissionIdentity()
        }
    }

    private class Capture : ByteArrayOutputStream() {
        @Volatile var release: CountDownLatch? = null
        @Volatile var reject: String? = null
        private val messages = mutableListOf<String>()
        override fun write(bytes: ByteArray, offset: Int, length: Int) {
            release?.await(1, TimeUnit.SECONDS)
            val text = String(bytes, offset, length, Charsets.UTF_8)
            if (text == reject) throw java.io.IOException("Disconnected during task write")
            synchronized(this) {
                messages += text
                super.write(bytes, offset, length)
            }
        }
        @Synchronized fun records(): List<String> = messages.toList()
    }
}
