package com.example.mdpandroid

import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.ObjectInputStream
import java.io.ObjectOutputStream
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class ArenaSnapshotTest {
    @Test
    fun savedArenaRestoresTargetsFacesSelectionAndRobotPose() {
        val original = AppState(
            obstacles = listOf(
                Obstacle("B1", 4, 5, targetId = "11", targetFace = Face.N),
                Obstacle("B2", 8, 9)
            ),
            selectedObstacleId = "B1",
            robot = RobotState(6, 7, Face.E),
            connected = true
        )
        val bytes = ByteArrayOutputStream().also { output ->
            ObjectOutputStream(output).use { it.writeObject(original.arenaSnapshot()) }
        }.toByteArray()
        val snapshot = ObjectInputStream(ByteArrayInputStream(bytes)).use {
            it.readObject() as ArenaSnapshot
        }

        val restored = AppState().withArenaSnapshot(snapshot)

        assertEquals(original.obstacles, restored.obstacles)
        assertEquals("B1", restored.selectedObstacleId)
        assertEquals(original.robot, restored.robot)
        assertNull(restored.connectedAddress)
        assertEquals(false, restored.connected)
    }
}
