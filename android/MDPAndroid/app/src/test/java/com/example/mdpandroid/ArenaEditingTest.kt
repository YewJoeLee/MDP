package com.example.mdpandroid

import org.junit.Assert.*
import org.junit.Test

class ArenaEditingTest {
    private val arena = AppState(obstacles = listOf(Obstacle("B1", 10, 10, "11", Face.N),
        Obstacle("B2", 12, 12)), selectedObstacleId = "B1", robot = RobotState(3, 3, Face.N))

    @Test fun occupiedDropPreservesTheOriginalObstacleAndSendsNothing() {
        for (point in listOf(GridPoint(12, 12), GridPoint(3, 3))) {
            val change = planObstacleMove(arena, "B1", point)
            assertEquals(arena, change.state)
            assertTrue(change.commands.isEmpty())
            assertNotNull(change.message)
        }
    }

    @Test fun outsideDropRemovesOnlyThatObstacleLocally() {
        val change = planObstacleMove(arena, "B1", GridPoint(-1, 10))
        assertEquals(listOf("B2"), change.state.obstacles.map { it.id })
        assertNull(change.state.selectedObstacleId)
        assertTrue(change.commands.isEmpty())
    }

    @Test fun movingAnAnnotatedObstaclePreservesItsFaceWithoutSendingCommands() {
        val change = planObstacleMove(arena, "B1", GridPoint(11, 10))
        assertEquals(11, change.state.obstacles.first().x)
        assertEquals(10, change.state.obstacles.first().y)
        assertEquals(Face.N, change.state.obstacles.first().targetFace)
        assertEquals("11", change.state.obstacles.first().targetId)
        assertTrue(change.commands.isEmpty())
    }

    @Test fun clearingAFaceUpdatesTheLocalLayoutWithoutSendingCommands() {
        val change = planClearObstacleFace(arena, "B1")
        assertNull(change.state.obstacles.first().targetFace)
        assertEquals("11", change.state.obstacles.first().targetId)
        assertTrue(change.commands.isEmpty())
    }

    @Test fun arenaSendContainsOnlyTheExistingPiObstacleList() {
        assertEquals(listOf("[(1, 10, 10, \"N\"), (2, 12, 12, \"\")]"),
            RobotProtocol.arenaSetup(arena.obstacles))
    }

    @Test fun completeSetupUsesTheLatestLocalEditsIncludingClearedFacesAndDeletions() {
        val moved = planObstacleMove(arena, "B1", GridPoint(11, 10)).state
        val cleared = planClearObstacleFace(moved, "B1").state
        val removed = planObstacleMove(cleared, "B2", GridPoint(20, 12)).state

        assertEquals(listOf("[(1, 11, 10, \"\")]"),
            RobotProtocol.arenaSetup(removed.obstacles))
    }

    @Test fun incomingConnectionLeavesDemoAndArmsLiveStop() {
        val connected = AppState(demoMode = true).connectedTo(BluetoothDeviceInfo("peer", "Robot"))
        assertFalse(connected.demoMode)
        assertTrue(connected.connected)
        assertEquals("peer", connected.connectedAddress)
    }

    @Test fun outgoingCommandsKeepTheExistingPiFormat() {
        assertEquals("STOP", ConnectionProtocol.payload("STOP"))
        assertEquals("ROBOT,3,3,N", ConnectionProtocol.payload("ROBOT,3,3,N"))
        assertEquals("[(1, 10, 12, \"N\")]", ConnectionProtocol.payload("[(1, 10, 12, \"N\")]"))
    }
}
