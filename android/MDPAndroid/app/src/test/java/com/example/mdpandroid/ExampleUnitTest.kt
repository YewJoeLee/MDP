package com.example.mdpandroid

import org.junit.Test

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue

class ProtocolAndActivityTest {
    @Test
    fun targetRecognitionAcceptsNumericObstacleIdsAndKeepsKnownFaceWhenNoneIsProvided() {
        val message = parseProtocolMessage("TARGET,1,12") as ProtocolMessage.Target
        val update = applyTargetRecognition(
            listOf(Obstacle(id = "B1", x = 10, y = 10, targetFace = Face.E)),
            message
        )

        assertEquals("B1", message.obstacleId)
        assertNull(message.face)
        assertTrue(update.matched)
        assertEquals("12", update.obstacles.single().targetId)
        assertEquals(Face.E, update.obstacles.single().targetFace)
    }

    @Test
    fun targetRecognitionUsesFaceOnlyWhenItIsProvided() {
        val message = parseProtocolMessage("TARGET,1,12,N") as ProtocolMessage.Target
        val update = applyTargetRecognition(
            listOf(Obstacle(id = "B1", x = 10, y = 10, targetFace = Face.E)),
            message
        )

        assertEquals(Face.N, message.face)
        assertTrue(update.matched)
        assertEquals("12", update.obstacles.single().targetId)
        assertEquals(Face.N, update.obstacles.single().targetFace)
    }

    @Test
    fun targetForMissingObstacleIsReportedInsteadOfSilentlyApplied() {
        val existing = listOf(Obstacle(id = "B1", x = 10, y = 10))

        val update = applyTargetRecognition(
            existing, parseProtocolMessage("TARGET,2,11") as ProtocolMessage.Target
        )

        assertFalse(update.matched)
        assertEquals(existing, update.obstacles)
    }

    @Test
    fun clearingAFacePreservesTheReceivedTargetId() {
        val cleared = clearObstacleFace(
            listOf(Obstacle(id = "B1", x = 10, y = 10, targetId = "17", targetFace = Face.N)),
            "B1"
        ).single()

        assertEquals("17", cleared.targetId)
        assertNull(cleared.targetFace)
    }

    @Test
    fun combinedActivityLogUsesArrivalOrderWhenEntriesShareATimestamp() {
        val entries = mergeActivityLog(
            statusMessages = listOf(StatusMessage("12:00:00", "Connected", order = 2)),
            receivedRawMessages = listOf(StatusMessage("12:00:00", "ROBOT,6,2,W", order = 1))
        )

        assertEquals(ActivitySource.RECEIVED, entries.first().source)
        assertEquals(ActivitySource.STATUS, entries.last().source)
    }

    @Test
    fun malformedRobotMessageIsIgnored() {
        assertNull(parseProtocolMessage("ROBOT,6,2,invalid"))
        assertTrue(parseProtocolMessage("TARGET,B1,5,N") is ProtocolMessage.Target)
    }

    @Test
    fun facesTurnInTheExpectedDirection() {
        assertEquals(Face.W, Face.N.turnLeft())
        assertEquals(Face.E, Face.N.turnRight())
        assertEquals(Face.N, Face.W.turnRight())
        assertEquals(1, Face.E.dx)
        assertEquals(-1, Face.S.dy)
    }
}
