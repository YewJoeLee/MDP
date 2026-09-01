package com.example.mdpandroid

import org.junit.Test

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue

class ProtocolAndActivityTest {
    @Test
    fun targetRecognitionAcceptsNumericObstacleIdsAndKeepsKnownFaceWhenNoneIsProvided() {
        val message = parseProtocolMessage("TARGET,1,12") as ProtocolMessage.Target
        val obstacles = applyTargetRecognition(
            listOf(Obstacle(id = "B1", x = 10, y = 10, targetFace = Face.E)),
            message
        )

        assertEquals("B1", message.obstacleId)
        assertEquals("12", obstacles.single().targetId)
        assertEquals(Face.E, obstacles.single().targetFace)
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
}
