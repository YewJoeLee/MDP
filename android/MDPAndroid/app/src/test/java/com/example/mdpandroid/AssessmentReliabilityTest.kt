package com.example.mdpandroid

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class AssessmentReliabilityTest {
    @Test
    fun splitRobotCoordinateSurvivesAnIdleGap() {
        val framer = IncomingMessageFramer()
        framer.append("ROBOT,1,1")
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("ROBOT,1,10,N"), framer.append("0,N\n"))
    }

    @Test
    fun splitObstacleCoordinateSurvivesAnIdleGap() {
        val framer = IncomingMessageFramer()
        framer.append("ADD,B1,(10,")
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("ADD,B1,(10,12)"), framer.append("12)\n"))
    }

    @Test
    fun robotStatusWaitsForItsDelimiter() {
        val framer = IncomingMessageFramer()
        framer.append("MSG,rea")
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("MSG,ready"), framer.append("dy\n"))
    }

    @Test
    fun manualMovesBeyondAnyArenaBoundaryAreRejected() {
        for (robot in listOf(RobotState(1, 5, Face.W), RobotState(18, 5, Face.E),
            RobotState(5, 1, Face.S), RobotState(5, 18, Face.N))) {
            val next = nextRobotPose(RobotCommand.FORWARD, robot)!!
            assertNull(localRobotPose(next.x, next.y, next.direction, emptyList()))
        }
    }
}
