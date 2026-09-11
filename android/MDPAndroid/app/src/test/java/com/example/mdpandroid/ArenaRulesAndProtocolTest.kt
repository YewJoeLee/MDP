package com.example.mdpandroid

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ArenaRulesAndProtocolTest {
    @Test
    fun protocolFormatsExactWireCommands() {
        assertEquals("f", RobotProtocol.command(RobotCommand.FORWARD))
        assertEquals("tr", RobotProtocol.command(RobotCommand.TURN_RIGHT))
        assertEquals("ADD,B2,(3,4)", RobotProtocol.addObstacle("B2", GridPoint(3, 4)))
        assertEquals("FACE,B2,N,(3,4)", RobotProtocol.setObstacleFace("B2", Face.N, GridPoint(3, 4)))
        assertEquals("ROBOT,6,7,E", RobotProtocol.robotPose(RobotState(6, 7, Face.E)))
    }

    @Test
    fun localRobotPoseIsRejectedWhenItsFootprintWouldOverlapAnObstacle() {
        val obstacles = listOf(Obstacle("B1", x = 7, y = 7))

        assertNull(localRobotPose(x = 6, y = 6, direction = Face.N, obstacles = obstacles))
    }

    @Test
    fun localRobotPoseClampsToTheWholeFootprintBoundary() {
        val pose = localRobotPose(x = 0, y = 20, direction = Face.W, obstacles = emptyList())

        assertEquals(RobotState(x = 1, y = 18, direction = Face.W), pose)
    }

    @Test
    fun typedMovementCommandsProduceTheExpectedCandidatePose() {
        val robot = RobotState(x = 6, y = 6, direction = Face.E)

        assertEquals(RobotState(x = 7, y = 6, direction = Face.E), nextRobotPose(RobotCommand.FORWARD, robot))
        assertEquals(RobotState(x = 5, y = 6, direction = Face.E), nextRobotPose(RobotCommand.REVERSE, robot))
        assertEquals(RobotState(x = 6, y = 6, direction = Face.N), nextRobotPose(RobotCommand.TURN_LEFT, robot))
        assertNull(nextRobotPose(RobotCommand.STOP, robot))
    }

    @Test
    fun remoteRobotPoseRetainsTheReportedPoseAndDescribesTheMapConflict() {
        val obstacles = listOf(Obstacle("B1", x = 7, y = 7))

        val update = remoteRobotPose(x = 6, y = 6, direction = Face.N, obstacles = obstacles)

        assertEquals(RobotState(x = 6, y = 6, direction = Face.N), update.pose)
        assertEquals("B1", update.conflictingObstacle?.id)
    }

    @Test
    fun footprintConflictOnlyIncludesCellsWithinTheThreeByThreeRobotArea() {
        val robot = RobotState(x = 6, y = 6, direction = Face.N)

        assertTrue(robotOverlapsObstacle(robot, listOf(Obstacle("B1", x = 7, y = 7))) != null)
        assertFalse(robotOverlapsObstacle(robot, listOf(Obstacle("B2", x = 8, y = 8))) != null)
    }
}
