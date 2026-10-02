package com.example.mdpandroid

import org.junit.Assert.assertEquals
import org.junit.Test

class ArenaGestureTest {
    @Test
    fun obstacleReleasedOutsideMovesEvenBelowTapThreshold() {
        assertEquals(ArenaGestureAction.MOVE, classify(distance = 4f, release = GridPoint(-1, 10)))
    }

    @Test
    fun shortObstacleReleaseBeyondAnyEdgeMovesInsteadOfChangingFace() {
        listOf(GridPoint(-1, 10), GridPoint(20, 10), GridPoint(10, -1), GridPoint(10, 20)).forEach { release ->
            assertEquals(ArenaGestureAction.MOVE, classify(distance = 10f, release = release))
        }
    }

    @Test
    fun shortObstacleDragWithinMapChangesFace() {
        assertEquals(ArenaGestureAction.FACE, classify(distance = 10f))
    }

    @Test
    fun tapWithinMapSelectsInsteadOfMoving() {
        assertEquals(ArenaGestureAction.TAP, classify(distance = 4f))
        assertEquals(ArenaGestureAction.TAP, classify(distance = 0f, moved = false))
    }

    @Test
    fun normalObstacleDragWithinMapMoves() {
        assertEquals(ArenaGestureAction.MOVE, classify(distance = 30f))
    }

    @Test
    fun shortRobotDragKeepsFaceGestureBehavior() {
        assertEquals(ArenaGestureAction.FACE, classify(distance = 10f, release = GridPoint(-1, 10), obstacleSelected = false))
    }

    private fun classify(
        distance: Float,
        release: GridPoint = GridPoint(10, 10),
        moved: Boolean = true,
        obstacleSelected: Boolean = true
    ) = classifyArenaGesture(moved, distance, tapSlop = 6f, cellSize = 20f, release, obstacleSelected)
}
