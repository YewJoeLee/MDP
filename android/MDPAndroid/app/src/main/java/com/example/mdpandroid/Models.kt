package com.example.mdpandroid

data class GridPoint(val x: Int, val y: Int)

enum class Face(val code: String, val dx: Int, val dy: Int) {
    N("N", 0, 1),
    E("E", 1, 0),
    S("S", 0, -1),
    W("W", -1, 0);

    fun turnRight(): Face = when (this) {
        N -> E
        E -> S
        S -> W
        W -> N
    }

    fun turnLeft(): Face = when (this) {
        N -> W
        W -> S
        S -> E
        E -> N
    }
}

data class RobotState(
    val x: Int = 6,
    val y: Int = 2,
    val direction: Face = Face.W
)

data class Obstacle(
    val id: String,
    val x: Int,
    val y: Int,
    val targetId: String? = null,
    val targetFace: Face? = null
)

data class StatusMessage(val time: String, val text: String)
