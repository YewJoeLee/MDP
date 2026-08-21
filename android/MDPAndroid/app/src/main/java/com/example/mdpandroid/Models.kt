package com.example.mdpandroid

data class GridPoint(val x: Int, val y: Int)

enum class Face(val code: String) {
    N("N"), S("S"), E("E"), W("W")
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
