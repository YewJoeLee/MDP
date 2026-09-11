package com.example.mdpandroid

import androidx.compose.ui.geometry.Offset
import kotlin.math.abs
import kotlin.math.floor

/** Converts a canvas pixel offset into the grid cell it falls within, flipping Y so row 0 is the bottom of the arena. */
fun gridPoint(offset: Offset, width: Float, height: Float): GridPoint {
    val x = floor(offset.x / (width / MAP_COLUMNS)).toInt()
    val yFromTop = floor(offset.y / (height / MAP_ROWS)).toInt()
    return GridPoint(x, MAP_ROWS - 1 - yFromTop)
}

/** Maps a short drag's dominant axis to the facing direction it represents. */
fun faceFromDrag(delta: Offset): Face =
    if (abs(delta.x) > abs(delta.y)) {
        if (delta.x >= 0f) Face.E else Face.W
    } else {
        if (delta.y >= 0f) Face.S else Face.N
    }
