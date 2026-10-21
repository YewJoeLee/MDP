package com.example.mdpandroid

import androidx.compose.foundation.layout.size
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.luminance
import androidx.compose.ui.graphics.toPixelMap
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.test.captureToImage
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.unit.dp
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.ArenaCanvas
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import kotlin.math.abs

@RunWith(AndroidJUnit4::class)
class ArenaFacingVisibilityTest {
    @get:Rule val compose = createComposeRule()

    @Test fun allFourImageFacesStandOutOnBothObstacleStates() {
        val obstacles = listOf(null, "11").flatMapIndexed { row, target ->
            Face.entries.mapIndexed { column, face ->
                Obstacle("B${row * 4 + column + 1}", 4 + column * 4, 7 + row * 6,
                    targetId = target, targetFace = face)
            }
        }
        compose.setContent {
            ArenaCanvas(AppState(obstacles = obstacles), {}, { _, _, _ -> }, {},
                { _, _ -> }, { _, _ -> }, {}, {}, Modifier.size(400.dp).testTag("arena"))
        }
        val image = compose.onNodeWithTag("arena").captureToImage()
        val pixels = image.toPixelMap()
        val cell = image.width / 20f
        val background = pixels[(cell * 2.5f).toInt(), (cell * 2.5f).toInt()]
        fun sample(obstacle: Obstacle, x: Float, y: Float) = pixels[
            ((obstacle.x + x) * cell).toInt(), ((19 - obstacle.y + y) * cell).toInt()]
        obstacles.forEach { obstacle ->
            val face = obstacle.targetFace!!
            val marker = when (face) {
                Face.N -> sample(obstacle, .5f, .15f)
                Face.S -> sample(obstacle, .5f, .85f)
                Face.W -> sample(obstacle, .15f, .5f)
                Face.E -> sample(obstacle, .85f, .5f)
            }
            val unmarkedSide = when (face) {
                Face.N, Face.S -> sample(obstacle, .15f, .5f)
                Face.E, Face.W -> sample(obstacle, .5f, .15f)
            }
            assertTrue("${obstacle.id} $face: face must contrast with the dark arena",
                (marker.luminance() + .05f) / (background.luminance() + .05f) >= 7f)
            assertTrue("${obstacle.id} $face: face must be distinct from the other edges",
                colorDistance(marker, unmarkedSide) > .35f)
        }
    }

    private fun colorDistance(a: Color, b: Color) =
        abs(a.red - b.red) + abs(a.green - b.green) + abs(a.blue - b.blue)
}
