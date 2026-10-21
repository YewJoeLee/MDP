package com.example.mdpandroid.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.mdpandroid.R
import com.example.mdpandroid.ui.theme.*

/** Original deck texture. Decorative only; never intercepts map gestures. */
@Composable
internal fun GameTexture(modifier: Modifier = Modifier) {
    Image(painterResource(R.drawable.game_navy_texture), contentDescription = null,
        contentScale = ContentScale.Crop, alpha = .18f, modifier = modifier)
}

@Composable
internal fun GameSymbols(modifier: Modifier = Modifier, color: Color = GamePink) {
    Canvas(modifier.size(66.dp, 18.dp)) {
        val s = minOf(size.height * .72f, size.width * .18f)
        val cy = size.height / 2
        val stroke = Stroke(1.7.dp.toPx())
        drawCircle(color, s / 2, Offset(size.width * .14f, cy), style = stroke)
        val cx = size.width * .5f
        val triangle = Path().apply {
            moveTo(cx, cy - s * .53f); lineTo(cx + s * .54f, cy + s * .43f)
            lineTo(cx - s * .54f, cy + s * .43f); close()
        }
        drawPath(triangle, color, style = stroke)
        drawRect(color, Offset(size.width * .86f - s / 2, cy - s / 2), Size(s, s), style = stroke)
    }
}

/** Stair/door motif adapted from the supplied deck as responsive UI art. */
@Composable
internal fun GameStairs(modifier: Modifier = Modifier) {
    Canvas(modifier) {
        val w = size.width
        val h = size.height
        val path = Path().apply {
            moveTo(0f, h); lineTo(0f, h * .87f)
            for (i in 0..4) {
                lineTo(w * (i + 1) / 6, h * (.87f - i * .13f))
                lineTo(w * (i + 1) / 6, h * (.74f - i * .13f))
            }
            lineTo(w, h * .22f); lineTo(w, h); close()
        }
        drawRoundRect(GameMint, Offset(w * .72f, h * .04f), Size(w * .24f, h * .69f),
            cornerRadius = androidx.compose.ui.geometry.CornerRadius(4.dp.toPx()))
        drawRoundRect(GameTeal.copy(alpha = .5f), Offset(w * .8f, h * .16f), Size(w * .08f, h * .29f),
            cornerRadius = androidx.compose.ui.geometry.CornerRadius(w * .04f))
        drawPath(path, GamePink)
        drawPath(path, GamePinkEdge, style = Stroke(1.dp.toPx()))
    }
}

@Composable
internal fun GameBrand(compact: Boolean) {
    Column(Modifier.width(if (compact) 54.dp else 158.dp).padding(start = 8.dp, end = 8.dp),
        verticalArrangement = Arrangement.Center) {
        GameSymbols(Modifier.padding(bottom = 4.dp))
        Text(if (compact) "005" else "GROUP 5 / THE GAMES",
            fontFamily = GameDisplayFont, fontSize = if (compact) 11.sp else 9.sp,
            color = MaterialTheme.colorScheme.onSurface)
    }
}

@Composable
internal fun GameBriefing(number: String, title: String, instruction: String, modifier: Modifier = Modifier) {
    Surface(modifier, shape = androidx.compose.foundation.shape.RoundedCornerShape(14.dp),
        color = MaterialTheme.colorScheme.secondaryContainer) {
        Row(Modifier.padding(14.dp), horizontalArrangement = Arrangement.spacedBy(12.dp),
            verticalAlignment = Alignment.CenterVertically) {
            Text(number, fontFamily = GameDisplayFont, fontSize = 27.sp, color = GamePink)
            Column(Modifier.weight(1f)) {
                Text(title, fontFamily = GameDisplayFont, fontSize = 13.sp, color = MissionInk)
                Text(instruction, style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant)
            }
        }
    }
}
