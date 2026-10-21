package com.example.mdpandroid.ui.theme

import android.os.Build
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.dynamicDarkColorScheme
import androidx.compose.material3.dynamicLightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext

private val GameColorScheme = darkColorScheme(
    primary = GamePink,
    onPrimary = GameNavy,
    primaryContainer = MissionNavy,
    onPrimaryContainer = MissionInk,
    secondary = GameCyan,
    onSecondary = GameNavy,
    secondaryContainer = QuietPanel,
    onSecondaryContainer = MissionInk,
    tertiary = GameMint,
    onTertiary = GameNavy,
    tertiaryContainer = SuccessContainer,
    onTertiaryContainer = GameMint,
    error = MissionError,
    onError = Color(0xFF410E22),
    errorContainer = Color(0xFF492237),
    onErrorContainer = Color(0xFFFFDDE2),
    background = AppBackground,
    onBackground = MissionInk,
    surface = DarkSurface,
    onSurface = MissionInk,
    surfaceVariant = QuietPanel,
    onSurfaceVariant = Color(0xFFC3CAE0),
    outline = OutlineBlueGrey,
    outlineVariant = Color(0xFF34426B)
)

@Composable
fun MDPAndroidTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    dynamicColor: Boolean = false,
    content: @Composable () -> Unit
) {
    // Use the deck's navy appearance by default, independently of device settings.
    val colorScheme = if (dynamicColor && Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        val context = LocalContext.current
        if (darkTheme) dynamicDarkColorScheme(context) else dynamicLightColorScheme(context)
    } else GameColorScheme
    MaterialTheme(colorScheme = colorScheme, typography = Typography, content = content)
}
