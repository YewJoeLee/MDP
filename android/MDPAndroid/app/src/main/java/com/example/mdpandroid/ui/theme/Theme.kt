package com.example.mdpandroid.ui.theme

import android.app.Activity
import android.os.Build
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.dynamicDarkColorScheme
import androidx.compose.material3.dynamicLightColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext

private val DarkColorScheme = darkColorScheme(
    primary = Color(0xFF57D2BF),
    onPrimary = Color(0xFF00382F),
    primaryContainer = MissionNavy,
    onPrimaryContainer = Color.White,
    secondary = Color(0xFF9BC7E0),
    onSecondary = MissionInk,
    secondaryContainer = DarkPanel,
    onSecondaryContainer = Color(0xFFE7F1F4),
    tertiary = Color(0xFFFFD189),
    onTertiary = MissionInk,
    error = Color(0xFFFFB4AB),
    onError = Color(0xFF690005),
    background = DarkBackground,
    onBackground = Color(0xFFE6EEF4),
    surface = DarkSurface,
    onSurface = Color(0xFFE6EEF4),
    surfaceVariant = DarkPanel,
    onSurfaceVariant = Color(0xFFC0CED8),
    outline = Color(0xFF8FA2B0),
    outlineVariant = Color(0xFF3B4D5B)
)

private val LightColorScheme = lightColorScheme(
    primary = MissionTeal,
    onPrimary = Color.White,
    primaryContainer = MissionNavy,
    onPrimaryContainer = Color.White,
    secondary = ObstacleBlue,
    onSecondary = Color.White,
    secondaryContainer = QuietPanel,
    onSecondaryContainer = MissionInk,
    tertiary = TargetAmber,
    onTertiary = MissionInk,
    tertiaryContainer = Color(0xFFFFE1B5),
    onTertiaryContainer = Color(0xFF4B2F00),
    error = MissionError,
    onError = Color.White,
    errorContainer = Color(0xFFFFDAD6),
    onErrorContainer = Color(0xFF410002),
    background = AppBackground,
    onBackground = MissionInk,
    surface = Color.White,
    onSurface = MissionInk,
    surfaceVariant = QuietPanel,
    onSurfaceVariant = Color(0xFF43545F),
    outline = OutlineBlueGrey,
    outlineVariant = Color(0xFFD0DCE2)
)

@Composable
fun MDPAndroidTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    // Dynamic color is available on Android 12+
    dynamicColor: Boolean = false,
    content: @Composable () -> Unit
) {
    val colorScheme = when {
        dynamicColor && Build.VERSION.SDK_INT >= Build.VERSION_CODES.S -> {
            val context = LocalContext.current
            if (darkTheme) dynamicDarkColorScheme(context) else dynamicLightColorScheme(context)
        }

        darkTheme -> DarkColorScheme
        else -> LightColorScheme
    }

    MaterialTheme(
        colorScheme = colorScheme,
        typography = Typography,
        content = content
    )
}
