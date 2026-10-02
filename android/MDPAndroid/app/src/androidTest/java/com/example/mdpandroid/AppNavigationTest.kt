package com.example.mdpandroid

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.graphics.toPixelMap
import androidx.compose.ui.test.*
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.ARCMApp
import java.util.UUID
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class AppNavigationTest {
    @get:Rule val compose = createComposeRule()

    @Test fun underlineFollowsSelectedTabAndOverviewHasNoActivityPanel() {
        val context = ApplicationProvider.getApplicationContext<android.content.Context>()
        val name = "navigation-test-${UUID.randomUUID()}"
        val store = ArenaStore(context, name)
        val history = ActivityLogHistory(context, "$name.db")
        lateinit var controller: BluetoothController
        compose.runOnIdle {
            controller = BluetoothController(context, history, store)
            controller.setDemoMode(true)
        }
        try {
            compose.setContent {
                MaterialTheme(colorScheme = lightColorScheme(primary = Color.Magenta)) {
                    ARCMApp(controller, requestBluetoothPermissions = {})
                }
            }
            val tabs = listOf("Overview", "Arena", "Controls")
            for (title in listOf("Arena", "Controls", "Overview")) {
                compose.onNode(hasText(title) and hasClickAction()).performClick().assertIsSelected()
                val pixels = compose.onRoot().captureToImage().toPixelMap()
                for (tab in tabs) {
                    val bounds = compose.onNode(hasText(tab) and hasClickAction()).fetchSemanticsNode().boundsInRoot
                    val x = bounds.center.x.toInt()
                    val y = (bounds.bottom - 2).toInt()
                    if (tab == title) assertEquals("Underline must follow $title", Color.Magenta.toArgb(), pixels[x, y].toArgb())
                    else assertNotEquals("Unselected $tab must not be underlined", Color.Magenta.toArgb(), pixels[x, y].toArgb())
                }
                if (title == "Overview") {
                    compose.onNodeWithText("Robot activity", substring = true).assertDoesNotExist()
                    compose.onNodeWithText("Bluetooth connection").assertExists()
                } else compose.onNodeWithText("Robot activity", substring = true).assertExists()
            }
        } finally {
            compose.runOnIdle { controller.close() }
            store.close().get(10, TimeUnit.SECONDS)
            history.close().get(10, TimeUnit.SECONDS)
            context.deleteSharedPreferences(name)
            context.deleteDatabase("$name.db")
        }
    }
}
