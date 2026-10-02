package com.example.mdpandroid

import androidx.compose.ui.test.*
import androidx.compose.ui.test.junit4.v2.createComposeRule
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.mdpandroid.ui.ResetArenaButton
import com.example.mdpandroid.ui.theme.MDPAndroidTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class ResetArenaUiTest {
    @get:Rule val compose = createComposeRule()

    @Test fun resetRequiresConfirmationAndCancelKeepsTheMap() {
        var resets = 0
        compose.setContent { MDPAndroidTheme { ResetArenaButton(onResetArena = { resets++ }) } }
        compose.onNodeWithText("Reset map").performClick()
        compose.onNodeWithText("Reset map?").assertIsDisplayed()
        compose.onNodeWithText("Cancel").performClick()
        compose.runOnIdle { assertEquals(0, resets) }
        compose.onNodeWithText("Reset map").performClick()
        compose.onNodeWithText("Reset", substring = false).performClick()
        compose.runOnIdle { assertEquals(1, resets) }
        compose.onNodeWithText("Reset map?").assertDoesNotExist()
    }
}
