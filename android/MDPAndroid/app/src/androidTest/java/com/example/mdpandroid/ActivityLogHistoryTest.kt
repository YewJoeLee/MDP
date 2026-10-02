package com.example.mdpandroid

import android.net.Uri
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File
import java.util.UUID
import java.util.concurrent.TimeUnit

@RunWith(AndroidJUnit4::class)
class ActivityLogHistoryTest {
    private val context get() = ApplicationProvider.getApplicationContext<android.content.Context>()
    private fun main(action: () -> Unit) = InstrumentationRegistry.getInstrumentation().runOnMainSync(action)

    @Test fun browsingDoesNotLosePositionWhenNewLogsArriveAndAllPagesRemainReachable() = withHistory { history ->
        main { repeat(550) { history.append(ActivitySource.STATUS, StatusMessage("time", "entry $it")) } }
        await { history.state.totalCount == 550L }
        assertEquals(300, history.state.entries.size)
        main { history.followLatest(false); history.loadOlder() }
        await { !history.state.loading }
        assertEquals("entry 449", history.state.entries.first().message.text)
        val browsed = history.state.entries
        main { history.append(ActivitySource.RECEIVED, StatusMessage("time", "new arrival")) }
        await { history.state.totalCount == 551L }
        assertEquals(browsed, history.state.entries)
        assertTrue(history.state.hasNewer)
        while (history.state.hasOlder) {
            main { history.loadOlder() }
            await { !history.state.loading }
        }
        assertEquals("entry 0", history.state.entries.last().message.text)
        while (history.state.hasNewer) {
            main { history.loadNewer() }
            await { !history.state.loading }
        }
        assertEquals("new arrival", history.state.entries.first().message.text)
        assertTrue(history.state.entries.size <= 300)
    }

    @Test fun clearingDuringQueuedWritesCannotRestoreDeletedEntries() = withHistory { history ->
        main {
            repeat(250) { history.append(ActivitySource.STATUS, StatusMessage("time", "old $it")) }
            history.clear()
            history.append(ActivitySource.STATUS, StatusMessage("time", "after clear"))
        }
        await { !history.state.loading && history.state.totalCount == 1L }
        assertEquals(listOf("after clear"), history.state.entries.map { it.message.text })
    }

    @Test fun exportIncludesUnloadedHistoryAndReportsFailureWithoutLosingLogs() = withHistory { history ->
        main { repeat(400) { history.append(ActivitySource.RECEIVED, StatusMessage("time", "entry $it")) } }
        await { history.state.totalCount == 400L }
        val file = File(context.cacheDir, "log-export-${UUID.randomUUID()}.txt")
        try {
            main { history.export(Uri.fromFile(file)) }
            await { history.state.notice == "Log history exported." }
            assertEquals(400, file.readLines().size)
            assertEquals("time Received: entry 0", file.readLines().first())
            assertEquals("time Received: entry 399", file.readLines().last())
            main { history.dismissNotice(); history.export(Uri.parse("content://invalid.log.provider/test")) }
            await { history.state.notice?.startsWith("Could not export logs") == true }
            assertEquals(400L, history.state.totalCount)
        } finally { file.delete() }
    }

    @Test fun reopeningRestoresHistoryAndContinuesWithUniqueIds() {
        val name = "history-test-${UUID.randomUUID()}.db"
        var first: ActivityLogHistory? = null
        var second: ActivityLogHistory? = null
        try {
            main { first = ActivityLogHistory(context, name) }
            await { !first!!.state.loading }
            main { first!!.append(ActivitySource.STATUS, StatusMessage("time", "saved")) }
            await { first!!.state.totalCount == 1L }
            val oldId = first!!.state.entries.single().message.order
            requireNotNull(first).close().get(10, TimeUnit.SECONDS)
            main { second = ActivityLogHistory(context, name) }
            val restored = requireNotNull(second)
            await { !restored.state.loading }
            assertEquals("saved", restored.state.entries.single().message.text)
            main { restored.append(ActivitySource.STATUS, StatusMessage("time", "next")) }
            await { restored.state.totalCount == 2L }
            assertTrue(restored.state.entries.first().message.order > oldId)
        } finally {
            first?.close()?.get(10, TimeUnit.SECONDS)
            second?.close()?.get(10, TimeUnit.SECONDS)
            context.deleteDatabase(name)
        }
    }

    private fun withHistory(test: (ActivityLogHistory) -> Unit) {
        val name = "history-test-${UUID.randomUUID()}.db"
        lateinit var history: ActivityLogHistory
        main { history = ActivityLogHistory(context, name) }
        try { await { !history.state.loading }; test(history) }
        finally { history.close().get(10, TimeUnit.SECONDS); context.deleteDatabase(name) }
    }

    private fun await(condition: () -> Boolean) {
        val until = SystemClock.uptimeMillis() + 10_000
        while (!condition() && SystemClock.uptimeMillis() < until) SystemClock.sleep(10)
        assertTrue("History operation did not finish", condition())
    }
}
