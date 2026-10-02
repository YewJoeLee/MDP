package com.example.mdpandroid

import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import java.io.StringWriter
import java.util.UUID

@RunWith(AndroidJUnit4::class)
class ActivityLogStoreTest {
    private val context get() = ApplicationProvider.getApplicationContext<android.content.Context>()

    @Test fun fullHistorySurvivesReopeningAndPagesInBothDirections() {
        val name = "log-test-${UUID.randomUUID()}.db"
        try {
            ActivityLogStore(context, name).use { store ->
                repeat(350) { i -> store.append(ActivitySource.STATUS, "2026-10-02 12:00:00", "entry $i") }
            }
            ActivityLogStore(context, name).use { store ->
                assertEquals(350L, store.count())
                val latest = store.latest(100)
                assertEquals("entry 349", latest.first().message.text)
                assertEquals("entry 250", latest.last().message.text)
                val older = store.olderThan(latest.last().message.order, 100)
                assertEquals("entry 249", older.first().message.text)
                assertEquals("entry 150", older.last().message.text)
                val newer = store.newerThan(older.first().message.order, 100)
                assertEquals(latest, newer)
                val oldest = store.olderThan(store.olderThan(older.last().message.order, 100).last().message.order, 100)
                assertEquals(50, oldest.size)
                assertEquals("entry 0", oldest.last().message.text)
                assertFalse(store.hasBefore(oldest.last().message.order))
            }
        } finally { context.deleteDatabase(name) }
    }

    @Test fun exportPreservesFullTextSourcesAndChronologicalOrder() = withStore { store ->
        val longText = "TARGET,1," + "long detail ".repeat(900)
        store.append(ActivitySource.RECEIVED, "2026-10-02 12:00:00.001", longText)
        store.append(ActivitySource.STATUS, "2026-10-02 12:00:00.002", "Task complete")
        val writer = StringWriter()
        store.exportTo(writer)
        assertEquals("2026-10-02 12:00:00.001 Received: $longText\n2026-10-02 12:00:00.002 Status: Task complete\n", writer.toString())
    }

    @Test fun clearRemovesSavedHistoryAndFutureEntriesUseNewIds() = withStore { store ->
        val before = store.append(ActivitySource.STATUS, "time", "before")
        store.clear()
        assertEquals(0L, store.count())
        assertTrue(store.latest(100).isEmpty())
        val after = store.append(ActivitySource.RECEIVED, "time", "after")
        assertTrue(after.message.order > before.message.order)
        assertEquals(listOf(after), store.latest(100))
    }

    private fun withStore(test: (ActivityLogStore) -> Unit) {
        val name = "log-test-${UUID.randomUUID()}.db"
        try { ActivityLogStore(context, name).use(test) }
        finally { context.deleteDatabase(name) }
    }
}
