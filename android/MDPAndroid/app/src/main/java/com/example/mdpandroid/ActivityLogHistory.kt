package com.example.mdpandroid

import android.content.Context
import android.net.Uri
import android.os.Handler
import android.os.Looper
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import java.io.IOException
import java.util.concurrent.Executors
import java.util.concurrent.Future

data class ActivityHistoryState(
    val entries: List<ActivityLogEntry> = emptyList(), // Newest first.
    val totalCount: Long = 0,
    val hasOlder: Boolean = false,
    val hasNewer: Boolean = false,
    val loading: Boolean = true,
    val followingLatest: Boolean = true,
    val notice: String? = null
)

/** Serialized persistence and a bounded, bidirectional window over the complete saved history. */
class ActivityLogHistory(context: Context, databaseName: String = "robot-activity.db") {
    private val store = ActivityLogStore(context, databaseName)
    private val resolver = context.applicationContext.contentResolver
    private val worker = Executors.newSingleThreadExecutor()
    private val main = Handler(Looper.getMainLooper())
    private var generation = 0L
    private var savedCount = 0L // Accessed only by the worker.
    @Volatile private var closed = false
    private var closeCompletion: Future<*>? = null

    var state by mutableStateOf(ActivityHistoryState())
        private set

    init {
        work("Could not load log history") {
            savedCount = store.count()
            publishPage(store.latest(PAGE_SIZE))
        }
    }

    fun append(source: ActivitySource, message: StatusMessage) {
        work("Could not save log entry") {
            val entry = store.append(source, message.time, message.text)
            savedCount++
            val count = savedCount
            post {
                if (state.followingLatest) {
                    val entries = (listOf(entry) + state.entries).take(WINDOW_SIZE)
                    state = state.copy(entries = entries, totalCount = count,
                        hasOlder = count > entries.size, hasNewer = false)
                } else {
                    state = state.copy(totalCount = count, hasNewer = true)
                }
            }
        }
    }

    fun followLatest(follow: Boolean) {
        state = state.copy(followingLatest = follow && !state.hasNewer)
    }

    fun loadOlder() {
        if (state.loading || !state.hasOlder) return
        val cached = state.entries
        val oldest = cached.lastOrNull()?.message?.order ?: return
        state = state.copy(loading = true, followingLatest = false)
        work("Could not load older logs") {
            val entries = (cached + store.olderThan(oldest, PAGE_SIZE)).distinctBy { it.message.order }.takeLast(WINDOW_SIZE)
            publishPage(entries)
        }
    }

    fun loadNewer() {
        if (state.loading || !state.hasNewer) return
        val cached = state.entries
        val newest = cached.firstOrNull()?.message?.order ?: return showLatest()
        state = state.copy(loading = true, followingLatest = false)
        work("Could not load newer logs") {
            val entries = (store.newerThan(newest, PAGE_SIZE) + cached).distinctBy { it.message.order }.take(WINDOW_SIZE)
            publishPage(entries)
        }
    }

    fun showLatest() {
        if (state.loading) return
        state = state.copy(loading = true, followingLatest = true)
        work("Could not load latest logs") { publishPage(store.latest(PAGE_SIZE)) }
    }

    fun clear() {
        if (closed) return
        generation++ // Older query/write callbacks must not put deleted rows back on screen.
        // Keep the current window until deletion succeeds, including if storage is unavailable.
        state = state.copy(loading = true, followingLatest = true, notice = null)
        work("Could not clear log history") {
            store.clear()
            savedCount = 0
            publishPage(emptyList())
        }
    }

    fun export(uri: Uri) {
        work("Could not export logs") {
            val stream = resolver.openOutputStream(uri, "wt") ?: throw IOException("Cannot open the selected file")
            stream.bufferedWriter(Charsets.UTF_8).use(store::exportTo)
            post { state = state.copy(notice = "Log history exported.") }
        }
    }

    fun dismissNotice() { state = state.copy(notice = null) }

    /** Flush queued log writes before closing the database; never block the UI thread. */
    @Synchronized fun close(): Future<*> {
        closeCompletion?.let { return it }
        closed = true
        val completion = worker.submit { store.close() }
        closeCompletion = completion
        worker.shutdown()
        return completion
    }

    private fun publishPage(entries: List<ActivityLogEntry>) {
        val older = entries.lastOrNull()?.let { store.hasBefore(it.message.order) } ?: false
        val newer = entries.firstOrNull()?.let { store.hasAfter(it.message.order) } ?: false
        val count = savedCount
        post { state = state.copy(entries = entries, totalCount = count, hasOlder = older, hasNewer = newer, loading = false) }
    }

    private fun work(error: String, action: () -> Unit) {
        if (closed) return
        val epoch = generation
        worker.execute {
            workerEpoch = epoch
            try { action() }
            catch (failure: Exception) {
                post { state = state.copy(loading = false, notice = "$error: ${failure.message ?: "storage unavailable"}") }
            }
        }
    }

    private var workerEpoch = 0L // Accessed only by the worker.
    private fun post(update: () -> Unit) {
        val epoch = workerEpoch
        main.post { if (!closed && generation == epoch) update() }
    }

    private companion object {
        const val PAGE_SIZE = 100
        const val WINDOW_SIZE = 300
    }
}
