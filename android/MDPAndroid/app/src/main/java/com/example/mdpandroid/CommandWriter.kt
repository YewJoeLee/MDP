package com.example.mdpandroid

import java.util.ArrayDeque
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.TimeUnit

internal data class WriteRequest(
    val isCurrent: () -> Boolean = { true },
    val write: () -> Unit,
    val onSuccess: () -> Unit = {},
    val onFailure: (Exception) -> Unit = {},
    val onTimeout: () -> Unit = {},
    val onCancelled: () -> Unit = {}
)

/** Serializes complete batches. Stop discards pending work; the watchdog closes stalled sockets. */
internal class CommandWriter(private val timeoutMillis: Long = 2000) : AutoCloseable {
    private val executor = Executors.newSingleThreadExecutor()
    private val timer = Executors.newSingleThreadScheduledExecutor()
    private val pending = ArrayDeque<WriteRequest>()
    private var running = false
    private var closed = false

    fun enqueue(request: WriteRequest, urgent: Boolean = false): Boolean {
        val cancelled: List<WriteRequest>
        synchronized(this) {
            if (closed || (!urgent && pending.size >= 128)) return false
            cancelled = if (urgent) pending.toList().also { pending.clear() } else emptyList()
            pending.addLast(request)
            if (!running) {
                running = true
                executor.execute(::drain)
            }
        }
        cancelled.forEach { it.onCancelled() }
        return true
    }

    private fun drain() {
        while (true) {
            val request = synchronized(this) {
                if (closed || pending.isEmpty()) { running = false; return }
                pending.removeFirst()
            }
            if (!request.isCurrent()) {
                request.onCancelled()
                continue
            }
            val completed = AtomicBoolean(false)
            val watchdog = synchronized(this) {
                if (closed) return
                timer.schedule({
                    if (completed.compareAndSet(false, true)) request.onTimeout()
                }, timeoutMillis, TimeUnit.MILLISECONDS)
            }
            try {
                if (request.isCurrent()) {
                    request.write()
                    if (completed.compareAndSet(false, true)) request.onSuccess()
                } else if (completed.compareAndSet(false, true)) {
                    request.onCancelled()
                }
            } catch (error: Exception) {
                if (completed.compareAndSet(false, true)) request.onFailure(error)
            } finally {
                completed.set(true)
                watchdog.cancel(false)
            }
        }
    }

    fun cancelPending() {
        val cancelled = synchronized(this) { pending.toList().also { pending.clear() } }
        cancelled.forEach { it.onCancelled() }
    }

    override fun close() {
        synchronized(this) { closed = true; pending.clear() }
        executor.shutdownNow()
        timer.shutdownNow()
    }
}
