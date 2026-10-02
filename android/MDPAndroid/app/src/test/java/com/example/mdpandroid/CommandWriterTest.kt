package com.example.mdpandroid

import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import org.junit.Assert.*
import org.junit.Test

class CommandWriterTest {
    @Test fun stopCancelsPendingCommandsAndRunsAfterTheActiveWrite() {
        val started = CountDownLatch(1)
        val release = CountDownLatch(1)
        val stopped = CountDownLatch(1)
        val written = java.util.Collections.synchronizedList(mutableListOf<String>())
        val writer = CommandWriter(timeoutMillis = 2000)
        try {
            writer.enqueue(WriteRequest(write = { started.countDown(); release.await(); written.add("active") }))
            assertTrue(started.await(2, TimeUnit.SECONDS))
            writer.enqueue(WriteRequest(write = { written.add("f") }))
            writer.enqueue(WriteRequest(write = { written.add("beginExplore") }))
            writer.enqueue(WriteRequest(write = { written.add("STOP") }, onSuccess = { stopped.countDown() }), urgent = true)
            release.countDown()
            assertTrue(stopped.await(2, TimeUnit.SECONDS))
            assertEquals(listOf("active", "STOP"), written)
        } finally { release.countDown(); writer.close() }
    }

    @Test fun queuedWritesFromAnOldSessionAreDiscarded() {
        val started = CountDownLatch(1)
        val release = CountDownLatch(1)
        val drained = CountDownLatch(1)
        val current = AtomicBoolean(true)
        val staleWritten = AtomicBoolean(false)
        val cancelled = AtomicBoolean(false)
        val writer = CommandWriter()
        try {
            writer.enqueue(WriteRequest(write = { started.countDown(); release.await() }))
            assertTrue(started.await(2, TimeUnit.SECONDS))
            writer.enqueue(WriteRequest(isCurrent = { current.get() }, write = { staleWritten.set(true) }, onCancelled = { cancelled.set(true) }))
            writer.enqueue(WriteRequest(write = {}, onSuccess = { drained.countDown() }))
            current.set(false)
            release.countDown()
            assertTrue(drained.await(2, TimeUnit.SECONDS))
            assertFalse(staleWritten.get())
            assertTrue(cancelled.get())
        } finally { release.countDown(); writer.close() }
    }

    @Test fun stalledWriteExpiresAndCannotReportSuccess() {
        val expired = CountDownLatch(1)
        val release = CountDownLatch(1)
        val drained = CountDownLatch(1)
        val succeeded = AtomicBoolean(false)
        val writer = CommandWriter(timeoutMillis = 50)
        try {
            writer.enqueue(WriteRequest(write = { release.await() },
                onTimeout = { expired.countDown(); release.countDown() }, onSuccess = { succeeded.set(true) }))
            writer.enqueue(WriteRequest(write = {}, onSuccess = { drained.countDown() }))
            assertTrue(expired.await(2, TimeUnit.SECONDS))
            assertTrue(drained.await(2, TimeUnit.SECONDS))
            assertFalse(succeeded.get())
        } finally { release.countDown(); writer.close() }
    }

    @Test fun failedBatchNeverReportsSuccess() {
        val failed = CountDownLatch(1)
        val succeeded = AtomicBoolean(false)
        val writer = CommandWriter()
        try {
            writer.enqueue(WriteRequest(write = { throw java.io.IOException("peer disconnected") },
                onFailure = { failed.countDown() }, onSuccess = { succeeded.set(true) }))
            assertTrue(failed.await(2, TimeUnit.SECONDS))
            assertFalse(succeeded.get())
        } finally { writer.close() }
    }

    @Test fun stopCancelsQueuedMovementWithoutChangingTheDisplayedPose() {
        val started = CountDownLatch(1)
        val release = CountDownLatch(1)
        val stopped = CountDownLatch(1)
        val original = RobotState(5, 5, Face.N)
        var displayed = original
        val writer = CommandWriter()
        try {
            writer.enqueue(WriteRequest(write = { started.countDown(); release.await() }))
            assertTrue(started.await(2, TimeUnit.SECONDS))
            writer.enqueue(WriteRequest(write = {}, onSuccess = {
                displayed = nextRobotPose(RobotCommand.FORWARD, displayed)!!
            }))
            writer.enqueue(WriteRequest(write = {}, onSuccess = { stopped.countDown() }), urgent = true)
            release.countDown()
            assertTrue(stopped.await(2, TimeUnit.SECONDS))
            assertEquals(original, displayed)
        } finally { release.countDown(); writer.close() }
    }
}
