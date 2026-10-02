package com.example.mdpandroid

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

class TaskCompletionProtocolTest {
    @Test fun bareTaskCompletionIsRecognized() {
        assertNotNull(parseProtocolMessage("taskComplete"))
    }

    @Test fun completionWaitsForTheDelimiterAcrossSplitBluetoothReads() {
        val framer = IncomingMessageFramer()
        assertEquals(emptyList<String>(), framer.append("taskCom"))
        assertEquals(emptyList<String>(), framer.append("plete"))
        val records = framer.append("\r\n")
        assertEquals(listOf("taskComplete"), records)
        assertNotNull(parseProtocolMessage(records.single()))
    }

    @Test fun partialOrExtendedCompletionTokensAreNotRecognized() {
        listOf("taskCom", "taskCompleteLater", "taskComplete,1").forEach {
            assertNull(parseProtocolMessage(it))
        }
    }
}
