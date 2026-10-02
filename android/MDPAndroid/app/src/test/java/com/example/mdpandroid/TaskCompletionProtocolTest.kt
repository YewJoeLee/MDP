package com.example.mdpandroid

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

class TaskCompletionProtocolTest {
    @Test fun observedPiCompletionMessageIsRecognized() {
        assertEquals(ProtocolMessage.TaskComplete, parseProtocolMessage("MSG,Run complete. Done !"))
    }

    @Test fun observedCompletionWaitsForTheDelimiterAcrossSplitReads() {
        val framer = IncomingMessageFramer()
        assertEquals(emptyList<String>(), framer.append("MSG,Run com"))
        assertEquals(emptyList<String>(), framer.append("plete. Done !"))
        val records = framer.append("\r\n")
        assertEquals(listOf("MSG,Run complete. Done !"), records)
        assertEquals(ProtocolMessage.TaskComplete, parseProtocolMessage(records.single()))
    }

    @Test fun ordinaryOrExtendedCompletionStatusRemainsText() {
        listOf("Run complete", "Done !", "Run complete. Done ! Waiting", "taskComplete").forEach {
            assertEquals(ProtocolMessage.Text(it), parseProtocolMessage("MSG,$it"))
        }
    }

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
