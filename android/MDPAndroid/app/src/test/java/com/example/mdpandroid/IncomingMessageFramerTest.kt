package com.example.mdpandroid

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class IncomingMessageFramerTest {
    @Test
    fun targetWaitsForNewlineAcrossAnIdleGap() {
        val framer = IncomingMessageFramer()

        assertEquals(emptyList<String>(), framer.append("TARGET,2,"))
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("TARGET,2,11"), framer.append("11\n"))
    }

    @Test
    fun partialTargetPrefixAlsoWaitsForNewline() {
        val framer = IncomingMessageFramer()

        assertEquals(emptyList<String>(), framer.append("TAR"))
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("TARGET,2,11"), framer.append("GET,2,11\n"))
    }

    @Test
    fun unterminatedTextWaitsForNewline() {
        val framer = IncomingMessageFramer()

        assertEquals(emptyList<String>(), framer.append("MSG,ready"))
        assertTrue(framer.hasPending())
        assertEquals(listOf("MSG,ready"), framer.append("\n"))
    }

    @Test
    fun completeLinesAreDeliveredSeparately() {
        val framer = IncomingMessageFramer()

        assertEquals(listOf("TARGET,1,11", "TARGET,2,12"),
            framer.append("TARGET,1,11\nTARGET,2,12\r\n"))
    }

    @Test
    fun incompleteRobotCoordinateWaitsForNewline() {
        val framer = IncomingMessageFramer()
        framer.append("ROBOT,1,1")
        assertTrue(framer.hasPending())
        framer.append("0,N")
        assertEquals(listOf("ROBOT,1,10,N"), framer.append("\n"))
    }

    @Test
    fun completeTargetStillRequiresANewline() {
        val framer = IncomingMessageFramer()
        framer.append("TARGET,1,11")
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("TARGET,1,11"), framer.append("\n"))
    }

    @Test
    fun oversizedLineIsDiscardedAndNextDelimitedMessageRecovers() {
        var overflows = 0
        val framer = IncomingMessageFramer { overflows++ }
        assertEquals(emptyList<String>(), framer.append("TARGET,1," + "1".repeat(9000)))
        assertEquals(emptyList<String>(), framer.append(""))
        assertEquals(listOf("MSG,ready"), framer.append("rest\nMSG,ready\n"))
        assertEquals(1, overflows)
    }

    @Test
    fun resetDropsPartialDataFromThePreviousSession() {
        val framer = IncomingMessageFramer()
        framer.append("TARGET,1,")
        framer.reset()
        assertEquals(listOf("ROBOT,8,9,N"), framer.append("ROBOT,8,9,N\n"))
    }
}
