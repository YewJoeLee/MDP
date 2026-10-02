package com.example.mdpandroid

/** Collects incoming Pi messages until a newline or carriage return arrives. */
internal class IncomingMessageFramer(private val onOverflow: () -> Unit = {}) {
    private val pending = StringBuilder()
    private var discarding = false

    fun append(chunk: String): List<String> {
        val lines = mutableListOf<String>()
        for (character in chunk) {
            if (character == '\n' || character == '\r') {
                if (!discarding) pending.toString().trim().takeIf(String::isNotEmpty)?.let(lines::add)
                pending.clear()
                discarding = false
            } else if (!discarding) {
                pending.append(character)
                if (pending.length > 8192) {
                    pending.clear()
                    discarding = true
                    onOverflow()
                }
            }
        }
        return lines
    }

    fun hasPending(): Boolean = pending.isNotEmpty()

    fun reset() {
        pending.clear()
        discarding = false
    }

}
