package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Monitor
import org.futurerobotics.jargon.blocks.buildBlocksSystem
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo

internal class ExternalQueueTest {
    @Test
    fun test() {
        val items = List(200) { Any() }
        val queue = ExternalQueue<Any>()
        val monitor: Monitor<Any?>
        val system = buildBlocksSystem {
            monitor = queue.monitor()
        }
        system.start()
        queue.addAll(items)
        val output = ArrayList<Any>()
        while (true) {
            system.loop()
            val value = monitor.value
            if (value != null) output.add(value)
            else break
        }
        expectThat(items).isEqualTo(output)
    }
}
