package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.buildBlockSystem
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo

internal class ExternalQueueTest {
    @Test
    fun test() {
        val items = List(200) { Any() }
        val queue = ExternalQueue<Any>()
        val monitor: Monitor<out Any?>
        val system = buildBlockSystem {
            monitor = queue.output.monitor()
        }
        system.init()
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
