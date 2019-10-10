package org.futurerobotics.jargon.control

import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isA
import strikt.assertions.isFalse
import strikt.assertions.isTrue

internal class PulseTest : AbstractBlockSystemTest() {

    @Test
    fun `pulse pulses`() {
        val monitor: Monitor<Boolean>
        val pulse = Pulse()
        val system = buildBlocksSystem {
            monitor = pulse.monitor()
        }

        expectThat(monitor) {
            system.init()
            repeat(5) {
                repeat(5) {
                    system.loop(0.0)
                    get { value }.isA<Boolean>().isFalse()
                }
                pulse.pulse()
                system.loop(0.0)
                get { value }.isA<Boolean>().isTrue()
            }
        }

    }
}