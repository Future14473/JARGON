package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Monitor
import org.futurerobotics.jargon.blocks.buildTestBlocksSystem
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isA
import strikt.assertions.isFalse
import strikt.assertions.isTrue

internal class PulseTest {
    @Test
    fun `pulse pulses`() {
        val monitor: Monitor<Boolean>
        val pulse = Pulse()
        val system = buildTestBlocksSystem {
            monitor = pulse.monitor()
        }

        expectThat(monitor) {
            system.start()
            repeat(5) {
                repeat(5) {
                    system.loop()
                    get { value }.isA<Boolean>().isFalse()
                }
                pulse.pulse()
                system.loop()
                get { value }.isA<Boolean>().isTrue()
            }
        }
    }
}
