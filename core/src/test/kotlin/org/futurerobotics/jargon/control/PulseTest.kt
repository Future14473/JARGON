package org.futurerobotics.jargon.control

import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isA
import strikt.assertions.isFalse
import strikt.assertions.isTrue

internal class PulseTest : AbstractBlockSystemTest() {

    @Test
    fun testPulse() {
        val pulseBlock = Pulse()
        val system = buildBlockSystem {
            val pulse = pulseBlock.add()<Boolean>()
            monitor = pulse.monitor()
        }

        expectThat(monitor) {
            system.init()
            repeat(5) {
                repeat(5) {
                    system.loop(0.0)
                    get { value }.isA<Boolean>().isFalse()
                }
                pulseBlock.pulse()
                system.loop(0.0)
                get { value }.isA<Boolean>().isTrue()
            }
        }

    }
}