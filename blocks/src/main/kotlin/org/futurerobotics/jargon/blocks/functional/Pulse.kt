package org.futurerobotics.jargon.blocks.functional

import kotlinx.atomicfu.atomic
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock

/**
 * A block that only has a single boolean output:
 *
 * When [pulse] is called externally; the output will be `true` the next time this component is processed, then
 * reset back to false. This is useful to "pulse" a condition to another component.
 */
class Pulse : PrincipalOutputBlock<Boolean>(Processing.LAZY) {

    private val queuePulse = atomic(0)
    /**
     * Pulses so that the next output of this component, only when processed, will be `true`;
     * then resets to `false`.
     */
    fun pulse() {
        queuePulse.getAndIncrement()
    }

    override fun Context.getOutput(): Boolean {
        val out = queuePulse.value > 0
        if (out) queuePulse.getAndDecrement()
        return out
    }

    override fun stop() {
        queuePulse.value = 0
    }
}
