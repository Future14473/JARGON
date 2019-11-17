package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.util.value
import java.util.concurrent.atomic.AtomicBoolean

/**
 * A block that only has a single boolean output:
 *
 * When [pulse] is called externally; the output will be `true` the next time this component is processed, then
 * reset back to false. This is useful to "pulse" a condition to another component.
 */
class Pulse : PrincipalOutputBlock<Boolean>(Processing.LAZY) {

    private val queuePulse = AtomicBoolean()
    /**
     * Pulses so that the next output of this component, only when processed, will be `true`;
     * then resets to `false`.
     */
    fun pulse() {
        queuePulse.value = true
    }

    override fun init() {
        queuePulse.value = false
    }

    override fun Context.getOutput(): Boolean =
        queuePulse.compareAndSet(true, false)
}
