package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.Processing.LAZY
import org.futurerobotics.jargon.util.value
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * A component that only has a single boolean output that is usually false, but it can be "pulsed" true.
 *
 * When [pulse] is called externally; the output will be true the next time this component is processed, then
 * reset back to false. This is useful to "pulse" a condition to another component.
 */
class Pulse : AbstractBlock(0, 1, IN_FIRST, LAZY) {

    private val queuePulse = AtomicBoolean()

    /**
     * Pulses so that the next output of this component, only when requested, will be true;
     * then resets to false until next pulse.
     */
    fun pulse() {
        queuePulse.value = true
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = queuePulse.compareAndSet(true, false)
    }

    override fun init(outputs: MutableList<Any?>) {
        queuePulse.value = false
    }
}

/**
 * A component that contains a [queue] which is popped every time the component is processed. If the queue is empty,
 * returns [kotlin.Unit]. The queue can be modified externally.
 *
 * This can for instance be used queue up motion profiles to feed into a [MotionProfileFollower]
 *
 */
class ExternalQueue<T> private constructor(
    private val clearOnStart: Boolean,
    private val queue: Queue<T>
) : AbstractBlock(0, 1, IN_FIRST, LAZY), Queue<T> by queue {
    /**
     * @param clearOnStart is true, the queue is cleared when the system starts.
     * @see [ExternalQueue]
     */
    @JvmOverloads
    constructor(clearOnStart: Boolean = false) : this(clearOnStart, ConcurrentLinkedQueue())

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = queue.poll()
    }

    override fun init(outputs: MutableList<Any?>) {
        if (clearOnStart) queue.clear()
    }

}
