package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_LAZY
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
class Pulse : SingleOutputBlock<Boolean>(0, IN_FIRST_LAZY) {
    private val queuePulse = AtomicBoolean()

    /**
     * Pulses so that the next output of this component, only when requested, will be true;
     * then resets to false until next pulse.
     */
    fun pulse() {
        queuePulse.value = true
    }

    override fun doInit(): Boolean? {
        queuePulse.value = false
        return false
    }

    override fun getOutput(inputs: List<Any?>): Boolean {
        return queuePulse.compareAndSet(true, false)
    }
}

/**
 * A component that contains a concurrent [queue] which is popped every time someone [getOutput]s. If the queue is empty,
 * returns `null`. This queue can be modified externally.
 *
 * This can for instance be used queue up motion profiles to feed into a [MotionProfileFollower]
 */
class ExternalQueue<T> private constructor(
    private val clearOnStart: Boolean,
    private val queue: Queue<T>
) : SingleOutputBlock<T>(0, IN_FIRST_LAZY), Queue<T> by queue {
    /**
     * @param clearOnStart is true, the queue is cleared when the system starts.
     * @see [ExternalQueue]
     */
    @JvmOverloads
    constructor(clearOnStart: Boolean = false) : this(clearOnStart, ConcurrentLinkedQueue())

    override fun doInit(): T? {
        if (clearOnStart) queue.clear()
        return null
    }

    override fun getOutput(inputs: List<Any?>): T = queue.poll()
}

/**
 * A block that simply passes all its inputs into its outputs.
 *
 * Has a few limited but useful applications.
 */
class PassBlock(size: Int, processing: Block.Processing) : AbstractBlock(size, size, processing) {
    override fun init() {
    }

    override fun process(inputs: List<Any?>) {
    }

    override fun getOutput(index: Int, inputs: List<Any?>): Any? {
        return inputs[index]
    }
}