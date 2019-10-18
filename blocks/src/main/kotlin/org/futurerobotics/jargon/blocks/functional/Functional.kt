package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.AbstractBlock
import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.blocks.SystemValues
import org.futurerobotics.jargon.blocks.motion.MotionProfileFollower
import org.futurerobotics.jargon.util.value
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * A block that only has a single boolean output, but it can be "pulsed" true.
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

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Boolean =
        queuePulse.compareAndSet(true, false)
}

/**
 * A component that contains a concurrent [queue] which is popped every time someone [processOutput]s. If the queue is
 * empty, returns `null`. This queue can be modified externally.
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

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): T = queue.poll()
}

/**
 * A block that simply passes all its inputs into its outputs.
 *
 * Has a few limited but useful applications.
 */
class PassBlock(size: Int, processing: Block.Processing) : AbstractBlock(size, size, processing) {
    private var inputs: List<Any?>? = null
    override fun init() {
        inputs = null
    }

    override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        this.inputs = inputs
    }

    override fun getOutput(index: Int): Any? = inputs!![index]
}