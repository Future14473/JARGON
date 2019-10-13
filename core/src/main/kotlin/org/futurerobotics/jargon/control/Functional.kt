package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.util.value
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * A block with one constant output [value].
 *
 * This is itself a [BlockOutput] representing this block's only output.
 *
 * @param value the constant value
 */
class Constant<T>(private val value: T) : SingleOutputBlock<T>(
    0, IN_FIRST_LAZY
) {
    override fun doInit(): T? = value
    override fun getOutput(inputs: List<Any?>): T = value
    override fun toString(): String = "Constant($value)"
}

/**
 * A block with only one output, [value], which can be changed externally.
 *
 * This is itself a [BlockOutput] representing this block's only output.
 *
 * @param value the value outputted
 */
class ExternalValue<T>(@Volatile var value: T) : SingleOutputBlock<T>(
    0, IN_FIRST_LAZY
) {
    override fun doInit(): T? = null
    override fun getOutput(inputs: List<Any?>): T = value
    override fun toString(): String = "ExternalConstant($value)"
}

/**
 * A block with only one input, and stores the value inputted in [value]. Useful for extracting information
 * out of a system.
 *
 * This is itself a [BlockInput] representing its only input.
 */
@Suppress("UNCHECKED_CAST")
class Monitor<T> : AbstractBlock(
    1, 0, Block.Processing.IN_FIRST_ALWAYS
), BlockInput<T> {
    @Volatile
    private var _value: T? = null
    /**
     * The last value given to this monitor. Will be `null` if nothing has been received yet (or the given value
     * is null).
     */
    val value: T? get() = _value

    override fun init() {
        _value = null
    }

    override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        _value = inputs[0] as T?
    }

    override fun getOutput(index: Int): Any? = throw IndexOutOfBoundsException(index)

    override val block: Block get() = this
    override val inputIndex: Int get() = 0
    override fun toString(): String = "Monitor($value)"
}

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

    override fun getOutput(inputs: List<Any?>): Boolean = queuePulse.compareAndSet(true, false)
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
    private var inputs: List<Any?>? = null
    override fun init() {
        inputs = null
    }

    override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        this.inputs = inputs
    }

    override fun getOutput(index: Int): Any? = inputs!![index]
}