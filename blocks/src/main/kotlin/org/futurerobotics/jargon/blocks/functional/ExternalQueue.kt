package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.blocks.control.MotionProfileFollower
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * A block that is a [queue], where elements are polled from the queue when outputs are requested from this block. If
 * the queue is empty, returns `null`. This queue can be modified externally.
 *
 * This can, for instance, be used queue up motion profiles to feed into a [MotionProfileFollower].
 */
class ExternalQueue<T> private constructor(
    private val clearOnStop: Boolean, private val queue: Queue<T>
) : PrincipalOutputBlock<T>(Processing.LAZY), Queue<T> by queue {

    /**
     * @param clearOnStop when this is true, the queue is cleared when the system starts.
     * @see ExternalQueue
     */
    @JvmOverloads
    constructor(clearOnStop: Boolean = false) : this(clearOnStop, ConcurrentLinkedQueue())

    override fun stop() {
        if (clearOnStop) queue.clear()
    }

    override fun Context.getOutput(): T = queue.poll()
}
