package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * A component that contains a concurrent [queue] which is polled when outputs are requested from this block. If the
 * queue is empty, returns `null`. This queue can be modified externally.
 *
 * This can, for instance, be used queue up motion profiles to feed into a [MotionProfileFollower]
 */
class ExternalQueue<T> private constructor(
    private val clearOnStart: Boolean, private val queue: Queue<T>
) : PrincipalOutputBlock<T>(Processing.LAZY), Queue<T> by queue {

    /**
     * @param clearOnStart when this is true, the queue is cleared when the system starts.
     * @see [ExternalQueue]
     */
    @JvmOverloads
    constructor(clearOnStart: Boolean = false) : this(clearOnStart, ConcurrentLinkedQueue())

    override fun init() {
        if (clearOnStart) queue.clear()
    }

    override fun Context.getOutput(): T = queue.poll()
}
