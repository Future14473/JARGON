package org.futurerobotics.jargon.running

/**
 * Represents something that can be run as a loop, using [init], then [loop] which is called repeatedly, and finally
 * [stop].
 *
 * This can later be run using a [LoopSystemRunner].
 */
interface LoopSystem : InitStoppable {

    /** Performs any initialization before loop start. */
    override fun init() {
    }

    /**
     * Runs one cycle of the loop. Information about the last [loopTimeInNanos] should be given; 0 if not known (first loop).
     *
     * Return value indicates if to stop looping; `true` to indicate to break the loop, `false` to continue.
     */
    fun loop(loopTimeInNanos: Long = 0L): Boolean

    /** Run when loop is stopped or interrupted. */
    override fun stop() {
    }
}

/**
 * A loop system that is built up of a list of other [systems]. All will be inited, looped, and stopped in the same order.
 */
class CompositeLoopSystem : LoopSystem {

    private val systems: List<LoopSystem>

    constructor(systems: List<LoopSystem>) {
        this.systems = systems.toList()
    }

    constructor(vararg systems: LoopSystem) {
        this.systems = systems.asList()
    }

    override fun init() {
        systems.forEach { it.init() }
    }

    override fun loop(loopTimeInNanos: Long): Boolean {
        var shouldShutdown = false
        systems.forEach {
            shouldShutdown = it.loop(loopTimeInNanos) || shouldShutdown
        }
        return shouldShutdown
    }

    override fun stop() {
        systems.forEach { it.stop() }
    }
}
