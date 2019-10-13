package org.futurerobotics.jargon.system

/**
 * Represents something that can be run with a loop, using [init], then [loop] which is called repeatedly, and finally
 * [stop] methods.
 *
 * see [AbstractLoopSystemDriver]
 */
interface LoopSystem : InitStoppable {

    /**
     * Runs one cycle of the loop, also using the [loopTime] of the last loop; or Double.NAN if not known
     * (first loop).
     *
     * Return value tells if to stop looping; returns `true` to indicate to break the loop, `false` to continue.
     */
    fun loop(loopTime: Double = Double.NaN): Boolean
}

/**
 * A driver for a [LoopSystem], [System], that implements [Runnable]
 * @param system the [LoopSystem]
 */
abstract class AbstractLoopSystemDriver(protected val system: LoopSystem) : Runnable {
    /**
     * Runs the loop based system.
     */
    abstract override fun run()
}

/**
 * A [Runnable] around a [LoopSystem] that simply runs the system until completion or the thread is interrupted.
 *
 * [LoopSystem.stop] is placed in a finally block.
 */
class SimpleLoopSystemDriver(system: LoopSystem, private val loopRegulator: LoopRegulator = LoopAsFastAsPossible()) :
    AbstractLoopSystemDriver(system) {
    @Volatile
    private var thread: Thread? = null

    override fun run() {
        try {
            thread = Thread.currentThread()
            system.init()
            loopRegulator.start()

            var elapsedTime: Double = Double.NaN
            while (!Thread.interrupted()) {
                if (system.loop(elapsedTime)) break
                elapsedTime = loopRegulator.syncAndRestart()
            }
        } finally {
            thread = null
            loopRegulator.stop()
            system.stop()
        }
    }

    /**
     * Interrupts the current running thread of this driver, if any.
     */
    fun interrupt() {
        thread?.interrupt()
    }
}
