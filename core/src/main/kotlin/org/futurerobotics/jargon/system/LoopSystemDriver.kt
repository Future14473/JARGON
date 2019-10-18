package org.futurerobotics.jargon.system

/**
 * Represents something that  can run a [LoopSystem] in a specific way. Sort of a strategy pattern.
 */
interface LoopSystemDriver {
    /**
     * Runs the given [system]
     */
    fun run(system: LoopSystem)
}

/**
 * A [Runnable] around a [LoopSystem] that simply runs the system until completion or the thread is interrupted.
 *
 * [LoopSystem.stop] is placed in a finally block.
 */
class SimpleLoopSystemDriver(private val loopRegulator: LoopRegulator = LoopAsFastAsPossible()) :
    LoopSystemDriver {
    @Volatile
    private var thread: Thread? = null

    override fun run(system: LoopSystem) {
        try {
            thread = Thread.currentThread()
            system.init()
            loopRegulator.start()

            var loopTimeInNanos: Long = 0
            while (!Thread.interrupted()) {
                if (system.loop(loopTimeInNanos)) break
                loopTimeInNanos = loopRegulator.syncAndRestart()
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

/**
 * A [Runnable] around a [LoopSystem] that runs the system until completion, the thread is interrupted, or
 * it has run [maxTimes]
 *
 * [LoopSystem.stop] is placed in a finally block.
 */
class LimitedLoopSystemDriver(
    private val maxTimes: Int,
    private val loopRegulator: LoopRegulator = LoopAsFastAsPossible()
) : LoopSystemDriver {
    @Volatile
    private var thread: Thread? = null

    override fun run(system: LoopSystem) {
        try {
            thread = Thread.currentThread()
            system.init()
            loopRegulator.start()

            var loopTime: Long = 0
            var i = 0
            while (i < maxTimes && !Thread.interrupted()) {
                if (system.loop(loopTime)) break
                loopTime = loopRegulator.syncAndRestart()
                i++
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

