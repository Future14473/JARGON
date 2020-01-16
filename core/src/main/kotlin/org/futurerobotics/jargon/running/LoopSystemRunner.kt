package org.futurerobotics.jargon.running

/**
 * A [Runnable] that runs a [LoopSystem], using with the given [regulator].
 * This is single threaded.
 *
 * This will run repeatedly until the [LoopSystem] requests stop, or the thread is interrupted.
 *
 * [LoopSystem.stop] is placed in a `finally` block.
 *
 * See module `coroutine-extensions` for a suspend version if you are using coroutines.
 */
class LoopSystemRunner(
    private val system: LoopSystem, private val regulator: FrequencyRegulator
) : Runnable {

    /**
     * Runs the system.
     *
     * If the thread is interrupted, this will exit.
     * @see LoopSystemRunner
     */
    @Throws(InterruptedException::class)
    override fun run() {
        try {
            system.init()
            regulator.start()

            var elapsedNanos = 0L
            while (true) {
                if (system.loop(elapsedNanos)) break
                Thread.sleep(regulator.getDelayMillis())
                elapsedNanos = regulator.endLoopAndGetElapsedNanos()
            }
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
