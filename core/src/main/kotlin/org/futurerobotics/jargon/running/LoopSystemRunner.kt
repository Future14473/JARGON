package org.futurerobotics.jargon.running

/**
 * A [Runnable] that runs a [LoopSystem], using with the given [regulator].
 * This is single threaded.
 *
 * This will run repeatedly until the [LoopSystem] requests stop, or the thread is interrupted (will simply exit).
 *
 * [LoopSystem.stop] is placed in a `finally` block.
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
            while (!Thread.interrupted()) {
                if (system.loop(elapsedNanos)) break
                regulator.sync()
                elapsedNanos = regulator.getElapsedNanos()
            }
        } catch (_: InterruptedException) {
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
