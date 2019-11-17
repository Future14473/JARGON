package org.futurerobotics.jargon.system.looping

/**
 * A [Runnable] that runs a [LoopSystem], using with the given [regulator].
 * This is single threaded.
 *
 * When run, it calls start, runs repeatedly until either the loop calls to break or [InterruptedException] is
 * thrown, and then stops.
 *
 * [LoopSystem.stop] is placed in a `finally` block.
 */
class LoopSystemRunner(
    private val system: LoopSystem, private val regulator: LoopRegulator
) : Runnable {

    /**
     * Runs the system.
     * @see [LoopSystemRunner]
     */
    override fun run() {
        try {
            system.start()
            regulator.start()

            var elapsedNanos = 0L
            while (!Thread.interrupted()) {
                if (system.loop(elapsedNanos)) break
                val (delayTime, elapsed) = regulator.getDelayAndElapsedNanos()
                Thread.sleep(delayTime / 1_000_000, (delayTime % 1_000_000).toInt())
                elapsedNanos = elapsed
            }
        } catch (ignored: InterruptedException) {
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
