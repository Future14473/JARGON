package org.futurerobotics.jargon.system.looping

/**
 * A [Runnable] that runs a [LoopSystem], using with the given [regulator].
 *
 * When run, it calls start, runs repeatedly until either the loop calls to break or [InterruptedException] is
 * thrown, and then stops.
 *
 * [LoopSystem.stop] is placed in a `finally` block, and any exception thrown during stopping is caught and
 * printed to System.err. Generally try not to throw things around.
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

            var loopTimeInNanos = 0L
            while (!Thread.interrupted()) {
                if (system.loop(loopTimeInNanos)) break
                loopTimeInNanos = regulator.syncAndRestart()
            }
        } catch (ignored: InterruptedException) {
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
