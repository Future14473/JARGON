package org.futurerobotics.jargon.running

import kotlinx.coroutines.yield

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
     * @see [LoopSystemRunner]
     */
    @Throws(InterruptedException::class)
    override fun run() {
        try {
            system.start()
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

/**
 * Like a [LoopSystemRunner], but supports using a [SuspendFrequencyRegulator].
 *
 * This will _throw_ a cancellation exception if the coroutine running this is cancelled.
 *
 * `stop` are still placed in finally blocks.
 */
class SuspendLoopSystemRunner(
    private val system: LoopSystem, private val regulator: SuspendFrequencyRegulator
) : SuspendRunnable {

    /**
     * Runs the system.
     * @see [LoopSystemRunner]
     */
    override suspend fun runSuspend() {
        try {
            system.start()
            regulator.start()

            var elapsedNanos = 0L
            while (true) {
                yield() //throw if cancelled.
                if (system.loop(elapsedNanos)) break
                regulator.syncSuspend()
                elapsedNanos = regulator.getElapsedNanos()
            }
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
