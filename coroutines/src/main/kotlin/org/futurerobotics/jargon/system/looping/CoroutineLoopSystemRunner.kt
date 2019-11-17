package org.futurerobotics.jargon.system.looping

import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import org.futurerobotics.jargon.util.SuspendRunnable
import kotlin.coroutines.coroutineContext

/**
 * Like a [LoopSystemRunner], but runs CoroutinesInstead
 */
class CoroutineLoopSystemRunner(
    private val system: LoopSystem, private val regulator: LoopRegulator
) : SuspendRunnable {

    /**
     * Runs the system.
     * @see [LoopSystemRunner]
     */
    override suspend fun run() {
        try {
            system.start()
            regulator.start()

            var elapsedNanos = 0L
            while (coroutineContext.isActive) {
                if (system.loop(elapsedNanos)) break
                val (delayTime, elapsedTime) = regulator.getDelayAndElapsedNanos()
                delay((delayTime + 1_000_000 / 2) / 1_000_000)
                elapsedNanos = elapsedTime
            }
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
