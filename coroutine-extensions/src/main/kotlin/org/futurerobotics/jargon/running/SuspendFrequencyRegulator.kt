package org.futurerobotics.jargon.running

import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import org.futurerobotics.jargon.util.SuspendFunction

/**
 * Like a [LoopSystemRunner], but uses coroutine delay.
 *
 * Runs until loop system ends or coroutine is cancelled.
 *
 * `stop` are still placed in finally blocks.
 */
class SuspendLoopSystemRunner(
    private val system: LoopSystem, private val regulator: FrequencyRegulator
) : SuspendFunction<Unit> {

    /**
     * Runs the system.
     * @see LoopSystemRunner
     */
    override suspend fun invoke() {
        try {
            system.init()
            regulator.start()
            yield()
            var elapsedNanos = 0L
            while (true) {
                if (system.loop(elapsedNanos)) break
                delay(regulator.getDelayMillis())
                elapsedNanos = regulator.endLoopAndGetElapsedNanos()
            }
        } finally {
            system.stop()
            regulator.stop()
        }
    }
}
