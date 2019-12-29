package org.futurerobotics.jargon.running

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import kotlinx.coroutines.yield

/**
 * A [FrequencyRegulator] that instead supports sync using a suspend function, namely [syncSuspend].
 *
 * The [sync] function should use runblocking.
 */
interface SuspendFrequencyRegulator : FrequencyRegulator {

    /**
     * Like [sync], but suspends instead of blocking.
     */
    suspend fun syncSuspend()
}

/**
 * A [SuspendFrequencyRegulator] which has a sync based delaying a number of milliseconds, using [getDelayMillis].
 */
abstract class DelaySuspendFrequencyRegulator : DelayFrequencyRegulator(), SuspendFrequencyRegulator {

    override suspend fun syncSuspend() {
        delay(getDelayMillis())
    }
}

private fun DelayFrequencyRegulator.delaySuspendRegulatorFromNonSuspend(): DelaySuspendFrequencyRegulator =
    this as? DelaySuspendFrequencyRegulator
        ?: object : DelaySuspendFrequencyRegulator(), FrequencyRegulator by this {
            override fun getDelayMillis() = this@delaySuspendRegulatorFromNonSuspend.getDelayMillis()
            override fun sync() = super.sync()
        }

private fun FrequencyRegulator.asSuspendFrequencyRegulator(): SuspendFrequencyRegulator =
    this as? SuspendFrequencyRegulator
        ?: (this as? DelayFrequencyRegulator)?.delaySuspendRegulatorFromNonSuspend()
        ?: object : SuspendFrequencyRegulator, FrequencyRegulator by this {
            @Suppress("BlockingMethodInNonBlockingContext")
            override suspend fun syncSuspend() = withContext(Dispatchers.IO) {
                sync()
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
    private val system: LoopSystem, regulator: FrequencyRegulator
) : SuspendFunction<Unit> {

    private val regulator = regulator.asSuspendFrequencyRegulator()
    /**
     * Runs the system.
     * @see LoopSystemRunner
     */
    override suspend fun invoke() {
        try {
            system.init()
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
