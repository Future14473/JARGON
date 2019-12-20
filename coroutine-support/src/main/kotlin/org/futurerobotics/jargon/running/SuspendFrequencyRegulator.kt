package org.futurerobotics.jargon.running

import kotlinx.coroutines.*

/**
 * A [FrequencyRegulator] that instead supports sync using a suspend function, namely [syncSuspend].
 *
 * The sync function then defaults to use [runBlocking].
 */
interface SuspendFrequencyRegulator : FrequencyRegulator {

    /**
     * Gets the amount of nanoseconds needed to delay so that the time between the last call to [start] is
     * controlled to this [FrequencyRegulator]'s liking, for the next loop after the call to [start].
     */
    suspend fun syncSuspend()

    override fun sync(): Unit = runBlocking {
        syncSuspend()
    }
}

private const val million = 1_000_000

/**
 * A [SuspendFrequencyRegulator] which has a sync based delaying a number of milliseconds, using [getDelayMillis].
 */
abstract class DelaySuspendFrequencyRegulator : DelayFrequencyRegulator(), SuspendFrequencyRegulator {

    override suspend fun syncSuspend() {
        delay(getDelayMillis())
    }

    override fun sync(): Unit = super<SuspendFrequencyRegulator>.sync()
}

private fun DelayFrequencyRegulator.delaySuspendRegulatorFromNonSuspend(): DelaySuspendFrequencyRegulator =
    this as? DelaySuspendFrequencyRegulator
        ?: object : DelaySuspendFrequencyRegulator(), FrequencyRegulator by this {
            override fun getDelayMillis(): Long = this@delaySuspendRegulatorFromNonSuspend.getDelayMillis()
            override fun sync() = this@delaySuspendRegulatorFromNonSuspend.sync()
        }

private fun FrequencyRegulator.asSuspendFrequencyRegulator(): SuspendFrequencyRegulator =
    this as? SuspendFrequencyRegulator
        ?: (this as? DelayFrequencyRegulator)?.delaySuspendRegulatorFromNonSuspend()
        ?: object : SuspendFrequencyRegulator, FrequencyRegulator by this {
            override suspend fun syncSuspend() = withContext(Dispatchers.IO) { sync() }
            override fun sync() = this@asSuspendFrequencyRegulator.sync()
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
) : SuspendRunnable {

    private val regulator = regulator.asSuspendFrequencyRegulator()
    /**
     * Runs the system.
     * @see LoopSystemRunner
     */
    override suspend fun runSuspend() {
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
