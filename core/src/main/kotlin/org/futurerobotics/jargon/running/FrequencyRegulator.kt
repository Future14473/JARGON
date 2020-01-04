package org.futurerobotics.jargon.running

import kotlin.math.roundToLong

/**
 * A [FrequencyRegulator] is used in systems to control how often a [LoopSystem] is run.
 *
 * It both keeps track of elapsed time per cycle, and possibly enforces a cycle to be run at a certain speed.
 *
 * If using in kotlin, see `coroutine-support` package for a version of this that uses coroutines instead
 * of thread blocking instead.
 */
interface FrequencyRegulator {

    /**
     * This is called to indicate the start of a loop.
     */
    fun start()

    /**
     * Gets the amount of nanoseconds needed to delay so that the time between the last call to [start]/[sync] is
     * controlled to this [FrequencyRegulator]'s liking, then restarts the loop.
     *
     * After syncing, computes the elapsed nanos to later get in [getElapsedNanos].
     */
    @Throws(InterruptedException::class)
    fun sync()

    /**
     * Gets the amount of nanoseconds since the last call to start, _determined right after calling _sync_.
     *
     * if [sync] has not been called, behaviour is undefined.
     */
    fun getElapsedNanos(): Long

    /**
     * Stops timing.
     */
    fun stop()
}

private const val million = 1_000_000

/**
 * A [FrequencyRegulator] which has a sync based delaying a number of nanoseconds, using [getDelayNanos].
 */
abstract class DelayFrequencyRegulator : FrequencyRegulator {

    /**
     * Gets the amount of delay needed, in milliseconds.
     */
    abstract fun getDelayMillis(): Long

    override fun sync() {
        Thread.sleep(getDelayMillis())
    }
}

/**
 * A [FrequencyRegulator] that runs its cycle as fast as possible; timing using the given [clock].
 */
open class UnregulatedRegulator(private val clock: Clock = Clock.DEFAULT) : FrequencyRegulator {

    @Volatile
    private var lastNanos = clock.nanoTime()

    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun sync() {
    }

    override fun getElapsedNanos(): Long {
        val nanos = clock.nanoTime()
        return (nanos - lastNanos)
            .also { lastNanos = nanos }
    }

    override fun stop() {
    }
}

/**
 * A [FrequencyRegulator] that limits its maximum speed to a certain given `period`, and delays if slower. Note that if
 * the block is running slow it will attempt to catch up over the next cycles by sleeping less if possible.
 *
 * With the current implementation there is no limit to how much "catching up" might occur.
 */
class MaxSpeedRegulator(period: Double, private val clock: Clock = Clock.DEFAULT) : DelayFrequencyRegulator() {

    private val periodNanos = (period * 1e9).roundToLong()

    private var lastNanos = clock.nanoTime()

    override fun start() {
        elapsed = -1
        lastNanos = clock.nanoTime()
    }

    private var elapsed = -1L

    override fun getDelayMillis(): Long {
        val nanos = clock.nanoTime()
        elapsed = nanos - lastNanos
        return if (elapsed >= periodNanos) {
            lastNanos = nanos
            0L
        } else {
            lastNanos += periodNanos
            ((periodNanos - elapsed) / 1_000_000).also {
                elapsed = periodNanos
            }
        }
    }

    override fun getElapsedNanos(): Long =
        if (elapsed != -1L) elapsed
        else clock.nanoTime() - lastNanos

    override fun stop() {
    }
}
