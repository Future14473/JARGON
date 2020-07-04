package org.futurerobotics.jargon.running

import kotlin.math.roundToLong

/**
 * A [FrequencyRegulator] is used in systems to control how often a [LoopSystem] is run.
 *
 * It both keeps track of elapsed time per cycle, and possibly enforces a cycle to be run at a certain speed.
 */
interface FrequencyRegulator {

    /**
     * Called to indicate the start of timing.
     */
    fun start()

    /**
     * Gets the amount of nanoseconds needed to delay so that the time between the last call to [start]/[getDelayNanos] is
     * controlled to this [FrequencyRegulator]'s liking.
     *
     * Usually [getElapsedNanosAndEndLoop] will be called next.
     */
    fun getDelayNanos(): Long

    /**
     * Gets the amount of nanoseconds since the last call to start, and then restarts the loop.
     *
     * This is usually called after [getDelayNanos].
     */
    fun getElapsedNanosAndEndLoop(): Long

    /**
     * Stops timing.
     */
    fun stop()
}

/**
 * Gets the delay but rounded to milliseconds.
 */
fun FrequencyRegulator.getDelayMillis(): Long = (getDelayNanos() + 500_000) / 1_000_000

/**
 * A [FrequencyRegulator] that runs its cycle as fast as possible; timing using the given [clock].
 */
open class UnregulatedRegulator
@JvmOverloads constructor(private val clock: Clock = Clock) : FrequencyRegulator {

    @Volatile
    private var lastNanos = clock.nanoTime()

    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun getDelayNanos(): Long = 0L

    override fun getElapsedNanosAndEndLoop(): Long {
        val nanos = clock.nanoTime()
        return nanos - lastNanos
            .also { lastNanos = nanos }
    }

    override fun stop() {
    }
}

/**
 * A [FrequencyRegulator] that limits its maximum speed to a certain given `period`, and delays if slower.
 *
 * However, if running slow, it may attempt to "catch up" over the next cycles by not delaying when it could. With
 * the current implementation there is no limit to how much "catching up" might occur.
 */
class MaxSpeedRegulator(period: Double, private val clock: Clock = Clock) : FrequencyRegulator {

    private val periodNanos = (period * 1e9).roundToLong()

    private var lastNanos = clock.nanoTime()

    private var elapsed = -1L

    override fun start() {
        elapsed = -1
        lastNanos = clock.nanoTime()
    }

    override fun getDelayNanos(): Long {
        val nanos = clock.nanoTime()
        elapsed = nanos - lastNanos
        return if (elapsed >= periodNanos) {
            lastNanos = nanos
            0L
        } else {
            lastNanos += periodNanos
            (periodNanos - elapsed).also {
                elapsed = periodNanos
            }
        }
    }

    override fun getElapsedNanosAndEndLoop(): Long =
        if (elapsed != -1L) elapsed
        else clock.nanoTime() - lastNanos

    override fun stop() {
    }
}
