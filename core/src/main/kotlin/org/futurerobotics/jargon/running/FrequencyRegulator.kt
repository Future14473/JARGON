package org.futurerobotics.jargon.running

import kotlin.math.roundToLong

/**
 * A [FrequencyRegulator] is used in systems to control how often a [LoopSystem] is run.
 *
 * It both keeps track of elapsed time per cycle, and possibly enforces a cycle to be run at a certain speed.
 */
interface FrequencyRegulator {

    /**
     * This is called to indicate the start of timing.
     */
    fun start()

    /**
     * Gets the amount of nanoseconds needed to delay so that the time between the last call to [start]/[getDelayNanos] is
     * controlled to this [FrequencyRegulator]'s liking.
     *
     * Usually [endLoopAndGetElapsedNanos] will be called next.
     */
    fun getDelayNanos(): Long

    /**
     * Gets the amount of nanoseconds since the last call to start, and then restarts the loop.
     *
     * This is usually called after [getDelayNanos].
     *
     * if [getDelayNanos] has not been called, behaviour is undefined.
     */
    fun endLoopAndGetElapsedNanos(): Long

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
@JvmOverloads constructor(private val clock: Clock = Clock.DEFAULT) : FrequencyRegulator {

    @Volatile
    private var lastNanos = clock.nanoTime()

    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun getDelayNanos(): Long = 0L

    override fun endLoopAndGetElapsedNanos(): Long {
        val nanos = clock.nanoTime()
        return nanos - lastNanos
            .also { lastNanos = nanos }
    }

    override fun stop() {
    }
}

/**
 * A [FrequencyRegulator] that limits its maximum speed to a certain given `period`, and delays if slower. Note that if
 * the block is running slow it will attempt to catch up over the next cycles by delaying.
 *
 * With the current implementation there is no limit to how much "catching up" might occur.
 */
class MaxSpeedRegulator(period: Double, private val clock: Clock = Clock.DEFAULT) : FrequencyRegulator {

    private val periodNanos = (period * 1e9).roundToLong()

    private var lastNanos = clock.nanoTime()

    override fun start() {
        elapsed = -1
        lastNanos = clock.nanoTime()
    }

    private var elapsed = -1L

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

    override fun endLoopAndGetElapsedNanos(): Long =
        if (elapsed != -1L) elapsed
        else clock.nanoTime() - lastNanos

    override fun stop() {
    }
}
