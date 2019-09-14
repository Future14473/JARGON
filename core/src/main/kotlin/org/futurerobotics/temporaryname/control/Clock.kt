package org.futurerobotics.temporaryname.control

import kotlin.math.roundToLong

/**
 * Represents a time keeping device. This can use real time or be manually set.
 * Has one function [nanoTime] that returns the current relative time in nanoseconds.
 *
 * Default implementation simply uses [System.nanoTime].
 * For other implementations, see
 */
interface Clock {

    /** Gets the current time in nanoseconds */
    fun nanoTime(): Long

    /** Default clock implementation that uses [System.nanoTime] */
    object Default : Clock {

        override fun nanoTime(): Long = System.nanoTime()
    }
}

/**
 * A [Clock] where the [nanoTime] can be set directly.
 *
 * @property nanoTime the time returned by [nanoTime]
 */
class ManualClock(var nanoTime: Long = 0) : Clock {

    override fun nanoTime(): Long = nanoTime
}

/**
 * A [LoopRegulator] is used in control systems to control how the control loop is run.
 *
 * It both keeps track of elapsed time per cycle, and possibly enforces a cycle to be run at a certain speed.
 *
 * Every control system has exactly one of these.
 */
interface LoopRegulator {
    /**
     * Indicates the start of a cycle.
     */
    fun start()


    /**
     * Possibly pauses the current thread so that the time between the last call to [start] is
     * controlled to this [LoopRegulator]'s liking; and then returns the elapsed time in nanoseconds.
     */
    fun syncAndTime(): Long
}

/**
 * Possibly pauses the current thread so that the time between the last call to [start] is
 * controlled to this [LoopRegulator]'s liking; and then returns the elapsed time in _seconds_.
 */
fun LoopRegulator.syncAndTimeSeconds(): Double = syncAndTime() / 1e9

/**
 * A [LoopRegulator] that runs its cycle as fast as possible; timing using the given [clock].
 */
class LoopAsFastAsPossible(private val clock: Clock = Clock.Default) : LoopRegulator {
    private var lastNanos = clock.nanoTime()
    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun syncAndTime(): Long {
        val nanos = clock.nanoTime()
        return (nanos - lastNanos).also {
            lastNanos = nanos
        }
    }
}

/**
 * A [LoopRegulator] that limits its maximum speed to a certain number of cycles per second;
 * otherwise will stop thread.
 *
 * @param maxHertz the maximum hertz to run the loop at.
 */
class LoopAtMaximumHertz(maxHertz: Double, private val clock: Clock = Clock.Default) : LoopRegulator {
    private var lastNanos = clock.nanoTime()
    private val minPeriod = (1e9 / maxHertz).roundToLong()
    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun syncAndTime(): Long {
        val elapsed = clock.nanoTime() - lastNanos
        if (elapsed >= minPeriod) return elapsed
        Thread.sleep((minPeriod - elapsed) / 1000000)
        return minPeriod
    }

}