package org.futurerobotics.jargon.running

import org.futurerobotics.jargon.running.Clock.SystemClock

/**
 * Represents a time keeping device. This can use real time or be manually set.
 * It has one function, [nanoTime] that returns the current relative time in nanoseconds.
 *
 * [SystemClock] (companion object) implementation uses [System.nanoTime].
 */
interface Clock {

    /** Gets the current (relative) time in nanoseconds */
    fun nanoTime(): Long

    /**
     * The default implementation of a [Clock] which uses System.nanoTime
     */
    companion object SystemClock : Clock {

        override fun nanoTime(): Long = System.nanoTime()
    }
}

/**
 * A [Clock] where the [nanoTime] can be set manually.
 *
 * @property nanoTime the time returned by [nanoTime]
 */
class ManualClock(var nanoTime: Long = 0) : Clock {

    override fun nanoTime(): Long = nanoTime
}

/**
 * A clock that always outputs a time a fixed number of [nanos] greater than the previous time.
 *
 * @param nanos The time in nanoseconds each step.
 */
class FixedTestClock(var nanos: Long) : Clock {

    private var time = 0L
    override fun nanoTime(): Long {
        time += nanos
        return time
    }
}
