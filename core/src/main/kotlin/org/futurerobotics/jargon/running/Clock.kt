package org.futurerobotics.jargon.running

/**
 * Represents a time keeping device. This can use real time or be manually set.
 * It has one function, [nanoTime] that returns the current relative time in nanoseconds.
 *
 * [Default] implementation uses [System.nanoTime].
 */
interface Clock {

    /** Gets the current time in nanoseconds */
    fun nanoTime(): Long

    /** Default clock implementation that uses [System.nanoTime] */
    object Default : Clock {

        override fun nanoTime(): Long = System.nanoTime()
    }

    companion object {
        /** The default clock that uses [System.nanoTime]. For java people. */
        @JvmField
        val default: Clock = Default
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
 * A clock that always outputs a time [nanos] nanoseconds after the previous.
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
