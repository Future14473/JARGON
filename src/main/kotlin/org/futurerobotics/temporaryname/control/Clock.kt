package org.futurerobotics.temporaryname.control

/**
 * A interface to represent a Time-keeping device.
 * Has one function [nanoTime] that returns the current relative time in nanoseconds.
 *
 * Default implementation simply uses [System.nanoTime].
 * For other implementations, see
 */
interface Clock {
    /** Gets the current time in nanoseconds */
    fun nanoTime(): Long

    /** Default clock implementation that uses [System.nanoTime] */
    object Default : org.futurerobotics.temporaryname.control.Clock {
        override fun nanoTime(): Long = System.nanoTime()
    }
}

/**
 * A [Clock] does not change its the time returned by [nanoTime] until [tick] is called, in which it will update
 * its internal time to the [referenceClock]'s time.
 *
 * This can be useful to make sure multiple systems are running with the same time info.
 */
class TickClock(private val referenceClock: org.futurerobotics.temporaryname.control.Clock = org.futurerobotics.temporaryname.control.Clock.Default) :
    org.futurerobotics.temporaryname.control.Clock {
    //    @Volatile
    private var time = referenceClock.nanoTime()

    override fun nanoTime(): Long = time
    /**
     * "ticks" the clock; sets the internal time to the [referenceClock]'s time.
     */
    fun tick() {
        time = referenceClock.nanoTime()
    }
}

/**
 * A manual clock where the [time] can be set directly.
 *
 * @property time the time returned by [nanoTime]
 */
class ManualClock(var time: Long = 0) : org.futurerobotics.temporaryname.control.Clock {
    override fun nanoTime(): Long = time
}