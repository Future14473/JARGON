package org.futurerobotics.temporaryname.control

/**
 * A interface to represent a something that keepins time.
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
 * A [Clock] does not change its the time returned by [nanoTime] until [tick] is called, in which it will update
 * its internal time to the [referenceClock]'s time.
 *
 * This can be useful to make sure multiple systems are running with the same time info.
 */
class TickClock(private val referenceClock: Clock = Clock.Default) : Clock {

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
class ManualClock(var time: Long = 0) : Clock {

    override fun nanoTime(): Long = time
}

/**
 * A Stopwatch to time events that uses the given [clock]
 */
class Stopwatch(private val clock: Clock = Clock.Default) {

    private var lastTimeStamp = clock.nanoTime()
    /**
     * Starts this stopwatch.
     */
    fun start() {
        lastTimeStamp = clock.nanoTime()
    }

    /**
     * Returns the elapsed time since the last call to [start] in nanoseconds.
     */
    fun nanos(): Long = clock.nanoTime() - lastTimeStamp

    fun seconds(): Double = nanos() / 1e9
}

/**
 * Measures the time to run the block in milliseconds
 */
inline fun Stopwatch.measureTimeMillis(block: () -> Unit): Double {
    start()
    block()
    return nanos() / 1e3
}

/**
 * Measures the time to run the block in nanoseconds.
 */
inline fun Stopwatch.measureTimeNanos(block: () -> Unit): Long {
    start()
    block()
    return nanos()
}