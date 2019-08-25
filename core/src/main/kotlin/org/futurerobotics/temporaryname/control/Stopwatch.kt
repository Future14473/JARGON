package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.MathUnits.microseconds
import org.futurerobotics.temporaryname.math.MathUnits.milliseconds
import org.futurerobotics.temporaryname.math.MathUnits.minutes
import org.futurerobotics.temporaryname.math.MathUnits.nanoseconds
import org.futurerobotics.temporaryname.math.MathUnits.seconds

/**
 * A Stopwatch to time events that uses the given [clock]
 */
class Stopwatch(private val clock: Clock) {
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

    /**
     * Returns the current time in nanoseconds, and restarts this stopwatch
     */
    fun restart(): Long = nanos().also { start() }

    /**
     * Returns the elapsed time since the last call to [start] in microseconds.
     */
    fun micros(): Double = nanos() * microseconds / nanoseconds

    /**
     * Returns the elapsed time since the last call to [start] in milliseconds.
     */
    fun millis(): Double = nanos() * milliseconds / nanoseconds

    /**
     * Returns the elapsed time since the last call to [start] in milliseconds.
     */
    fun seconds(): Double = nanos() * seconds / nanoseconds

    /**
     * Returns the elapsed time since the last call to [start] in nanoseconds.
     */
    fun minutes(): Double = nanos() * minutes / nanoseconds

    /**
     * Measures the time to run the block in nanoseconds.
     */
    inline fun measureTimeNanos(block: () -> Unit): Long {
        start()
        block()
        return nanos()
    }

    /**
     * Measures the time to run the block in milliseconds
     */
    inline fun measureTimeMillis(block: () -> Unit): Double {
        start()
        block()
        return millis()
    }
}