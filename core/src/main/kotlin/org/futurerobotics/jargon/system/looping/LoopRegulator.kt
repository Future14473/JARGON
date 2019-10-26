package org.futurerobotics.jargon.system.looping

import org.futurerobotics.jargon.system.looping.Clock.Default
import kotlin.math.roundToLong

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
        val default: Default = Default
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
 */
class FixedTestClock(var nanos: Long) : Clock {

    private var time = 0L
    override fun nanoTime(): Long {
        time += nanos
        return time
    }
}

/**
 * A [LoopRegulator] is used in systems to control how often a [LoopSystem] is run.
 *
 * It both keeps track of elapsed time per cycle, and possibly enforces a cycle to be run at a certain speed.
 *
 * Every control system has exactly one of these.
 */
interface LoopRegulator {

    /**
     * This is called to indicate the start of a loop.
     */
    fun start()

    /**
     * Possibly pauses the current thread so that the time between the last call to [start] is
     * controlled to this [LoopRegulator]'s liking, then returns the elapsed time in nanoseconds.
     *
     * The sum of the elapsed times given should equal any used [Clock]'s elapsed nanoTime as closely as possible.
     *
     * This is also allowed to throw [InterruptedException] if either the thread is interrupted or to inidcate an
     * immediate quitting of the loop.
     */
    @Throws(InterruptedException::class)
    fun syncAndRestart(): Long

    /**
     * Stops timing.
     */
    fun stop()
}

/**
 * A [LoopRegulator] that runs its cycle as fast as possible; timing using the given [clock].
 */
open class LoopAsFastAsPossible(private val clock: Clock = Default) : LoopRegulator {

    private var lastNanos = clock.nanoTime()
    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun syncAndRestart(): Long {
        val nanos = clock.nanoTime()
        return (nanos - lastNanos)
            .also { lastNanos = nanos }
    }

    override fun stop() {
    }
}

/**
 * A [LoopRegulator] that limits its maximum speed to a certain number of cycles per second;
 * otherwise will sleep the running thread. Note that if the block is running slow it will attempt
 * to catch up over the next cycles by not sleeping if possible.
 *
 * With the current implementation there is no limit to how much "catching up" might occur.
 *
 * @param maxHertz the maximum hertz to run the loop at.
 */
class LoopWithMaxSpeed(maxHertz: Double, private val clock: Clock = Default) : LoopRegulator {

    private var lastNanos = clock.nanoTime()
    private val minNanos = (1e9 / maxHertz).roundToLong()
    override fun start() {
        lastNanos = clock.nanoTime()
    }

    override fun syncAndRestart(): Long {
        val nanos = clock.nanoTime()
        val elapsed = nanos - lastNanos

        return if (elapsed >= minNanos) {
            lastNanos = nanos
            elapsed
        } else {
            val neededNanos = minNanos - elapsed
            lastNanos += minNanos

            Thread.sleep(neededNanos / 1_000_000, (neededNanos % 1_000_000).toInt())
            minNanos
        }
    }

    override fun stop() {
    }
}

/**
 * A loop regulator that limits the total number of loops that can happen, and wraps an[otherRegulator].
 *
 * At least one loop will always be run.
 */
class LoopMaxTimes @JvmOverloads constructor(
    private val maxTimes: Int,
    private val otherRegulator: LoopRegulator = LoopAsFastAsPossible()
) : LoopRegulator {

    private var timesRun = 0

    init {
        require(maxTimes > 0) { "Max times ($maxTimes) cannot be negative." }
    }

    override fun start() {
        timesRun = 0
        otherRegulator.start()
    }

    override fun syncAndRestart(): Long {
        if (++timesRun >= maxTimes) throw InterruptedException()
        return otherRegulator.syncAndRestart()
    }

    override fun stop() {
        timesRun = 0
        otherRegulator.stop()
    }
}
