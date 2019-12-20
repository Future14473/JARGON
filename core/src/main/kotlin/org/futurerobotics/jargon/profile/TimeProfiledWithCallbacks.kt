package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.util.Stepper
import java.util.*
import java.util.concurrent.CompletableFuture
import java.util.concurrent.Future

/**
 * A wrapper around another [profiled] object that also provides callbacks in [Future]s, via [addCallback].
 *
 * The callbacks will be triggered whenever [atTime] or [stepper] is called with a time greater than the
 * callback's time.
 *
 * Callbacks will only be triggered if the time called happens _after_ the callback is added. We would do a
 * builder pattern thing for immutability but this is simpler to use...
 *
 * @see ProfileTimeIndicator
 */
class TimeProfiledWithCallbacks<out T : Any, P : TimeProfiled<T>>(private val profiled: P) : TimeProfiled<T> {

    private val callbacks = TreeSet<TimeFuture>()

    private class TimeFuture(val time: Double, val future: CompletableFuture<Nothing?>) : Comparable<TimeFuture> {
        override fun compareTo(other: TimeFuture) = time.compareTo(other.time)
    }

    /**
     * Adds a [Future] that will be complete when traversed pass the given [time].
     */
    fun addCallback(time: ProfileTimeIndicator<P>): Future<*> = addCallback(time.getTime(profiled))

    /**
     * Adds a [Future] that will be complete when traversed pass the given [time].
     */
    fun addCallback(time: Double): Future<*> {
        require(time.isFinite()) { "Time ($time) must be finite" }
        val future = CompletableFuture<Nothing?>()
        callbacks += TimeFuture(
            time,
            future
        )
        return future
    }

    override val duration: Double
        get() = profiled.duration

    override fun atTime(time: Double): T {
        runFutures(time)
        return profiled.atTime(time)
    }

    override fun stepper(): Stepper<T> {
        val stepper = profiled.stepper()
        return Stepper { t ->
            runFutures(t)
            stepper.stepTo(t)
        }
    }

    private fun runFutures(time: Double) {
        while (callbacks.isNotEmpty() && callbacks.first().time <= time)
            callbacks.pollFirst()!!.future.complete(null)
    }
}

/**
 * Creates a [TimeProfiledWithCallbacks] using [this] profiled.
 */
fun <T : Any, P : TimeProfiled<T>> P.withCallbacks(): TimeProfiledWithCallbacks<T, P> =
    TimeProfiledWithCallbacks(this)
