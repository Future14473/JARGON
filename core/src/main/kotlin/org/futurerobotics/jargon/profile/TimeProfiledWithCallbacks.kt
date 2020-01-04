package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.util.Stepper
import java.util.concurrent.CompletableFuture
import java.util.concurrent.ConcurrentSkipListSet
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

    private val callbacks = ConcurrentSkipListSet<TimeCallback>()

    private class TimeCallback(val time: Double, val callback: Runnable) : Comparable<TimeCallback> {
        override fun compareTo(other: TimeCallback) = time.compareTo(other.time)
    }

    /**
     * Adds a [callback] that will run when traversed pass the given [time].
     *
     * The callback should be a short, thread-safe, and not throw exceptions, as it is called when the profiled
     * is traversed.
     */
    fun addCallback(time: ProfileTimeIndicator<P>, callback: Runnable) = addCallback(time.getTime(profiled), callback)

    /**
     * Adds a [callback] that will run when traversed pass the given [time].
     *
     * The callback should be a short, thread-safe, and not throw exceptions, as it is called when the profiled
     * is traversed.
     */
    fun addCallback(time: Double, callback: Runnable) {
        require(time.isFinite()) { "Time ($time) must be finite" }
        callbacks += TimeCallback(time, callback)
    }

    /**
     * Returns a future that will complete when traversed pass the given [time].
     *
     * If running on android, __this can only be used with android API level 24+__.
     */
    fun addFuture(time: Double): CompletableFuture<*> = CompletableFuture<Nothing?>().also {
        addCallback(time, Runnable { it.complete(null) })
    }

    /**
     * Returns a future that will complete when traversed pass the given [time].
     *
     * If running on android, __this can only be used with android API level 24+__.
     */
    fun addFuture(time: ProfileTimeIndicator<P>): CompletableFuture<*> = addFuture(time.getTime(profiled))

    override val duration: Double
        get() = profiled.duration

    override fun atTime(time: Double): T {
        runCallbacks(time)
        return profiled.atTime(time)
    }

    override fun stepper(): Stepper<T> {
        val stepper = profiled.stepper()
        return Stepper { t ->
            runCallbacks(t)
            stepper.stepTo(t)
        }
    }

    private tailrec fun runCallbacks(time: Double) {
        val callback = callbacks.pollFirst() ?: return
        if (callback.time <= time) {
            callback.callback.run()
            runCallbacks(time)
        }
    }
}

/**
 * Creates a [TimeProfiledWithCallbacks] using [this] profiled.
 */
fun <T : Any, P : TimeProfiled<T>> P.withCallbacks(): TimeProfiledWithCallbacks<T, P> =
    TimeProfiledWithCallbacks(this)
