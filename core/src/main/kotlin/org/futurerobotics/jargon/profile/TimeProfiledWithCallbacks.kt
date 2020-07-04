package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.util.Stepper
import java.util.concurrent.CompletableFuture
import java.util.concurrent.ConcurrentSkipListSet

//TODO: revise this?
/**
 * A wrapper around another [profiled] object that also provides callbacks via [addCallback].
 *
 * The callbacks will be triggered whenever [atTime] or [timeStepper] is called with a time greater than the
 * callback's time.
 *
 * Callbacks will only be triggered if the time called happens _after_ the callback is added. So add your callbacks
 * first.
 *
 * @see ProfileTimeIndicator
 */
class TimeProfiledWithCallbacks<out T, out P : TimeProfiled<T>>(private val profiled: P) : TimeProfiled<T> {

    private val callbacks = ConcurrentSkipListSet<TimeCallback>()

    private class TimeCallback(val time: Double, val callback: Runnable) : Comparable<TimeCallback> {
        override fun compareTo(other: TimeCallback) = time.compareTo(other.time)
    }

    /**
     * Adds a [callback] that will run when traversed pass the given [timeIndicator].
     *
     * The callback should be short, thread-safe, and not throw exceptions, as it is called when the profiled
     * is traversed.
     *
     * For callbacks that may start other long running tasks, you can:
     * - Use java 8's [CompletableFuture] with [addFuture] (if you are using java 8 only, android api 24+)
     * - Use kotlin coroutines and the `coroutine-extensions` module, and see `addJob` extension function which
     *      returns a job, which a coroutine can then join on;
     * - Add a callback that kick starts another task handled somewhere else.
     */
    fun addCallback(timeIndicator: ProfileTimeIndicator<P>, callback: Runnable): Unit =
        addCallback(timeIndicator.getTime(profiled), callback)

    /**
     * See other [addCallback] doc.
     *
     * Adds a callback at the given [time].
     */
    fun addCallback(time: Double, callback: Runnable) {
        require(time.isFinite()) { "Time ($time) must be finite" }
        callbacks += TimeCallback(time, callback)
    }

    /**
     * Returns a [CompletableFuture] that will complete when traversed pass the given [time].
     *
     * This only works if you are using java 8 (android API level 24+).
     * @see [addCallback]
     */
    fun addFuture(time: ProfileTimeIndicator<P>): CompletableFuture<Void?> =
        addFuture(time.getTime(profiled))

    /**
     * Returns a [CompletableFuture] that will complete when traversed pass the given [time].
     *
     * This only works if you are using java 8 (android API level 24+).
     * @see [addCallback]
     */
    fun addFuture(time: Double): CompletableFuture<Void?> =
        CompletableFuture<Void?>().also {
            addCallback(time, Runnable { it.complete(null) })
        }

    override val duration: Double
        get() = profiled.duration

    override fun atTime(time: Double): T {
        runCallbacks(time)
        return profiled.atTime(time)
    }

    override fun timeStepper(): Stepper<T> {
        val stepper = profiled.timeStepper()
        return Stepper { t ->
            runCallbacks(t)
            stepper.stepTo(t)
        }
    }

    private fun runCallbacks(time: Double) {
        val iterator = callbacks.iterator()
        while (iterator.hasNext()) {
            val callback: TimeCallback = iterator.next()
            if (callback.time <= time) {
                callback.callback.run()
                iterator.remove()
            }
        }
    }
}

/**
 * Creates a [TimeProfiledWithCallbacks] using [this] profiled.
 */
fun <T, P : TimeProfiled<T>> P.withCallbacks(): TimeProfiledWithCallbacks<T, P> =
    TimeProfiledWithCallbacks(this)
