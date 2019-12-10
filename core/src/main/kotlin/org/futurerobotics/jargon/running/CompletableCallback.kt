package org.futurerobotics.jargon.running

import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CompletableJob
import java.util.concurrent.CompletableFuture

/**
 * Represents a callback to signal when something is completed or canceled.. Unifies [CompletableFuture] and
 * [CompletableDeferred].
 */
interface CompletableCallback {

    /**
     * Runs a callback when a task is complete.
     */
    fun complete()

    /**
     * Runs a callback when a task is canceled.
     */
    fun cancel()
}

/**
 * A [CompletableCallback] that completes a [CompletableFuture] with the given [completeValue].
 */
class CompletableFutureCallback<T>(future: CompletableFuture<T>, completeValue: T) : CompletableCallback {

    private var completeValue: T? = completeValue
    private var future: CompletableFuture<T>? = future
    override fun complete() {
        future?.complete(completeValue)
        future = null
        completeValue = null
    }

    override fun cancel() {
        future?.cancel(false)
        future = null
        completeValue = null
    }
}

/** Returns a [CompletableFutureCallback] for [this]. */
fun <T> CompletableFuture<T>.completionCallback(completeValue: T): CompletableFutureCallback<T> =
    CompletableFutureCallback(this, completeValue)

/**
 * A [CompletableCallback] that completes a [CompletableJob].
 *
 * This can complete at most once.
 */
class CompletableJobCallback(job: CompletableJob) : CompletableCallback {

    private var job: CompletableJob? = job
    override fun complete() {
        job?.complete()
        job = null
    }

    override fun cancel() {
        job?.cancel()
        job = null
    }
}

/** Returns a [CompletableJob] for [this]. */
fun CompletableJob.completionCallback(): CompletableJobCallback =
    CompletableJobCallback(this)

/**
 * A [CompletableCallback] that completes a [CompletableDeferred] with the given [completeValue].
 *
 * This can complete at most once.
 */
class CompletableDeferredCallback<T>(deferred: CompletableDeferred<T>, completeValue: T) : CompletableCallback {

    private var completeValue: T? = completeValue
    private var deferred: CompletableDeferred<T>? = deferred
    override fun complete() {
        completeValue?.let { value ->
            deferred?.complete(value)
        }
        completeValue = null
        deferred = null
    }

    override fun cancel() {
        deferred?.cancel()
        completeValue = null
        deferred = null
    }
}

/** Returns a [CompletableJob] for [this]. */
fun <T> CompletableDeferred<T>.completionCallback(completeValue: T): CompletableDeferredCallback<T> =
    CompletableDeferredCallback(this, completeValue)
