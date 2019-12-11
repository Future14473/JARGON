package org.futurerobotics.jargon.running

import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CompletableJob
import java.util.concurrent.CompletableFuture

/**
 * Represents a unified to signal when something is completed or canceled. Unifies [CompletableFuture] and
 * [CompletableDeferred].
 *
 * @see [CompletableFutureCallback]
 * @see [CompletableJobCallback]
 */
interface CompletionCallback {

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
 * A [CompletionCallback] that completes a [CompletableFuture] with a `null` value when complete.
 */
class CompletableFutureCallback(future: CompletableFuture<Any?>) : CompletionCallback {

    private var future: CompletableFuture<Any?>? = future
    override fun complete() {
        future?.complete(null)
        future = null
    }

    override fun cancel() {
        future?.cancel(false)
        future = null
    }
}

/** Returns a [CompletableFutureCallback] for [this]. */
fun CompletableFuture<Any?>.completionCallback(): CompletableFutureCallback = CompletableFutureCallback(this)

/**
 * A [CompletionCallback] that completes a [CompletableJob].
 *
 * This can complete at most once.
 */
class CompletableJobCallback(job: CompletableJob) : CompletionCallback {

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
