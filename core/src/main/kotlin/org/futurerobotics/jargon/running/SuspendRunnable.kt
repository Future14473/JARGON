package org.futurerobotics.jargon.running

import kotlinx.coroutines.runBlocking

/**
 * Like a [Runnable], but runs a suspend function.
 */
interface SuspendRunnable {

    /**
     * Runs any action whatsoever, suspending.
     */
    suspend fun runSuspend()
}

/**
 * Returns this [SuspendRunnable] as a function.
 */
val SuspendRunnable.asFunc: suspend () -> Unit get() = { runSuspend() }

/**
 * Returns this [SuspendRunnable] as a blocking [Runnable].
 */
val SuspendRunnable.asBlocking: Runnable
    get() = Runnable {
        runBlocking {
            runSuspend()
        }
    }

/**
 * Creates a [SuspendRunnable] that runs the given [action].
 */
inline fun SuspendRunnable(crossinline action: suspend () -> Unit): SuspendRunnable = object : SuspendRunnable {
    override suspend fun runSuspend() {
        action()
    }
}
