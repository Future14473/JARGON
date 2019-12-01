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
val SuspendRunnable.asFunc: suspend () -> Unit get() = suspend { runSuspend() }

/**
 * Returns this [SuspendRunnable] as a blocking [Runnable].
 */
val SuspendRunnable.asBlocking: Runnable
    get() = Runnable {
        runBlocking {
            runSuspend()
        }
    }
