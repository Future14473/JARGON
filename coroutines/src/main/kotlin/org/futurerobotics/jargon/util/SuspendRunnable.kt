package org.futurerobotics.jargon.util

/**
 * Like a [Runnable], but runs a [suspend] function.
 */
interface SuspendRunnable {

    /**
     * Runs, any action whatsoever.
     */
    suspend fun run()
}

/**
 * Returns this [SuspendRunnable] as a [suspend] function.
 */
val SuspendRunnable.func: suspend () -> Unit get() = suspend { run() }
