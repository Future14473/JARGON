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
 * Returns this [SuspendRunnable] as a suspend function.
 */
fun SuspendRunnable.asFunction(): suspend () -> Unit = { runSuspend() }

/**
 * Returns this [SuspendRunnable] as a blocking [Runnable].
 */
fun SuspendRunnable.asBlocking(): Runnable = Runnable {
    runBlocking {
        runSuspend()
    }
}

/**
 * Creates a [SuspendRunnable] that runs the given [action].
 */
@Suppress("FunctionName")
inline fun SuspendRunnable(crossinline action: suspend () -> Unit): SuspendRunnable = object :
    SuspendRunnable {
    override suspend fun runSuspend() = action()
}
