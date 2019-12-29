package org.futurerobotics.jargon.running

import kotlinx.coroutines.runBlocking
import java.util.concurrent.Callable

/**
 * A suspend function that _may_ return a value.
 */
interface SuspendFunction<T> {

    /**
     * Returns any action whatsoever, suspending.
     */
    suspend operator fun invoke(): T
}

/**
 * Returns this [SuspendFunction] as a suspend function.
 */
fun <T> SuspendFunction<T>.asFunction(): suspend () -> T = { invoke() }

/**
 * Returns this [SuspendFunction] as a blocking [Runnable].
 */
fun <T> SuspendFunction<T>.asBlocking(): Callable<T> = Callable {
    runBlocking {
        invoke()
    }
}

/**
 * Creates a [SuspendFunction] that runs the given [action].
 */
@Suppress("FunctionName")
inline fun <T> SuspendRunnable(crossinline action: suspend () -> T): SuspendFunction<T> = object :
    SuspendFunction<T> {
    override suspend fun invoke() = action()
}
