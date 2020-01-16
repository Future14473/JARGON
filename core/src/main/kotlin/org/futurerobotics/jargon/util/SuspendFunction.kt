package org.futurerobotics.jargon.util

/**
 * A suspend function that can return a value.
 */
interface SuspendFunction<out T> {

    /**
     * Calls the function, and _may_ return a value.
     */
    suspend operator fun invoke(): T
}
