package org.futurerobotics.jargon.control

/**
 * Represents a simple signal that takes a [Reference] and a [State] to produce a [Signal].
 *
 * When first created or [reset]ed, the [signal] should return a "zero" value.
 */
interface SimpleController<in Reference, in State, out Signal> {

    /**
     * Resets the controller.
     *
     * The [signal] should now represent some "zero" value.
     */
    fun reset()

    /**
     * Gets the last outputted signal of this controller.
     *
     * If just [reset]ed or just created, the signal should be a "zero" value.
     */
    val signal: Signal

    /**
     * Given the [reference], [currentState], and [elapsedTimeInNanos], updates and returns the current signal.
     *
     * This should also update [signal].
     */
    fun update(reference: Reference, currentState: State, elapsedTimeInNanos: Long): Signal
}
