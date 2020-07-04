package org.futurerobotics.jargon.control

/**
 * Represents a simple controller: takes some kind of [Reference] (target position) and a [State] (measured position) to
 * produce a [Signal].
 *
 * When first created or [reset]ed, the [signal] should return a "zero" value.
 */
interface SimpleController<in Reference, in State, out Signal> {

    /**
     * Resets the controller.
     *
     * The [signal] should now represent some zero value.
     */
    @Deprecated("State bad", level = DeprecationLevel.WARNING)
    fun reset()

    /**
     * Gets the last outputted signal of this controller.
     *
     * If just [reset]ed or just created, the signal should be a "zero" value.
     */
    val signal: Signal

    /**
     * Given the [reference], [currentState], and [elapsedTimeInNanos], updates the controller and returns the
     * current signal, which should be the current [signal].
     */
    fun update(reference: Reference, currentState: State, elapsedTimeInNanos: Long): Signal
}
