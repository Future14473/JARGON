package org.futurerobotics.jargon.control

/**
 * Represents a simple controller: takes some kind of [Reference] (target position) and a [State] (measured position) to
 * produce a [Signal].
 *
 * When first created or [reset]ed, the [signal] should return a "zero" value.
 */
interface SimpleController<in Reference, in State, out Signal> {

    /**
     * Gets the last outputted signal of this controller.
     */
    val signal: Signal

    /**
     * Given the [reference], [currentState], and [elapsedTimeInNanos], updates the controller and returns the
     * current signal, which should be the current [signal].
     */
    fun update(reference: Reference, currentState: State, elapsedTimeInNanos: Long): Signal
}
