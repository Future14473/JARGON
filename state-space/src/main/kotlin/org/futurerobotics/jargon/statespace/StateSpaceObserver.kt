package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * An observer for a [StateSpaceRunner], which estimates from signals and measurements.
 */
interface StateSpaceObserver {

    /**
     * Resets this filter completely, with an [initialState].
     *
     * @see currentState
     */
    fun reset(initialState: Vec)

    /**
     * Given the current (possibly linearized) [matrices], signal [u], measurement [y], and [timeInNanos], updates the
     * filter and returns a state estimate.
     *
     * [reset] must have been called at least once.
     */
    fun update(matrices: DiscreteStateSpaceMatrices, u: Vec, y: Vec, timeInNanos: Long): Vec

    /**
     * Gets the current estimated state,
     *
     * or set to manually override the current state. This should also have the s
     *
     * [reset] must have been called at least once.
     */
    var currentState: Vec
}
