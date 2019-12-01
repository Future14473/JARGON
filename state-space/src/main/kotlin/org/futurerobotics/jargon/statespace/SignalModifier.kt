package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Represents a feed forward. This may _adjust_ additional signals that will be added to the controller's signal.
 *
 * Note that the references this get may first be modified by [StateModifier], if they are added to a
 * [StateSpaceRunnerBuilder] _before_ this element is added.
 */
interface SignalModifier {

    /**
     * If the modifications is to be included in the signal that all other parts of the system
     * sees. If false, this will _only_ be added to the output the external world sees.
     */
    val includeInObserver: Boolean

    /**
     * Given the current [matrices], current reference [r], and expected next reference [r1] (if known, else `null`),
     * and signal [u], gets an adjusted signal.
     *
     * This will include all [StateModifier]s!!
     */
    fun modifySignal(matrices: DiscreteStateSpaceMatrices, x: Vec, u: Vec, y: Vec): Vec
}
