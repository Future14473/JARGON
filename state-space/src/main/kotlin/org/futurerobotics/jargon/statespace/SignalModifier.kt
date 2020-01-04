package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * This may _adjust_ additional signals before it becomes the actual signal.
 *
 * This will be applied AFTER [FeedForward]s; but if this [SignalModifier] does not [includeInObserver]
 * it will not see feed-forwards [includeInObserver].
 */
interface SignalModifier {

    /**
     * If the modifications is to be included in the signal that all other parts of the system
     * sees. If false, this will _only_ be added to the output the external world sees.
     */
    val includeInObserver: Boolean

    /**
     * Given the current [matrices], current state [x], current signal (with appropriate feed-forwards).
     */
    fun modifySignal(matrices: DiscreteStateSpaceMatrices, x: Vec, u: Vec, y: Vec): Vec
}
