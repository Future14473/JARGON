package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Represents a feed forward. This may produce additional signals that will be added to the controller's signal.
 *
 * Note that the references [FeedForward]s get may first be modified by [StateModifier], if they are added to a
 * [StateSpaceRunnerBuilder] _before_ this element is added.
 */
interface FeedForward {

    /**
     * If this feed forward is to be included in the signal that all other parts of the system
     * sees. If false, this will _only_ be added to the output the external world sees.
     */
    val includeInObserver: Boolean

    /**
     * Given the current [matrices], current reference [r], and expected next reference [r1] (if known, else `null`),
     * gets a feed-forward signal (that will later be added to the actual signal).
     */
    fun getFeedForward(matrices: DiscreteStateSpaceMatrices, r: Vec, r1: Vec?): Vec
}
