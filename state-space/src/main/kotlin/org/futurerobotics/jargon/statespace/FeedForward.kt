@file:Suppress("LocalVariableName", "PrivatePropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Represents a feed forward. This may produce additional signals that will be added to the controller's signal.
 *
 * Note that the references [FeedForward]s get may first be modified by [StateDecorator], if they are added to a
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

/**
 * A feed forward for reference tracking, `uff = Kff(r1 - A*r)`, where Kff may change depending on time/input
 * via [getKff].
 *
 * @see ReferenceTrackingFeedForward
 */
abstract class VaryingReferenceTrackingFeedForward : FeedForward {

    final override fun getFeedForward(matrices: DiscreteStateSpaceMatrices, r: Vec, r1: Vec?): Vec {
        val Kff = getKff(matrices, r, r1)
        return if (r1 != null) {
            Kff(r1 - matrices.A * r)
        } else zeroVec(Kff.rows)
    }

    abstract fun getKff(matrices: DiscreteStateSpaceMatrices, r: Vec, r1: Vec?): Mat
}

/**
 * A feed forward for reference tracking, `uff = Kff(r1 - A*r)`, where Kff is a constant matrix.
 */
class ReferenceTrackingFeedForward(Kff: Mat, copyMat: Boolean = true) : VaryingReferenceTrackingFeedForward() {

    private val Kff: Mat = Kff.copyIf(copyMat)
    override val includeInObserver: Boolean
        get() = true

    override fun getKff(matrices: DiscreteStateSpaceMatrices, r: Vec, r1: Vec?): Mat = Kff
}
