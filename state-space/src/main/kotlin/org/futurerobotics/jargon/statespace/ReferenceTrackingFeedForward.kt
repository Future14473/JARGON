@file:Suppress("LocalVariableName", "PrivatePropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

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

    /**
     * Gets the current `Kff` matrix, given the current [matrices], reference [r], and next reference [r1] if
     * known.
     */
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

/**
 * A [FeedForward] based on a plant inversion, _recomputed every cycle_.
 *
 * @see plantInversion
 * @see ReferenceTrackingFeedForward
 */
class RepeatedPlantInversion(private val qrCost: QRCost? = null) : FeedForward {

    override val includeInObserver: Boolean
        get() = true

    override fun getFeedForward(matrices: DiscreteStateSpaceMatrices, r: Vec, r1: Vec?): Vec =
        if (r1 == null) {
            zeroVec(matrices.numInputs)
        } else {
            val Kff = plantInversion(matrices.B, qrCost)
            Kff(r1 - matrices.A * r)
        }
}
