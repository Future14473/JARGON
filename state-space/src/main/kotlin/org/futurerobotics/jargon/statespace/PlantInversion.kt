@file:Suppress("LocalVariableName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

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
