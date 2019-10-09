@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*


/**
 * Generic Q and R cost matrices for state deviation and control effort, respectively
 */
class QRCost(Q: Mat, R: Mat) {
    init {
        require(Q.isSquare) { "Q must be square" }
        require(R.isSquare) { "R must be square" }
    }

    /** State cost matrix Q */
    val Q: Mat = Q.toImmutableMat()
    /** Control cost matrix Q */
    val R: Mat = R.toImmutableMat()

    /** Q matrix; state cost */
    operator fun component1(): Mat = Q

    /** R matrix; control effort cost */
    operator fun component2(): Mat = R

    /**
     * Returns if the dimensions of this QRCost is applicable to the given [model]
     */
    infix fun applicableTo(model: LinearStateSpaceModel): Boolean {
        return Q.isSquare && Q.rows == model.stateSize && R.isSquare &&
                R.rows == model.inputSize
    }
}