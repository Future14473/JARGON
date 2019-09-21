package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*


/**
 * Generic Q and R cost matrices for state deviation and control effort, respectively
 */
//todo: namings.
class QRCost(Q: Mat, R: Mat) {
    init {
        require(Q.isSquare) { "Q must be square" }
        require(Q.isSquare) { "R must be square" }
    }

    /** State cost matrix Q */
    val Q: Mat = Q.copy()
    /** Control cost matrix Q */
    val R: Mat = R.copy()
    operator fun component1() = Q
    operator fun component2() = R

    /**
     * Returns if the dimensions of this QRCost is applicable to the given [model]
     */
    infix fun applicableTo(model: StateSpaceModel): Boolean {
        return Q.isSquare && Q.rows == model.stateSize && R.isSquare &&
                R.rows == model.inputSize
    }
}