/**
 * Generic state space calculation utils.
 */
@file:JvmName("StateSpaceCalculations")
@file:Suppress("LocalVariableName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.hipparchus.linear.RiccatiEquationSolverImpl


/**
 * Computes the K_ff matrix for plant-inversion feed forward.
 *
 * If costMatrices given is `null`, returns B.pinv()
 *
 * u_ff = (K_ff)(r_(k+1)-A*r).
 */
fun plantInversionKFF(stateSpaceModel: DiscreteSSModel, QRCost: QRCost? = null): Mat {
    val B = stateSpaceModel.B
    if (QRCost == null) return B.pinv()
    require(QRCost applicableTo stateSpaceModel)
    val (Q, R) = QRCost
    return (B.T * Q * B + R).inv() * B.T * Q
}

fun steadyStateKalmanErrorCov(model: DiscreteSSModel, Q: Mat, R: Mat): Mat {
    val (A, B, C) = model
    return steadyStateKalmanErrorCov(A, B, C, Q, R)
}

fun steadyStateKalmanErrorCov(A: Mat, B: Mat, C: Mat, Q: Mat, R: Mat): Mat {
    val Pprior = DiscreteRicattiEquationSolverImpl(A, B, Q, R).p
    val S = C * Pprior * C.T + R
    val K = Pprior * C.T * S.inv()
    return (pureEye(A.cols) - K * C) * Pprior
}


fun continuousLQR(model: StateSpaceModel, QRCost: QRCost): Mat {
    return continuousLQR(model.A, model.B, QRCost.Q, QRCost.R)
}

fun continuousLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat {
    return RiccatiEquationSolverImpl(A, B, Q, R).k
}

fun discreteLQR(model: StateSpaceModel, QRCost: QRCost): Mat {
    return discreteLQR(model.A, model.B, QRCost.Q, QRCost.R)
}

fun discreteLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat {
    return DiscreteRicattiEquationSolverImpl(A, B, Q, R).k
}

