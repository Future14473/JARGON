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
fun plantInversionKFF(stateSpaceModel: DiscreteLinSSModel, cost: QRCost? = null): Mat {
    val B = stateSpaceModel.B
    if (cost == null) return B.pinv()
    require(cost applicableTo stateSpaceModel)
    val (Q, R) = cost
    return (B.T * Q * B + R).inv() * B.T * Q
}

/**
 * Calculates the stead state kalman filter error.
 */
fun steadyStateKalmanErrorCov(model: DiscreteLinSSModel, Q: Mat, R: Mat): Mat {
    val (A, B, C) = model
    return steadyStateKalmanErrorCov(A, B, C, Q, R)
}

/**
 * Calculates the stead state kalman filter error.
 */
fun steadyStateKalmanErrorCov(A: Mat, B: Mat, C: Mat, Q: Mat, R: Mat): Mat {
    val prior = DiscreteRicattiEquationSolverImpl(A, B, Q, R).p
    val S = C * prior * C.T + R
    val K = prior * C.T * S.inv()
    return (idenMat(A.cols) - K * C) * prior
}


/**
 * Solves the continuous LQR K gain
 */
fun continuousLQR(model: LinearStateSpaceModel, cost: QRCost): Mat = continuousLQR(model.A, model.B, cost.Q, cost.R)

/**
 * Solves the discrete LQR K gain
 */
fun continuousLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat = RiccatiEquationSolverImpl(A, B, Q, R).k

/**
 * Solves the discrete LQR K gain
 */
fun discreteLQR(model: DiscreteLinSSModel, cost: QRCost): Mat = discreteLQR(model.A, model.B, cost.Q, cost.R)

/**
 * Solves the discrete LQR K gain
 */
fun discreteLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat = DiscreteRicattiEquationSolverImpl(A, B, Q, R).k

/**
 * Solves the LQR gain, either continuous or discrete based on the model.
 */
fun anyLQR(model: LinearStateSpaceModel, cost: QRCost): Mat = when (model) {
    is ContinuousLinSSModel -> continuousLQR(model, cost)
    is DiscreteLinSSModel -> discreteLQR(model, cost)
}

