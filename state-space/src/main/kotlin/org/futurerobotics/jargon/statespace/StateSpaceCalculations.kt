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
@JvmOverloads
fun plantInversionKFF(stateSpaceModel: DiscreteLinearStateSpaceModel, cost: QRCost? = null): Mat {
    val B = stateSpaceModel.B
    if (cost == null) return B.pinv()
    require(cost applicableTo stateSpaceModel)
    val (Q, R) = cost
    return (B.T * Q * B + R).inv() * B.T * Q
}

/**
 * Calculates the stead state kalman filter error.
 */
fun steadyStateKalmanErrorCov(model: DiscreteLinearStateSpaceModel, Q: Mat, R: Mat): Mat =
    steadyStateKalmanErrorCov(model.A, model.B, model.C, Q, R)

/**
 * Calculates the steady state kalman filter error matrix.
 */
fun steadyStateKalmanErrorCov(A: Mat, B: Mat, C: Mat, Q: Mat, R: Mat): Mat {
    val prior = DiscreteRicattiEquationSolverImpl(A, B, Q, R).p
    val S = C * prior * C.T + R
    val K = prior * C.T * S.inv()
    return (idenMat(A.cols) - K * C) * prior
}

/**
 * Solves the continuous LQR K gain.
 */
fun continuousLQR(model: LinearStateSpaceModel, cost: QRCost): Mat = continuousLQR(model.A, model.B, cost.Q, cost.R)

/**
 * Solves the continuous LQR K gain
 */
fun continuousLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat = RiccatiEquationSolverImpl(A, B, Q, R).k

/**
 * Solves the discrete LQR K gain. Experimental
 */
fun discreteLQR(model: LinearStateSpaceModel, cost: QRCost): Mat = discreteLQR(model.A, model.B, cost.Q, cost.R)

/**
 * Solves the discrete LQR K gain. Experimental.
 */
fun discreteLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat = DiscreteRicattiEquationSolverImpl(A, B, Q, R).k

/**
 * Solves the LQR gain, either continuous or discrete based on the model, using the _current_ states.
 */
fun anyLQR(model: LinearStateSpaceModel, cost: QRCost): Mat = when (model) {
    is ContinuousLinearStateSpaceModel -> continuousLQR(model, cost)
    is DiscreteLinearStateSpaceModel -> discreteLQR(model, cost)
    else -> throw IllegalArgumentException("Unknown if the model is continuous or discrete")
}

