@file:JvmName("StateSpaceCalculations")
@file:Suppress("LocalVariableName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.hipparchus.linear.RiccatiEquationSolverImpl
import kotlin.math.roundToLong

/**
 * Computes the K_ff matrix for plant-inversion feed forward.
 *
 * If costMatrices given is `null`, returns B.pinv()
 *
 * u_ff = (K_ff)(r_(k+1)-A*r).
 */
@JvmOverloads
fun plantInversion(B: Mat, cost: QRCost? = null): Mat {
    if (cost == null) return B.pinv()
    val Q = cost.Q
    val R = cost.R
    return (B.T * Q * B + R).inv() * B.T * Q
}

/**
 * Computes the K_ff matrix for plant-inversion feed forward.
 *
 * If costMatrices given is `null`, returns B.pinv()
 *
 * u_ff = (K_ff)(r_(k+1)-A*r).
 */
@JvmOverloads
fun plantInversion(matrices: DiscreteStateSpaceMatrices, cost: QRCost? = null): Mat =
    plantInversion(matrices.B, cost)

/**
 * Calculates the steady-state kalman-filter error covariance matrix.
 */
@ExperimentalStateSpace
fun steadyStateKalmanFilterCovariance(A: Mat, B: Mat, C: Mat, Q: Mat, R: Mat): Mat {
    val prior = DiscreteRiccatiEquationSolverImpl(A, B, Q, R).p
    val S = C * prior * C.T + R
    val K = prior * C.T * S.inv()
    return (idenMat(A.cols) - K * C) * prior
}

/**
 * Solves the continuous LQR K gain.
 */
fun continuousLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat = RiccatiEquationSolverImpl(A, B, Q, R).k

/**
 * Solves the continuous LQR K gain.
 */
fun continuousLQR(matrices: ContinuousStateSpaceMatrices, qrCost: QRCost): Mat =
    continuousLQR(matrices.A, matrices.B, qrCost.Q, qrCost.R)

/**
 * Solves the discrete LQR K gain (experimental).
 */
@ExperimentalStateSpace
fun discreteLQR(A: Mat, B: Mat, Q: Mat, R: Mat): Mat = DiscreteRiccatiEquationSolverImpl(A, B, Q, R).k

/**
 * Solves the discrete LQR K gain (experimental).
 */
@ExperimentalStateSpace
fun discreteLQR(matrices: DiscreteStateSpaceMatrices, qrCost: QRCost): Mat =
    discreteLQR(matrices.A, matrices.B, qrCost.Q, qrCost.R)

/**
 * Discretizes the given continuous state-space system given by [matrices],
 * using zero-order hold over a given [period].
 */
fun discretizeZeroOrderHold(matrices: ContinuousStateSpaceMatrices, period: Double): DiscreteStateSpaceMatrices =
    with(matrices) {
        val exp = expm(
            concat2x2dynamic(
                A, B,
                0, 0
            ) * period
        )
        val Ad = exp.getQuad(0, 0, A.rows)
        val Bd = exp.getQuad(0, 1, A.rows)
        return DiscreteStateSpaceMatrices(
            Ad,
            Bd,
            C.copy(),
            (period * 1e9).roundToLong()
        )
    }

/**
 * Discretizes a given [cost] in the context of a continuous state-space representation [matrices], using zero-order
 * hold over a given [period].
 */
fun discretizeQRCostZeroOrderHold(matrices: ContinuousStateSpaceMatrices, cost: QRCost, period: Double): QRCost =
    with(matrices) {
        val Qd = expm(
            concat2x2dynamic(
                -A.T, cost.Q,
                0, A
            ) * period
        ).let { m ->
            m.getQuad(1, 1, A.rows) * m.getQuad(0, 1, A.rows)
        }.let { (it + it.T) / 2.0 }
        val Rd = cost.R / period
        return QRCost(Qd, Rd)
    }
