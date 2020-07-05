@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

private fun checkMatrices(A: Mat, B: Mat, C: Mat) {

    val numStates: Int = A.rows

    require(A.isSquare) { "A matrix must be square" }
    require(B.rows == numStates)
    { "B matrix rows (${B.rows}) have same number of rows as state vector (${numStates})" }
    require(C.cols == numStates)
    { "C matrix cols  (${B.cols}) must have same number of columns as state vector (${numStates})" }
}

/**
 * State space matrices in a continuous time domain, representing a system where:
 * ```
 * x_dot = Ax + Bu
 * y = Cx
 * ```
 *
 * D matrix is currently not supported.
 */
data class ContinuousStateSpaceMatrices(
    val A: Mat, val B: Mat, val C: Mat
) {

    init {
        checkMatrices(A, B, C)
    }

    /** The number of states. */
    val numStates: Int = A.rows

    /** The number of inputs. */
    val numInputs: Int = B.cols

    /** The number of outputs. */
    val numOutputs: Int = C.rows

    /** Discretizes this state-space system using zero-order hold over a given [period]. */
    fun discretizeZeroOrderHold(period: Double): DiscreteStateSpaceMatrices = discretizeZeroOrderHold(this, period)

    /**
     * Discretizes a given [cost] in the context of these state space matrices, using zero-order hold over a
     * given [period].
     */
    fun discretizeQRCostZeroOrderHold(cost: QRCost, period: Double): QRCost =
        discretizeQRCostZeroOrderHold(this, cost, period)
}

/**
 * State space matrices in a discrete time domain, representing a system where:
 * ```
 * x_next = Ax + Bu
 * y = Cx
 * ```
 *
 * @param periodNanos the period of discretization, in nanoseconds
 */
data class DiscreteStateSpaceMatrices(
    val A: Mat, val B: Mat, val C: Mat, val periodNanos: Long
) {

    init {
        checkMatrices(A, B, C)
    }

    /** The number of states. */
    val numStates: Int = A.rows

    /** The number of inputs. */
    val numInputs: Int = B.cols

    /** The number of outputs. */
    val numOutputs: Int = C.rows

    /** The period of discretization, in seconds. */
    val periodSeconds: Double get() = periodNanos / 1e9
}

/**
 * Cost matrices Q and R for _state_ and _control effort_ respectively, used to in LQR and in plant inversion
 * (optionally).
 *
 * Both must be symmetric matrices.
 */
data class QRCost(
    /** Q, state cost matrix. */
    val Q: Mat,
    /** R, control effort cost matrix. */
    val R: Mat
) {

    init {
        require(Q.isSymmetric()) { "Q matrix must be symmetric" }
        require(R.isSymmetric()) { "R matrix must be symmetric" }
    }
}

/**
 * Noise covariance matrices Q and R for state and measurement noise, respectively.
 */
data class NoiseCovariance(
    /** Q, State noise covariance. */
    val Q: Mat,
    /** R, Measurement noise covariance. */
    val R: Mat
) {

    init {
        require(Q.isSymmetric()) { "Q matrix must be symmetric" }
        require(R.isSymmetric()) { "R matrix must be symmetric" }
    }
}
