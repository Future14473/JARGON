@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Common superclass of [ContinuousStateSpaceMatrices] and [DiscreteStateSpaceMatrices]. Contains
 * [A], [B], and [C] matrices.
 *
 * A `D` matrix is currently not yet supported (and is rarely needed).
 */
abstract class StateSpaceMatrices(
    /** System matrix: states by states */
    val A: Mat,
    /** Control matrix: states by inputs */
    val B: Mat,
    /** Measurement matrix: outputs by states */
    val C: Mat
) {

    /** The number of states. */
    val numStates: Int = A.rows
    /** The number of inputs. */
    val numInputs: Int = B.cols
    /** The number of outputs. */
    val numOutputs: Int = C.rows

    init {
        require(A.isSquare) { "A matrix must be square" }
        require(B.rows == numStates)
        { "B matrix rows (${B.rows}) have same number of rows as state vector (${numStates})" }
        require(C.cols == numStates)
        { "C matrix cols  (${B.cols}) must have same number of columns as state vector (${numStates})" }
    }

    /**
     * Returns either the next state or the state's derivative, modeled by these matrices,
     * given the currents state [x] and signal [u].
     */
    fun getStateEvolution(x: Vec, u: Vec): Vec = A * x + B * u

    /** [A] */
    operator fun component1(): Mat = A

    /** [B] */
    operator fun component2(): Mat = B

    /** [C] */
    operator fun component3(): Mat = C

    override fun toString(): String {
        return """${javaClass.simpleName}:
A
$A
B
$B
C
$C
"""
    }
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
class ContinuousStateSpaceMatrices(
    A: Mat, B: Mat, C: Mat
) : StateSpaceMatrices(A, B, C) {

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
 * @param periodInNanos the period of discretization, in nanoseconds
 */
class DiscreteStateSpaceMatrices(
    A: Mat, B: Mat, C: Mat, val periodInNanos: Long
) : StateSpaceMatrices(A, B, C) {

    /** The period of discretization, in seconds. */
    val period: Double get() = periodInNanos / 1e9
}

/**
 * Cost matrices Q and R for _state_ and _control effort_ respectively, used to in LQR and in plant inversion
 * (optionally).
 *
 * Both must be symmetric matrices.
 */
class QRCost(
    /** Q, state cost matrix. */
    val Q: Mat,
    /** R, control effort cost matrix. */
    val R: Mat
) {

    init {
        require(Q.isSymmetric()) { "Q matrix must be symmetric" }
        require(R.isSymmetric()) { "R matrix must be symmetric" }
    }

    /** [Q] */
    operator fun component1(): Mat = Q

    /** [R] */
    operator fun component2(): Mat = R
}

/**
 * Noise covariance matrices Q and R for state and measurement noise, respectively.
 */
class NoiseCovariance(
    /** Q, State noise covariance. */
    val Q: Mat,
    /** R, Measurement noise covariance. */
    val R: Mat
) {


    init {
        require(Q.isSymmetric()) { "Q matrix must be symmetric" }
        require(R.isSymmetric()) { "R matrix must be symmetric" }
    }

    /** [Q] */
    operator fun component1(): Mat = Q

    /** [R] */
    operator fun component2(): Mat = R
}
