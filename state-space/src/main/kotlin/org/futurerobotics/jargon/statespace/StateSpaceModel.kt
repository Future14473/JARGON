@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Generic state space model
 * @see LinearStateSpaceModel
 */
//one day, perhaps: linear_ized_ models
interface StateSpaceModel {

    /** The size of the state vector. */
    val stateSize: Int
    /** The size of the input vector. */
    val inputSize: Int
    /** The size of the output vector. */
    val outputSize: Int

    /**
     * Gets either x-dot or x_k+1 given the current state vector [x] and signal vector [u]
     */
    fun processState(x: Vec, u: Vec): Vec

    /**
     * Gets the output/measurement vector given the current state vector [x] and signal vector [u]
     */
    fun processOutput(x: Vec, u: Vec): Vec
}

/**
 * A linear state space model, where the A, B, C, D matrices are fixed.
 */
interface LinearStateSpaceModel : StateSpaceModel {

    /** A, the system matrix; dimension (states x states), indicates how the state changes over time*/
    val A: Mat
    /** B, the control matrix; dimension (state x input), indicates how input changes state*/
    val B: Mat
    /** C, the measurement matrix; dimension (output x state), indicates how measurement is read from state*/
    val C: Mat
    /** D, the feed-through matrix; dimension (*/
    val D: Mat
}

/** Interface to indicate that a model is continuous. */
interface ContinuousStateSpaceModel : StateSpaceModel {

    /** Discretizes this [ContinuousStateSpaceModel] using the given [period]. */
    fun discretize(period: Double): DiscreteStateSpaceModel
}

/** Interface to indicate that a model is discretized with a certain [period] */
interface DiscreteStateSpaceModel : StateSpaceModel {

    /** The period to which this model has been discretized. Should be >0. */
    val period: Double
}

/**
 * Skeletal implementation of a [LinearStateSpaceModel].
 *
 * @param A the system matrix
 * @param B the control matrix
 * @param C the measurement matrix
 * @param D the feed-through matrix
 */
sealed class AbstractLinearStateSpaceModel(A: Mat, B: Mat, C: Mat, D: Mat) : LinearStateSpaceModel {

    /** The size of the state vector. */
    final override val stateSize: Int
    /** The size of the input vector. */
    final override val inputSize: Int
    /** The size of the output vector. */
    final override val outputSize: Int

    init {
        require(A.isSquare) { "A matrix must be square" }
        stateSize = A.rows
        inputSize = B.cols
        outputSize = C.rows
        require(B.rows == stateSize)
        { "B matrix must have same number of rows as the state vector (${stateSize}.size)" }
        require(C.cols == stateSize)
        { "C matrix must have same number of columns as the state vector (${stateSize})" }
        require(D.cols == inputSize)
        { "D matrix must have same number of columns as the input vector ($inputSize)" }
        require(D.rows == outputSize)
        { "C and D matrix rows must match to deduce the output vector size" }
    }

    override val A: Mat = A.toImmutableMat()
    override val B: Mat = B.toImmutableMat()
    override val C: Mat = C.toImmutableMat()
    override val D: Mat = D.toImmutableMat()

    override fun processState(x: Vec, u: Vec): Vec = A * x + B * u

    override fun processOutput(x: Vec, u: Vec): Vec = C * x + D * u

    override fun toString(): String =
        """
${this.javaClass.simpleName}
A: 
${A.formatReadable()}
B: 
${B.formatReadable()}
C:
${C.formatReadable()}
D: 
${D.formatReadable()}
"""
}

/**
 * A [LinearStateSpaceModel] that is continuous.
 */
open class ContinuousLinSSModel constructor(A: Mat, B: Mat, C: Mat, D: Mat) :
    AbstractLinearStateSpaceModel(A, B, C, D), ContinuousStateSpaceModel {

    fun discretize(period: Double, QRCost: QRCost): Pair<DiscreteLinSSModel, QRCost> {
        require(QRCost applicableTo this) { "Cost matrices must be applicable to this model" }
        val model = discretize(period)

        val q = expm(MatConcat.dynamic2x2Square(-A.T, QRCost.Q, 0, A) * period).let {
            it.getQuad(A.rows, 1, 1) * it.getQuad(A.rows, 0, 1)
        }
        val r = QRCost.R / period
        return model to QRCost(q, r)
    }

    override fun discretize(period: Double): DiscreteLinSSModel {
        val (ad, bd) = expm(MatConcat.dynamic2x2Square(A, B, 0, 0) * period).let {
            it.getQuad(A.rows, 0, 0) to it.getQuad(A.rows, 0, 1)
        }
        return DiscreteLinSSModel(ad, bd, C, D, period)
    }
}

/**
 * A [LinearStateSpaceModel] that is discrete.
 *
 * @param period the period the discretization is based on.
 */
open class DiscreteLinSSModel(A: Mat, B: Mat, C: Mat, D: Mat, override val period: Double) :
    AbstractLinearStateSpaceModel(A, B, C, D), DiscreteStateSpaceModel

