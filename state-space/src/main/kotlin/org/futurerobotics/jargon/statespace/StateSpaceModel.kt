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

/** Interface to indicate that a [LinearStateSpaceModel] is continuous. */
interface ContinuousLinearStateSpaceModel : LinearStateSpaceModel {

    /** Discretizes this [ContinuousLinearStateSpaceModel] using the given [period]. */
    fun discretize(period: Double): DiscreteLinearStateSpaceModel
}

/** Interface to indicate that a [LinearStateSpaceModel] is discretized with a certain [period] */
interface DiscreteLinearStateSpaceModel : LinearStateSpaceModel {

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
 * @param copyMat if true, matrices will be copied, else no.
 */
abstract class AbstractLinearStateSpaceModel @JvmOverloads constructor(
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    copyMat: Boolean = true
) : LinearStateSpaceModel {

    /** The size of the state vector. */
    final override val stateSize: Int
    /** The size of the input vector. */
    final override val inputSize: Int
    /** The size of the output vector. */
    final override val outputSize: Int
    final override val A: Mat
    final override val B: Mat
    final override val C: Mat
    final override val D: Mat

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
        fun Mat.maybeCopy() = if (copyMat) copy() else this
        this.A = A.maybeCopy()
        this.B = B.maybeCopy()
        this.C = C.maybeCopy()
        this.D = D.maybeCopy()
    }

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
 * Implementation of [ContinuousLinearStateSpaceModel].
 */
open class ContinuousLinSSModelImpl @JvmOverloads constructor(A: Mat, B: Mat, C: Mat, D: Mat, copyMat: Boolean = true) :
    AbstractLinearStateSpaceModel(A, B, C, D, copyMat), ContinuousLinearStateSpaceModel {

    /**
     * Discretizes the current model, using the given [period], as well as discretizing the given [qrCost]
     */
    fun discretize(period: Double, qrCost: QRCost): Pair<DiscreteLinearStateSpaceModel, QRCost> {
        require(qrCost applicableTo this) { "Cost matrices must be applicable to this model" }
        val model = discretize(period)

        val q = expm(MatConcat.dynamic2x2Square(-A.T, qrCost.Q, 0, A) * period).let {
            it.getQuad(A.rows, 1, 1) * it.getQuad(A.rows, 0, 1)
        }
        val r = qrCost.R / period
        return model to QRCost(q, r)
    }

    override fun discretize(period: Double): DiscreteLinSSModelImpl {
        val (ad, bd) = expm(MatConcat.dynamic2x2Square(A, B, 0, 0) * period).let {
            it.getQuad(A.rows, 0, 0) to it.getQuad(A.rows, 0, 1)
        }
        return DiscreteLinSSModelImpl(period, ad, bd, C, D)
    }
}

/**
 * Implementation of [DiscreteLinearStateSpaceModel].
 */
open class DiscreteLinSSModelImpl @JvmOverloads constructor(
    override val period: Double, A: Mat, B: Mat, C: Mat, D: Mat,
    copyMat: Boolean = true
) : AbstractLinearStateSpaceModel(A, B, C, D, copyMat), DiscreteLinearStateSpaceModel

