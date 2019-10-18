@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.VectorStructure
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.util.repeatedList


/**
 * Generic State space model.
 *
 * @param A the system matrix
 * @param B the control matrix
 * @param C the measurement matrix
 * @param D the feed-through matrix, which is usually 0 so nobody cares about it very much
 */
sealed class LinearStateSpaceModel constructor(
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    stateStructure: VectorStructure? = null,
    inputStructure: VectorStructure? = null,
    outputStructure: VectorStructure? = null
) {
    /** The state [VectorStructure]. */
    val stateStructure: VectorStructure
    /** The input [VectorStructure] */
    val inputStructure: VectorStructure
    /** the output [VectorStructure] */
    val outputStructure: VectorStructure

    init {
        require(A.isSquare) { "A matrix must be square" }
        this.stateStructure = stateStructure?.also {
            require(A.rows == it.size)
            { "A matrix must have same size as state vector (${it.size})" }
        } ?: VectorStructure(A.rows)
        require(B.rows == stateSize)
        { "B matrix must have same number of rows as state vector ($stateStructure.size)" }
        this.inputStructure = inputStructure?.also {
            require(B.cols == it.size)
            { "B matrix must have same number of columns as output/signal vector (${it.size})" }
        } ?: VectorStructure(B.cols)
        require(C.cols == stateSize)
        { "C matrix must have same number of columns as state vector ($stateSize)" }
        require(D.cols == inputSize)
        { "D matrix must have same number of columns as input vector ($inputSize)" }
        this.outputStructure = outputStructure?.also {
            require(C.rows == it.size)
            { "C matrix must have same number of rows as output vector ($it.size)" }
            require(D.rows == it.size)
            { "D matrix must have same number of rows as output vector ($it.size)" }
        } ?: run {
            require(C.rows == D.rows)
            { "C and D matrix rows must match to deduce output vector size" }
            VectorStructure(C.rows)
        }
    }

    /** A or system matrix. */
    val A: Mat = A.toImmutableMat()
    /** B or control matrix. */
    val B: Mat = B.toImmutableMat()
    /** C or measurement matrix. Also the measurement Jacobian with respect to the state. */
    val C: Mat = C.toImmutableMat()
    /** D or feed-through matrix. */
    val D: Mat = D.toImmutableMat()


    /** The size of this model's state vector. */
    val stateSize: Int get() = stateStructure.size
    /** The size of this model's input/signal vector. */
    val inputSize: Int get() = inputStructure.size
    /** The size of this model's output/measurement vector. */
    val outputSize: Int get() = outputStructure.size


    /** A matrix */
    operator fun component1(): Mat = A

    /** B matrix */
    operator fun component2(): Mat = B

    /** C matrix */
    operator fun component3(): Mat = C

    /** D matrix */
    operator fun component4(): Mat = D

    /**
     * Gets either x-dot or x_k+1 given the current state vector [x] and signal vector [u]
     */
    fun processState(x: Vec, u: Vec): Vec = A * x + B * u

    /**
     * Gets the output/measurement vector given the current state vector [x] and signal vector [u]
     */
    fun processOutput(x: Vec, u: Vec): Vec = C * x + D * u

    override fun toString(): String = """${this::class.java.simpleName}
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
open class ContinuousLinSSModel @JvmOverloads constructor(
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    stateStructure: VectorStructure? = null,
    inputStructure: VectorStructure? = null,
    outputStructure: VectorStructure? = null
) : LinearStateSpaceModel(A, B, C, D, stateStructure, inputStructure, outputStructure) {


    /**
     * Discretizes this [ContinuousLinSSModel] using the given [period], also discretizing the given [QRCost].
     */
    fun discretize(period: Double, QRCost: QRCost): Pair<DiscreteLinSSModel, QRCost> {
        require(QRCost applicableTo this) { "Cost matrices must be applicable to this model" }
        val model = discretize(period)

        val q = expm(MatConcat.dynamic2x2Square(-A.T, QRCost.Q, 0, A) * period).let {
            it.getQuad(A.rows, 1, 1) * it.getQuad(A.rows, 0, 1)
        }
        val r = QRCost.R / period
        return model to QRCost(q, r)
    }

    /**
     *  Discretizes this [ContinuousLinSSModel] using the given [period]
     */
    fun discretize(period: Double): DiscreteLinSSModel {
        val (ad, bd) = expm(MatConcat.dynamic2x2Square(A, B, 0, 0) * period).let {
            it.getQuad(A.rows, 0, 0) to it.getQuad(A.rows, 0, 1)
        }
        return DiscreteLinSSModel(ad, bd, C, D, period, stateStructure, inputStructure, outputStructure)
    }
}

/**
 * A [LinearStateSpaceModel] that is discrete.
 *
 * @param period the period the discretization is based on.
 */
open class DiscreteLinSSModel @JvmOverloads constructor(
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    val period: Double,
    stateStructure: VectorStructure? = null,
    inputStructure: VectorStructure? = null,
    outputStructure: VectorStructure? = null
) : LinearStateSpaceModel(A, B, C, D, stateStructure, inputStructure, outputStructure)


/**
 * Utilities for creating [LinearStateSpaceModel]s from [FixedDriveModel]s.
 */
object LinearDriveModels {

    /**
     * Derives a [ContinuousLinSSModel] from the given [driveModel] that:
     *
     * - _state_ is bot's pose velocity in [x, y, heading],
     * - _signal/input_ is a vector of motors' voltages in the same order as given in the [driveModel]
     * - _measurement/output_ is a vector of the motors' angular
     *    velocity in the same order as given in the [driveModel].
     *
     * The drive model must be a [FixedDriveModel.isHolonomic] or else the ss model may not be controllable.
     * Otherwise for a non-holonomic, consider first using another controller that maps to _wheel velocities_
     * instead, and use [wheelVelocityController]
     */
    @Suppress("UnnecessaryVariable")
    fun poseVelocityController(driveModel: FixedDriveModel): ContinuousLinSSModel {
        require(driveModel.isHolonomic) { "Drive model must be holonomic" }
        val size = driveModel.numWheels
        val botDeccelFromBotVel = -driveModel.botAccelFromVolts * driveModel.voltsFromBotVel
        val a = botDeccelFromBotVel
        val b = driveModel.botAccelFromVolts
        val motorVelFromBotVel = driveModel.motorVelFromWheelVel * driveModel.wheelVelFromBotVel
        val c = motorVelFromBotVel
        val d = zeros(size, size)
        val names = List(size) { "wheel$it" }
        return ContinuousLinSSModel(
            a, b, c, d,
            stateStructure = VectorStructure(listOf("x", "y", "heading"), repeatedList(3, "")),
            inputStructure = VectorStructure(names, repeatedList(size, "volts")),
            outputStructure = VectorStructure(names, repeatedList(size, "rad/s"))
        )
    }

    /**
     * Derives a [ContinuousLinSSModel] from the given [driveModel] that:
     *
     * - _state_ is a vector of the motors' angular velocity in the same order as given in the [driveModel].
     * - _signal/input_ is a vector of motors' voltages in the same order as given in the [driveModel]
     * - _measurement/output_ directly corresponds to the state (motor angular velocity).
     *
     * The drive model may not be controllable if there are more than 3 wheels
     * since this derivation depends on the fact that the wheels do not slip and so the wheel's velocities are not
     * independent of each other, but dependent on the bot's pose acceleration.
     *
     * In that case you have a holonomic drive, so use [poseVelocityController]
     */
    @Suppress("UnnecessaryVariable")
    fun wheelVelocityController(driveModel: FixedDriveModel): ContinuousLinSSModel {
        val size = driveModel.numWheels
        val wheelDeccelFromWheelVel = -driveModel.wheelAccelFromVolts * driveModel.voltsFromWheelVel
        val a = wheelDeccelFromWheelVel
        val b = driveModel.wheelAccelFromVolts
        val c = eye(size) //measure vel directly
        val d = zeros(size, size)
        val names = List(size) { "wheel$it" }
        val velStructure = VectorStructure(names, repeatedList(size, "rad/s"))
        return ContinuousLinSSModel(
            a, b, c, d,
            stateStructure = velStructure,
            inputStructure = VectorStructure(names, repeatedList(size, "volts")),
            outputStructure = velStructure
        )
    }
}