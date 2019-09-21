@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.VectorStructure
import org.futurerobotics.jargon.mechanics.FixedWheelDriveModel
import org.futurerobotics.jargon.mechanics.HolonomicDriveModel
import org.futurerobotics.jargon.util.repeatedList


/**
 * Generic State space model.
 *
 * @param A the system matrix
 * @param B the control matrix
 * @param C the measurement matrix
 * @param D the feed-through matrix, which is usually 0 so nobody cares about it very much
 */
sealed class StateSpaceModel constructor(
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    stateStructure: VectorStructure? = null,
    inputStructure: VectorStructure? = null,
    outputStructure: VectorStructure? = null
) {
    val stateStructure: VectorStructure
    val inputStructure: VectorStructure
    val outputStructure: VectorStructure
    /** A or system matrix. */
    val A: Mat = A.copy()
    /** B or control matrix. */
    val B: Mat = B.copy()
    /** C or measurement matrix. Also the measurement Jacobian with respect to the state. */
    val C: Mat = C.copy()
    /** D or feed-through matrix. */
    val D: Mat = D.copy()

    init {
        require(A.isSquare) { "A matrix must be square" }
        this.stateStructure = stateStructure?.also {
            require(A.rows == it.size)
            { "A matrix must have same size as state vector (${it.size})" }
        } ?: VectorStructure(A.rows)
        require(B.rows == stateSize)
        { "B matrix must have same number of rows as state vector ($stateStructure.size)" }
        this.inputStructure = inputStructure?.also {
            require(B.rows == it.size)
            { "B matrix must have same number of columns as output/signal vector (${it.size})" }
        } ?: VectorStructure(B.rows)
        require(C.rows == stateSize)
        { "C matrix must have same number of columns as state vector ($stateSize)" }
        require(D.rows == inputSize)
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

    /** The size of this model's state vector. */
    val stateSize: Int get() = stateStructure.size
    /** The size of this model's input/signal vector. */
    val inputSize: Int get() = inputStructure.size
    /** The size of this model's output/measurement vector. */
    val outputSize: Int get() = outputStructure.size


    operator fun component1() = A
    operator fun component2() = B
    operator fun component3() = C
    operator fun component4() = D
}


/**
 * A [StateSpaceModel] that is continuous.
 */
open class ContinuousSSModel @JvmOverloads constructor(
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    stateStructure: VectorStructure? = null,
    inputStructure: VectorStructure? = null,
    outputStructure: VectorStructure? = null
) : StateSpaceModel(A, B, C, D, stateStructure, inputStructure, outputStructure) {


    fun discretize(period: Double, QRCost: QRCost): Pair<DiscreteSSModel, QRCost> {
        require(QRCost applicableTo this) { "Cost matrices must be applicable to this model" }
        val model = discretize(period)

        val q = MatConcat.square2x2(-A.T, QRCost.Q, 0, A).expm().let {
            it.getQuad(1, 1) * it.getQuad(0, 1)
        }
        val r = QRCost.R / period
        return model to QRCost(q, r)
    }

    fun discretize(period: Double): DiscreteSSModel {
        val (ad, bd) = MatConcat.square2x2(A, B, 0, 0).expm().let {
            it.getQuad(0, 0) to it.getQuad(0, 1)
        }
        return DiscreteSSModel(period, ad, bd, C, D, stateStructure, inputStructure, outputStructure)
    }

    companion object {
        /**
         * Derives a [ContinuousSSModel] from the given [driveModel] that:
         *
         * - _state_ is bot's pose velocity in [x, y, heading],
         * - _signal/input_ is a vector of motors' voltages in the same order as given in the [driveModel]
         * - _measurement/output_ is a vector of the motors' angular
         *    velocity in the same order as given in the [driveModel].
         *
         * The drive model must be a [HolonomicDriveModel] or else the ss model may not be controllable.
         * Otherwise for a non-holonomic, consider first using another controller that maps to _wheel velocities_
         * instead, and use [wheelVelocityController]
         */
        fun poseVelocityController(driveModel: HolonomicDriveModel): ContinuousSSModel {
            val size = driveModel.numWheels
            val botDeccelFromBotVel = -driveModel.botAccelFromVolts * driveModel.voltsFromBotVel
            val a = botDeccelFromBotVel
            val b = driveModel.botAccelFromVolts
            val motorVelFromBotVel = driveModel.motorVelFromWheelVel * driveModel.wheelVelFromBotVel
            val c = motorVelFromBotVel
            val d = zeros(size, size)
            val names = List(size) { "wheel$it" }
            return ContinuousSSModel(
                a, b, c, d,
                stateStructure = VectorStructure(listOf("x", "y", "heading")),
                inputStructure = VectorStructure(names, repeatedList(size, "volts")),
                outputStructure = VectorStructure(names, repeatedList(size, "rad/s"))
            )
        }

        /**
         * Derives a [ContinuousSSModel] from the given [driveModel] that:
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
        fun wheelVelocityController(driveModel: FixedWheelDriveModel): ContinuousSSModel {
            val size = driveModel.numWheels
            val wheelDeccelFromWheelVel = -driveModel.wheelAccelFromVolts * driveModel.voltsFromWheelVel
            val a = wheelDeccelFromWheelVel
            val b = driveModel.wheelAccelFromVolts
            val c = pureEye(size) //measure vel directly
            val d = zeros(size, size)
            val names = List(size) { "wheel$it" }
            val velStructure = VectorStructure(names, repeatedList(size, "rad/s"))
            return ContinuousSSModel(
                a, b, c, d,
                stateStructure = velStructure,
                inputStructure = VectorStructure(names, repeatedList(size, "volts")),
                outputStructure = velStructure
            )
        }
    }
}

/**
 * A [StateSpaceModel] that is discrete.
 *
 * @param period the period the discretization is based on.
 */
open class DiscreteSSModel @JvmOverloads constructor(
    val period: Double,
    A: Mat,
    B: Mat,
    C: Mat,
    D: Mat,
    stateStructure: VectorStructure? = null,
    inputStructure: VectorStructure? = null,
    outputStructure: VectorStructure? = null
) : StateSpaceModel(A, B, C, D, stateStructure, inputStructure, outputStructure)