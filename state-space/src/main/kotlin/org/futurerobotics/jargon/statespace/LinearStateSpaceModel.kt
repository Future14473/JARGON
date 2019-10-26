@file:Suppress("PropertyName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.mechanics.FixedDriveModel

/**
 * Generic State space model.
 *
 * @param A the system matrix
 * @param B the control matrix
 * @param C the measurement matrix
 * @param D the feed-through matrix, which is usually 0 so nobody cares about it very much
 */
sealed class LinearStateSpaceModel(A: Mat, B: Mat, C: Mat, D: Mat) {

    /** The size of the state vector. */
    val stateSize: Int
    /** The size of the input vector. */
    val inputSize: Int
    /** The size of the output vector. */
    val outputSize: Int

    init {
        require(A.isSquare) { "A matrix must be square" }
        stateSize = A.rows
        inputSize = B.cols
        outputSize = C.rows
        require(B.rows == stateSize)
        { "B matrix must have same number of rows as state vector (${stateSize}.size)" }
        require(C.cols == stateSize)
        { "C matrix must have same number of columns as state vector (${stateSize})" }
        require(D.cols == inputSize)
        { "D matrix must have same number of columns as input vector ($inputSize)" }
        require(D.rows == outputSize)
        { "C and D matrix rows must match to deduce output vector size" }
    }

    /** A or system matrix. */
    val A: Mat = A.toImmutableMat()
    /** B or control matrix. */
    val B: Mat = B.toImmutableMat()
    /** C or measurement matrix. Also the measurement Jacobian with respect to the state. */
    val C: Mat = C.toImmutableMat()
    /** D or feed-through matrix. */
    val D: Mat = D.toImmutableMat()

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
open class ContinuousLinSSModel constructor(A: Mat, B: Mat, C: Mat, D: Mat) : LinearStateSpaceModel(A, B, C, D) {

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
        return DiscreteLinSSModel(ad, bd, C, D, period)
    }
}

/**
 * A [LinearStateSpaceModel] that is discrete.
 *
 * @param period the period the discretization is based on.
 */
open class DiscreteLinSSModel(A: Mat, B: Mat, C: Mat, D: Mat, val period: Double) : LinearStateSpaceModel(A, B, C, D)

/**
 * Utilities for creating [LinearStateSpaceModel]s from [FixedDriveModel]s.
 */
object LinearDriveModels {

    /**
     * This is not recommended for anything beyond testing; use [decoupledMotorVelocityController] instead.
     *
     * Derives a [ContinuousLinSSModel] from the given [driveModel] that:
     *
     * - _state_ is bot's pose velocity in [x, y, heading],
     * - _signal/input_ is a vector of motors' voltages in the same order as given in the [driveModel]
     * - _measurement/output_ is a vector of the motors' angular
     *    velocity in the same order as given in the [driveModel].
     *
     * The drive model must be a [FixedDriveModel.isHolonomic] or else the ss model may not be controllable.
     * Otherwise for a non-holonomic, consider first using another controller that maps to _wheel velocities_
     * instead, and use [motorVelocityController]
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
        val d = zeroMat(size, size)
        return ContinuousLinSSModel(a, b, c, d)
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
    fun motorVelocityController(driveModel: FixedDriveModel): ContinuousLinSSModel {
        val motorAccelFromVolts = driveModel.motorVelFromWheelVel * driveModel.wheelAccelFromVolts
        return getMotorVelocityController(driveModel, motorAccelFromVolts)
    }

    /**
     * Derives a [ContinuousLinSSModel] similar to in [motorVelocityController]; while also "decoupling" wheel interactions
     * by multiplying the coupling terms  a factor of [coupling]. 0 means complete decoupling, 1 means do nothing to the model.
     *
     * The reason this might be required is that the controller assumes that all wheels have perfect traction, so
     * the resulting system is uncontrollable for many holonomic drives/drives with more 4 or more wheels (moving one
     * wheel must move the other wheels, since the bot moves). This breaks the assumption that assumption; by
     * reducing the "coupling" terms.
     *
     * Due to empirical tests, this model performs better [poseVelocityController].
     */
    fun decoupledMotorVelocityController(driveModel: FixedDriveModel, coupling: Double): ContinuousLinSSModel {
        require(coupling in 0.0..1.0) { "coupling ($coupling) must be between 0 and 1, or else things don't make sense." }
        val motorAccelFromVolts = (driveModel.motorVelFromWheelVel * driveModel.wheelAccelFromVolts).apply {
            repeat(rows) { i ->
                repeat(cols) { j ->
                    if (i != j) this[i, j] *= coupling
                }
            }
        }
        return getMotorVelocityController(driveModel, motorAccelFromVolts)
    }

    private fun getMotorVelocityController(
        driveModel: FixedDriveModel,
        motorAccelFromVolts: Mat
    ): ContinuousLinSSModel {
        val size = driveModel.numWheels
        val motorDeccelFromMotorVel =
            -motorAccelFromVolts * driveModel.voltsFromWheelVel * driveModel.wheelVelFromMotorVel
        val a = motorDeccelFromMotorVel
        val b = motorAccelFromVolts
        val c = idenMat(size) //measure vel directly
        val d = zeroMat(size, size)
        return ContinuousLinSSModel(a, b, c, d)
    }
}
