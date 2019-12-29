package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.mechanics.BotVelocityModel
import org.futurerobotics.jargon.mechanics.MotorBotInteraction
import org.futurerobotics.jargon.mechanics.MotorVelocityModel

/**
 * Utilities for creating common initial [ContinuousStateSpaceMatrices]s from [BotVelocityModel]s.
 */
object DriveStateSpaceModels {

    /**
     * Creates a [ContinuousStateSpaceMatrices] in which:
     *
     * - _state_: is bot's pose velocity in [x, y, heading],
     * - _signal/input_: is a vector of motors' voltages in the same order as given in the [botVelocityModel]
     * - _measurement/output_: is a vector of the motors' angular
     *    velocity in the same order as given in the [driveModel].
     *
     * The drive model must be a holonomic or else the model will not be controllable.
     * Otherwise, for a non-holonomic drive, consider first mapping signals into to _motor velocities_
     * instead, and use [motorVelocityController].
     *
     * @see decoupledMotorVelocityController
     */
    @JvmStatic
    fun poseVelocityController(
        botVelocityModel: BotVelocityModel,
        interaction: MotorBotInteraction
    ): ContinuousStateSpaceMatrices {
        val a = botVelocityModel.botAccelFromBotVel.copy()
        val b = botVelocityModel.botAccelFromVolts.copy()
        val motorVelFromBotVel = interaction.motorVelFromBotVel
        val c = motorVelFromBotVel.copy()
        return ContinuousStateSpaceMatrices(a, b, c)
    }

    /**
     * Creates a [ContinuousStateSpaceMatrices] in which:
     *
     * - _state_: is a vector of the motors' angular velocity in the same order as given in the [motorVelocityModel].
     * - _signal/input_: is a vector of motors' voltages in the same order as given in the [motorVelocityModel]
     * - _measurement/output_: directly corresponds to the state (motor angular velocity).
     *
     * This model is best used for differential-like drives only, as the drive model will not be controllable if there
     * are more than 3 wheels. This is because this derivation depends on the fact that the wheels do not slip and so
     * the wheel's velocities are not independent of each other, but dependent on the bot's pose acceleration.
     *
     * @see poseVelocityController
     * @see decoupledMotorVelocityController
     */
    @JvmStatic
    fun motorVelocityController(motorVelocityModel: MotorVelocityModel): ContinuousStateSpaceMatrices {
        val motorAccelFromVolts = motorVelocityModel.motorAccelFromVolts
        return getMotorVelocityController(motorVelocityModel, motorAccelFromVolts)
    }

    /**
     * Derives a [ContinuousStateSpaceMatrices] similar to [motorVelocityController]; while also "decoupling" motor
     * interactions by multiplying the coupling terms a factor of [coupling] (reducing how applying a voltage to one
     * motor affects the motion of _other_ motors). means complete decoupling, 1 means do nothing to the model.
     *
     * While this creates a motor velocity controller that is controllable, it does so at the cost of removing
     * the model from reality slightly.
     */
    @JvmStatic
    fun decoupledMotorVelocityController(
        driveModel: MotorVelocityModel,
        coupling: Double
    ): ContinuousStateSpaceMatrices {
        require(coupling in 0.0..1.0) { "coupling ($coupling) must be between 0 and 1, or else things don't make sense." }
        val motorAccelFromVolts = driveModel.motorAccelFromVolts.apply {
            repeat(rows) { r ->
                repeat(cols) { c ->
                    if (r != c) this[r, c] *= coupling
                }
            }
        }
        return getMotorVelocityController(driveModel, motorAccelFromVolts)
    }

    private fun getMotorVelocityController(
        motorVelocityModel: MotorVelocityModel,
        motorAccelFromVolts: Mat
    ): ContinuousStateSpaceMatrices {
        val size = motorVelocityModel.numMotors
        val a = -motorAccelFromVolts * motorVelocityModel.voltsFromMotorVel
        val b = motorAccelFromVolts.copy()
        val c = idenMat(size)
        return ContinuousStateSpaceMatrices(a, b, c)
    }
}
