package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.mechanics.BotVelocityModel
import org.futurerobotics.jargon.mechanics.MotorBotVelInteraction
import org.futurerobotics.jargon.mechanics.MotorVelocityModel

/**
 * Utilities for creating common [LinearStateSpaceModel]s from [BotVelocityModel]s.
 */
object DriveStateSpaceModels {

    /**
     * Derives a [ContinuousLinSSModelImpl] from the given [botVelocityModel] that:
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
    @Suppress("UnnecessaryVariable")
    @JvmStatic
    fun poseVelocityController(
        botVelocityModel: BotVelocityModel,
        interaction: MotorBotVelInteraction
    ): ContinuousLinSSModelImpl {
//        require(driveModel.isHolonomic) { "Drive model must be holonomic" }
        // perhaps we can get away with it; if we always set the y vel to 0. Needs testing.
        val size = botVelocityModel.numMotors
        val a = botVelocityModel.botAccelFromBotVel
        val b = botVelocityModel.botAccelFromVolts
        val motorVelFromBotVel = interaction.motorVelFromBotVel
        val c = motorVelFromBotVel
        val d = zeroMat(size, size)
        return ContinuousLinSSModelImpl(a, b, c, d)
    }

    /**
     * Derives a [ContinuousLinSSModelImpl] from the given [driveModel] that:
     *
     * - _state_: is a vector of the motors' angular velocity in the same order as given in the [driveModel].
     * - _signal/input_: is a vector of motors' voltages in the same order as given in the [driveModel]
     * - _measurement/output_: directly corresponds to the state (motor angular velocity).
     *
     * This model is best used for differential-like drives only, as the drive model will not be controllable if there
     * are more than 3 wheels. This is because this derivation depends on the fact that the wheels do not slip and so
     * the wheel's velocities are not independent of each other, but dependent on the bot's pose acceleration.
     *
     * @see poseVelocityController
     * @see decoupledMotorVelocityController
     */
    @Suppress("UnnecessaryVariable")
    @JvmStatic
    fun motorVelocityController(driveModel: MotorVelocityModel):
            ContinuousLinSSModelImpl {
        val motorAccelFromVolts = driveModel.motorAccelFromVolts
        return getMotorVelocityController(driveModel, motorAccelFromVolts)
    }

    /**
     * Derives a [ContinuousLinSSModelImpl] similar to in [motorVelocityController]; while also "decoupling" wheel
     * interactions by multiplying the coupling terms  a factor of [coupling]. 0 means complete decoupling,
     * 1 means do nothing to the model.
     *
     * Empirical testing shows that this model does not perform well under not perfect conditions, which is the
     * state of the world we live in. Instead, we suggest only using this model as a starting estimate of the actual
     * model, and tune from there.
     *
     * The reason this might be required is that the controller assumes that all wheels have perfect traction, so
     * the resulting system is uncontrollable for many holonomic drives/drives with more 4 or more wheels (moving one
     * wheel must move the other wheels, since the bot moves). This breaks the assumption that assumption; by
     * reducing the "coupling" terms.
     */
    @JvmStatic
    fun decoupledMotorVelocityController(driveModel: MotorVelocityModel, coupling: Double): ContinuousLinSSModelImpl {
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

    @Suppress("UnnecessaryVariable")
    private fun getMotorVelocityController(
        motorVelocityModel: MotorVelocityModel,
        motorAccelFromVolts: Mat
    ): ContinuousLinSSModelImpl {
        val size = motorVelocityModel.numMotors
        val a = -motorAccelFromVolts * motorVelocityModel.voltsFromMotorVel
        val b = motorAccelFromVolts
        val c = idenMat(size)
        val d = zeroMat(size, size)
        return ContinuousLinSSModelImpl(a, b, c, d)
    }
}
