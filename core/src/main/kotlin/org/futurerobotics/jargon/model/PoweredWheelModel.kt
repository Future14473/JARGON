package org.futurerobotics.jargon.model

/**
 * A [ForcePowerModel] that represents a [TorquePowerModel] connected to a wheel/spool.
 *
 * This is different from [PoweredDriveWheelModel] in it does not include wheel position/orientation
 * info.
 */
class PoweredWheelModel
private constructor(
    private val gearRatioOverWheelRadius: Double,
    private val power: TorquePowerModel
) : ForcePowerModel {

    override val outputVelPerInputVel: Double
        get() = 1 / inputVelPerOutputVel
    override val inputVelPerOutputVel: Double get() = gearRatioOverWheelRadius
    override val voltsPerVel: Double get() = power.voltsPerVel * inputVelPerOutputVel
    override val velPerVolt: Double get() = 1 / voltsPerVel
    override val additionalVoltsForFriction: Double get() = power.additionalVoltsForFriction
    override val voltsPerForce: Double get() = power.voltsPerTorque / inputVelPerOutputVel
    override val forcePerVolt: Double get() = 1 / voltsPerForce

    companion object {
        /**
         * Creates a [PoweredDriveWheelModel].
         *
         * @param wheelRadius the wheel's radius
         * @param gearRatio the gear ratio from output to input. A higher gear ratio means more torque, less speed.
         * @param power the [TorquePowerModel] to use.
         */
        @JvmStatic
        fun of(
            wheelRadius: Double,
            gearRatio: Double,
            power: TorquePowerModel
        ): PoweredWheelModel =
            PoweredWheelModel(
                checkWG(wheelRadius, gearRatio),
                power
            )

        internal fun from(
            gearRatioOverWheelRadius: Double,
            power: TorquePowerModel
        ): PoweredWheelModel =
            PoweredWheelModel(
                gearRatioOverWheelRadius,
                power
            )
    }
}

internal fun checkWG(wheelRadius: Double, gearRatio: Double): Double {
    require(wheelRadius.isFinite() && wheelRadius > 0)
    { "wheelRadius ($wheelRadius) must be finite and > 0" }
    require(gearRatio.isFinite() && gearRatio > 0)
    { "gearRatio ($gearRatio) must be finite and > 0" }
    return gearRatio / wheelRadius
}

