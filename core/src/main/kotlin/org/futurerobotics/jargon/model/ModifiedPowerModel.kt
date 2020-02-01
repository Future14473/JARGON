package org.futurerobotics.jargon.model

/**
 * Represents the modification of another [TorquePowerModel] (such as [DcMotorModel])
 * to account for non-ideal conditions (like friction compensation).
 */
class ModifiedPowerModel private constructor(
    private val innerModel: TorquePowerModel,
    private val factorForTorque: Double,
    private val factorForVel: Double,
    private val moreAdditionalVolts: Double
) : TorquePowerModel {

    override val voltsPerTorque: Double get() = innerModel.voltsPerTorque * factorForTorque
    override val torquePerVolt: Double get() = 1 / voltsPerTorque
    override val voltsPerVel: Double get() = innerModel.voltsPerVel * factorForVel
    override val velPerVolt: Double get() = 1 / voltsPerVel
    /**
     * The additional amount of volts needed to be applied to compensate for friction, in the direction of motion.
     */
    override val additionalVoltsForFriction: Double
        get() = innerModel.additionalVoltsForFriction + voltsPerTorque * moreAdditionalVolts

    companion object {
        /**
         * Creates a [ModifiedPowerModel].
         *
         * @param model the original inner model.
         * @param factorForTorque The amount of volts needed for torque should be multiplied by the given ratio.
         * This can account for some frictional forces related to acceleration. Default = 1.0.
         * @param factorForVel The amount of volts needed to maintain velocity should be multiplied by the
         * given ratio. This can account for some frictional forces related to velocity. Default = 1.0.
         * @param additionalVoltsForFriction An additional amount of volts
         * to be given in the direction of motion. This can account for some
         * static frictional forces. Default = 0.0.
         */
        @JvmStatic
        @JvmOverloads
        fun of(
            model: TorquePowerModel,
            factorForTorque: Double = 1.0,
            factorForVel: Double = 1.0,
            additionalVoltsForFriction: Double = 0.0
        ): ModifiedPowerModel {

            require(factorForTorque.isFinite() && factorForTorque > 0)
            { "factorForTorque ($factorForTorque) must be finite and > 0" }
            require(factorForVel.isFinite() && factorForVel > 0)
            { "factorForVel ($factorForVel) must be finite and > 0" }
            require(additionalVoltsForFriction.isFinite() && additionalVoltsForFriction >= 0)
            { "moreAdditionalVolts ($additionalVoltsForFriction) must be finite and > 0" }
            return if (model is ModifiedPowerModel) {
                ModifiedPowerModel(
                    model.innerModel,
                    model.factorForTorque * factorForTorque,
                    model.factorForVel * factorForVel,
                    model.moreAdditionalVolts + additionalVoltsForFriction
                )
            } else
                ModifiedPowerModel(
                    model,
                    factorForTorque,
                    factorForVel,
                    additionalVoltsForFriction
                )
        }
    }
}
