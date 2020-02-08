package org.futurerobotics.jargon.model

/**
 * Represents some gearing from input to output/the expected ratio of input to output velocity.
 */
interface Gearing {

    /**
     * The ratio from OUTPUT velocity to INPUT velocity.
     *
     * Units depend on context (angular velocity is rad/s, linear velocity is m/s)
     */
    val outputVelPerInputVel: Double
    /**
     * The ratio from INPUT velocity to OUTPUT velocity.
     *
     * Units depend on context (angular velocity is rad/s, linear velocity is m/s)
     */
    val inputVelPerOutputVel: Double // == gear ratio / wheel radius
}

/**
 * Common parts of [TorquePowerModel] and [ForcePowerModel]. Use those.
 */
interface VoltageVelModel {

    /**
     * The expected amount of volts needed to maintain a constant speed at the _output_, assuming no output torque.
     */
    val voltsPerVel: Double
    /**
     * The expected angular velocity at the _output_ maintained by the given constant voltage, assuming no output
     * torque.
     */
    val velPerVolt: Double

    /**
     * The additional amount of voltage needed to be applied to compensate for friction, in the direction of motion.
     */
    val additionalVoltsForFriction: Double
}

/**
 * A model that represents the relationship between voltage, output torque, and input/output velocity.
 *
 * @see DcMotorModel
 * @see ModifiedPowerModel
 */
interface TorquePowerModel : VoltageVelModel {

    /**
     * The expected amount of volts required per torque, assuming that the motor isn't moving.
     */
    val voltsPerTorque: Double
    /**
     * The expected amount of torque per input volt, assuming that the motor isn't moving.
     */
    val torquePerVolt: Double
}

/**
 * A model that represents the relationship between voltage, output _force_, output velocity,
 * and input velocity.
 *
 * This is to distinguish force from torque which may easily be confused.
 *
 * @see PoweredWheelModel
 * @see PoweredDriveWheelModel
 */
interface ForcePowerModel : VoltageVelModel, Gearing {

    /**
     * The expected amount of volts required per force, assuming that the motor isn't moving.
     */
    val voltsPerForce: Double
    /**
     * The expected amount of force per input volt, assuming that the motor isn't moving.
     */
    val forcePerVolt: Double
}
