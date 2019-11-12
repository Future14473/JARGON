package org.futurerobotics.jargon.mechanics

import kotlin.math.sign

/**
 * A not-involving-complex-calculus first-order approximation model of a DC brushed motor, based off the given
 * model:
 *
 * V = torque/kt*r + angvel/kv + sign(angvel) * i0 * r
 *
 * All units should be SI units; and values can also be derived off of a motor data sheet (see factory methods).
 *
 * This is based off of section 14.2 of
 * [calcmogul's state space guide][https://file.tavsys.net/control/state-space-guide.pdf], with an added [i0]
 * term for friction.
 *
 * This also assumes a positive torque/angular velocity corresponds to a positive voltage.
 *
 * Additional frictional forces are accounted for in [TransmissionModel].
 */
class MotorModel private constructor(
    kt: Double,
    r: Double,
    kv: Double,
    i0: Double = 0.0
) {

    init {
        require(kt > 0) { "kt ($kt) must be > 0" }
        require(r > 0) { "r ($r) must be > 0" }
        require(kv > 0) { "kv ($kv) must be > 0" }
        require(i0 >= 0) { "kv ($kv) must be >= 0" }
    }

    /**
     * Gets the expected amount of volts per output torque, assuming that the motor isn't moving.
     */
    val voltsPerTorque: Double = r / kt
    /**
     * Gets the expected amount of volts per angVel needed to maintain a constant speed, assuming no output torque.
     */
    val voltsPerAngVel: Double = 1 / kv
    /**
     * The additional amount of force needed to be applied to compensate for friction, in the direction of motion.
     */
    val torqueForFriction: Double = i0 * r / voltsPerTorque

    /**
     * Gets the expected voltage to turn this motor to apply a given [torque], at the current [angVel].
     *
     * May be negative.
     */
    fun getVoltage(torque: Double, angVel: Double): Double =
        (torque + torqueForFriction * sign(angVel)) * voltsPerTorque + angVel * voltsPerAngVel

    companion object {
        /**
         * Constructs a [MotorModel] with the given coefficients.
         *
         * @param kt the kt term
         * @param r the r term
         * @param kv the kv term
         * @param i0 the i0 term
         */
        @JvmStatic
        fun fromCoefficients(kt: Double, r: Double, kv: Double, i0: Double = 0.0): MotorModel =
            MotorModel(kt, r, kv, i0)

        /**
         * Constructs a model based on data given on a motor data sheet.
         */
        @JvmStatic
        fun fromMotorData(
            voltage: Double,
            stallTorque: Double,
            stallCurrent: Double,
            freeAngularVelocity: Double,
            freeCurrent: Double
        ): MotorModel {
            val r = voltage / stallCurrent
            return MotorModel(
                stallTorque / stallCurrent,
                r,
                freeAngularVelocity / (voltage - freeCurrent * r),
                freeCurrent
            )
        }
    }
}

/**
 * Extends a [MotorModel] with transmission [gearRatio] and accounts for additional frictional forces.
 *
 * This is a simple yet good enough model for most use cases.
 */
class TransmissionModel private constructor(
    val motor: MotorModel,
    val gearRatio: Double,
    val additionalConstantTorque: Double = 0.0,
    val outputTorqueMultiplier: Double = 1.0
) {

    init {
        require(gearRatio > 0) { "gearRatio ($gearRatio) must be > 0" }
        require(additionalConstantTorque >= 0) { "constantTorqueLoss ($additionalConstantTorque) must be >= 0" }
        require(outputTorqueMultiplier in 0.0..1.0)
        { "ratioTorqueLoss ($outputTorqueMultiplier) must be in the range 0.0..1.0" }
    }

    /**
     * Gets the expected amount of volts per output torque, given zero velocity.
     */
    val voltsPerOutputTorque: Double get() = motor.voltsPerTorque / gearRatio / outputTorqueMultiplier
    /**
     * Gets the expected amount of volts per angVel needed to maintain a constant velocity, assuming no output torque.
     */
    val voltsPerOutputVel: Double get() = motor.voltsPerAngVel * gearRatio
    /**
     * The additional amount of force needed to be applied to compensate for friction, in the direction of motion.
     */
    val torqueForFriction: Double get() = motor.torqueForFriction * gearRatio + additionalConstantTorque

    companion object {
        /**
         * Constructs a [TransmissionModel] with the given parameters, with friction based on torque losses.
         *
         * Frictional forces are divided into two parts: [ratioTorqueLoss] and [constantTorqueLoss].
         *
         * @param motor the motor model used
         * @param gearRatio the gear ratio of this transmission (higher ratio means more torque, less speed)
         * @param constantTorqueLoss the constant component of the torque required to overcome frictional forces.
         * @param ratioTorqueLoss the ratio from the output motor torque and the applied motor torque, some lost due to
         *                      acceleration related friction. This can also be interpreted as the the "percentage of torque"
         *                      that makes it to the output. In an ideal world, 1.0, for a deadlocked motor, 0.0.
         */
        @JvmStatic
        @JvmOverloads
        fun fromTorqueLosses(
            motor: MotorModel,
            gearRatio: Double,
            constantTorqueLoss: Double = 0.0,
            ratioTorqueLoss: Double = 1.0
        ): TransmissionModel = TransmissionModel(motor, gearRatio, constantTorqueLoss, ratioTorqueLoss)
    }
}
