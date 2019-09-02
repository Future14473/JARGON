package org.futurerobotics.temporaryname.mechanics

import kotlin.math.sign

/**
 * A not-involving-complex-calculus first-order approximation of a model of a DC brushed motor following:
 *
 * V = torque/[kt]*[r] + angvel/[kv] + sign(angvel) * [i0] * r
 *
 * All units should be SI units; convert units first!
 *
 * Refer to section 14.2 of [calcmogul's state space guide][https://file.tavsys.net/control/state-space-guide.pdf]
 * for more details.
 *
 * This also assumes a positive torque/angular velocity corresponds to a positive voltage,
 * and vice versa.
 *
 * This class is meant to be reusable; Frictional forces are accounted for in [TransmissionModel]
 *
 * @param kt the kt term; see [DcMotorModel]
 * @param r the r term; see [DcMotorModel]
 * @param kv the kv term; see [DcMotorModel]
 */
class DcMotorModel private constructor(
    private val kt: Double,
    private val r: Double,
    private val kv: Double,
    val i0: Double = 0.0
) {

    /**
     * Gets the expected amount of volts per output torque, assuming that the motor isn't moving.
     */
    val voltsPerTorque: Double get() = r / kt
    /**
     * Gets the expected amount of volts per angVel needed to maintain a constant speed, assuming no output torque.
     */
    val voltsPerAngVel: Double get() = 1 / kv
    /**
     * The number of volts needed to add to overcome frictional forces.
     */
    val voltsForFriction: Double get() = i0 * r

    init {
        require(kt > 0) { "kt ($kt) must be > 0" }
        require(r > 0) { "r ($r) must be > 0" }
        require(kv > 0) { "kv ($kv) must be > 0" }
    }

    /**
     * Gets the expected voltage to turn this motor at the given [torque] and current [angVel].
     *
     * May be negative.
     */
    fun getVoltage(torque: Double, angVel: Double): Double {
        return torque * voltsPerTorque + angVel * voltsPerAngVel + sign(angVel) * voltsForFriction
    }

    companion object {
        /**
         * Constructs a [DcMotorModel] with the given coefficients.
         */
        @JvmStatic
        fun fromCoefficients(kt: Double, r: Double, kv: Double, i0: Double = 0.0): DcMotorModel {
            return DcMotorModel(kt, r, kv, i0)
        }

        /**
         * Constructs based on data give on a motor data sheet.
         */
        @JvmStatic
        fun fromMotorData(
            voltage: Double,
            stallTorque: Double,
            stallCurrent: Double,
            freeAngularVelocity: Double,
            freeCurrent: Double
        ): DcMotorModel {
            return DcMotorModel(
                stallTorque / stallCurrent,
                voltage / stallCurrent,
                freeAngularVelocity / voltage,
                freeCurrent
            )
        }
    }
}

/**
 * Extends a [motor] with transmission [gearRatio], and accounts for additional frictional forces.
 *
 * This is a simple yet good enough model for most use cases.
 *
 * Frictional forces are divided into two parts:
 * [ratioTorqueLoss] and [constantTorqueLoss].
 * [ratioTorqueLoss] is the ratio of the torque "lost" due to frictional forces (acceleration), while [constantTorqueLoss] is
 * the constant amount of torque needed to keep the transmission at a constant speed (velocity).
 *
 * A negative gear ratio means going in the opposite direction.
 * @param motor the model of the motor used
 * @param gearRatio the gear ratio of this transmission (Input/Output, higher ratio means faster/less torque)
 * @param constantTorqueLoss the constant component of the torque required to overcome frictional forces.
 * @param ratioTorqueLoss the ratio from the output motor torque and the applied motor torque, due to frictional
 *                      or the "percentage of torque" that makes it to the output. In an ideal world, 1.0,
 *                      a deadlocked motor is 0.0.
 */
class TransmissionModel private constructor(
    val motor: DcMotorModel,
    val gearRatio: Double,
    val constantTorqueLoss: Double = 0.0,
    val ratioTorqueLoss: Double = 1.0
) {
    //outSpeed = inSpeed * gearRatio
    //outTorque = inTorque * ratioFrictionalTorque / gearRatio
    /**
     * Gets the ratio of the motor's torque to the outputTorque.
     */
    val motorTorquePerOutputTorque: Double get() = gearRatio / ratioTorqueLoss
    /**
     * Gets the ratio of the motor's angular velocity to the output angular velocity.
     */
    val motorAngVelPerOutputAngVel: Double get() = 1 / gearRatio
    /**
     * Gets the expected amount of volts per torque, assuming that the motor isn't moving.
     */
    val voltsPerTorque: Double get() = motor.voltsPerTorque * motorTorquePerOutputTorque
    /**
     * Gets the expected amount of volts per angVel needed to maintain a constant velocity, assuming no output torque.
     */
    val voltsPerAngVel: Double get() = motor.voltsPerAngVel * motorAngVelPerOutputAngVel
    /**
     * The idle extra volts needed to keep a motor at 0 torque, due to frictional forces.
     *
     * This is slightly more useful than stall current.
     */
    val voltsForFriction: Double get() = motor.voltsPerTorque * constantTorqueLoss + motor.voltsForFriction

    init {
        require(gearRatio.isFinite()) { "gearRatio ($gearRatio) must be finite" }
        require(constantTorqueLoss >= 0) { "frictionalTorque ($constantTorqueLoss) must be >= 0" }
        require(ratioTorqueLoss in 0.0..1.0) { "ratioFrictionalTorque ($ratioTorqueLoss) must be in the range 0.0..1.0" }
    }

    companion object {
        /**
         * Constructs a [TransmissionModel] with the given parameters.
         */
        @JvmStatic
        @JvmOverloads
        fun fromTorqueLosses(
            motor: DcMotorModel,
            gearRatio: Double,
            constantTorqueLoss: Double = 0.0,
            ratioTorqueLoss: Double = 1.0
        ): TransmissionModel {
            return TransmissionModel(motor, gearRatio, constantTorqueLoss, ratioTorqueLoss)
        }
    }
}
