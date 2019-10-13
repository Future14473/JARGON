package org.futurerobotics.jargon.mechanics

import kotlin.math.sign

/**
 * A not-involving-complex-calculus first-order approximation model of a DC brushed motor following:
 *
 * V = torque/[kt]*[r] + angvel/[kv] + sign(angvel) * [i0] * r
 *
 * All units should be SI units.
 *
 * (See static factory methods)
 *
 * Refer to section 14.2 of [calcmogul's state space guide][https://file.tavsys.net/control/state-space-guide.pdf]
 * for more details. This adds a [i0] term to the thing.
 *
 * This also assumes a positive torque/angular velocity corresponds to a positive voltage,
 * and vice versa.
 *
 * This class is meant to be reusable; Frictional forces are accounted for in [TransmissionModel]
 *
 */
class DcMotorModel private constructor(
    private val kt: Double,
    private val r: Double,
    private val kv: Double,
    private val i0: Double = 0.0
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
    val voltsPerTorque: Double get() = r / kt
    /**
     * Gets the expected amount of volts per angVel needed to maintain a constant speed, assuming no output torque.
     */
    val voltsPerAngVel: Double get() = 1 / kv

    /**
     * The number of volts needed to be added to overcome frictional forces.
     */
    val voltsForFriction: Double get() = i0 * r

    /**
     * Gets the expected voltage to turn this motor at the given [torque] and current [angVel].
     *
     * May be negative.
     */
    fun getVoltage(torque: Double, angVel: Double): Double =
        torque * voltsPerTorque + angVel * voltsPerAngVel + sign(angVel) * voltsForFriction

    companion object {
        /**
         * Constructs a [DcMotorModel] with the given coefficients.
         *
         * @param kt the kt term
         * @param r the r term
         * @param kv the kv term
         * @param i0 the i0 term
         */
        @JvmStatic
        fun fromCoefficients(kt: Double, r: Double, kv: Double, i0: Double = 0.0): DcMotorModel =
            DcMotorModel(kt, r, kv, i0)

        /**
         * Constructs a model based on data give on a motor data sheet.
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
 * Extends a [DcMotorModel] with transmission [gearRatio] and accounts for additional frictional forces.
 *
 * This is a simple yet good enough model for most use cases.
 *
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
    init {
        require(gearRatio.isFinite()) { "gearRatio ($gearRatio) must be finite" }
        require(constantTorqueLoss >= 0) { "constantTorqueLoss ($constantTorqueLoss) must be >= 0" }
        require(ratioTorqueLoss in 0.0..1.0)
        { "ratioTorqueLoss ($ratioTorqueLoss) must be in the range 0.0..1.0" }
    }

    /**
     * Gets the ratio of the motor's torque to the outputTorque.
     */
    val motorTorquePerOutputTorque: Double get() = 1 / gearRatio / ratioTorqueLoss
    /**
     * Gets the ratio of the motor's angular velocity to the output angular velocity.
     */
    val motorAngVelPerOutputAngVel: Double get() = gearRatio
    /**
     * Gets the expected amount of volts per torque, assuming that the motor isn't moving.
     */
    val motorVoltsPerOutputTorque: Double get() = motor.voltsPerTorque * motorTorquePerOutputTorque
    /**
     * Gets the expected amount of volts per angVel needed to maintain a constant velocity, assuming no output torque.
     */
    val voltsPerAngVel: Double get() = motor.voltsPerAngVel * motorAngVelPerOutputAngVel

    /**
     * The extra volts needed to add to overcome frictional forces.
     *
     * This is slightly more useful than free current.
     */
    val voltsForFriction: Double get() = motor.voltsPerTorque * constantTorqueLoss + motor.voltsForFriction

    companion object {
        /**
         * Constructs a [TransmissionModel] with the given parameters, based on torque losses.
         *
         * Frictional forces are divided into two parts: [ratioTorqueLoss] and [constantTorqueLoss].
         *
         * A higher gear ratio means more torque, less speed, and negative gear ratio means going in the opposite direction.
         *
         *
         * [ratioTorqueLoss] is the proportion between 0 and 1, that is the "amount of torque" that makes it to the output,
         * some of which is "lost" due to frictional forces (acceleration related friction), and
         * [constantTorqueLoss] is the constant amount of torque needed to keep the output at a constant speed
         * (velocity related friction).
         *
         * These can be guessed or measured; The goal of a feedback control system is that errors can be accounted for.
         *
         *
         * @param motor the model of the motor used
         * @param gearRatio the gear ratio of this transmission (Input/Output, higher ratio means faster/less torque)
         * @param constantTorqueLoss the constant component of the torque required to overcome frictional forces.
         * @param ratioTorqueLoss the ratio from the output motor torque and the applied motor torque, due to frictional
         *                      or the "percentage of torque" that makes it to the output. In an ideal world, 1.0,
         *                      a deadlocked motor is 0.0.
         */
        @JvmStatic
        @JvmOverloads
        fun fromTorqueLosses(
            motor: DcMotorModel,
            gearRatio: Double,
            constantTorqueLoss: Double = 0.0,
            ratioTorqueLoss: Double = 1.0
        ): TransmissionModel = TransmissionModel(motor, gearRatio, constantTorqueLoss, ratioTorqueLoss)
    }
}
