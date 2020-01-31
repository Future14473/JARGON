package org.futurerobotics.jargon.model

/**
 * A simple, linear, first-order approximation model of a DC motor, based off the given model:
 *
 * V = torque/kt*r + angvel/kv + sign(angvel) * i0 * r
 *
 * All units should be SI units; and values can also be derived off of a motor data sheet.
 *
 * See static factory methods for construction.
 *
 * This is based off of section 14.2 of
 * [calcmogul's state space guide][https://file.tavsys.net/control/state-space-guide.pdf], with an added `i0` term.
 * term for friction.
 */
class DcMotorModel
private constructor(
    kt: Double,
    r: Double,
    kv: Double,
    i0: Double = 0.0
) : TorquePowerModel {

    override val voltsPerTorque: Double = r / kt
    override val torquePerVolt: Double get() = 1 / voltsPerTorque
    override val voltsPerVel: Double = 1 / kv
    override val velPerVolt: Double get() = 1 / voltsPerVel
    override val additionalVoltsForFriction: Double = i0 * r

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
        fun fromCoefficients(kt: Double, r: Double, kv: Double, i0: Double = 0.0): DcMotorModel {
            require(kt > 0) { "kt ($kt) must be > 0" }
            require(r > 0) { "r ($r) must be > 0" }
            require(kv > 0) { "kv ($kv) must be > 0" }
            require(i0 >= 0) { "kv ($kv) must be >= 0" }
            return DcMotorModel(kt, r, kv, i0)
        }

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
        ): DcMotorModel {
            require(voltage > 0) { "voltage $voltage should be >= 0" }
            require(stallTorque > 0) { "stallTorque $stallTorque should be >= 0" }
            require(stallCurrent > 0) { "stallCurrent $stallCurrent should be >= 0" }
            require(freeAngularVelocity > 0) { "freeAngularVelocity $freeAngularVelocity should be >= 0" }
            require(freeCurrent >= 0) { "freeCurrent $freeCurrent should be >= 0" }
            val r = voltage / stallCurrent
            return DcMotorModel(
                stallTorque / stallCurrent,
                r,
                freeAngularVelocity / (voltage - freeCurrent * r),
                freeCurrent
            )
        }
    }
}
