package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.Vector2d

/**
 * A model for a wheel on a bot that cannot change location or direction on the bot.
 *
 * @param transmission the transmission model
 * @param position where the wheel is located on a bot relative to the center of the bot
 * @param radius the wheel's radius
 * @param orientation a unit vector in the direction the wheel is facing, such that a positive transmission torque results
 *                     in a force applied to the bot in that direction. Directly north, in most cases.
 */
class FixedWheelModel private constructor(
    val transmission: TransmissionModel,
    val position: Vector2d,
    val radius: Double,
    val orientation: Vector2d
) {

    init {
        require(position.isFinite()) { "The wheel position vector ($position) must be finite" }
        require(radius > 0) { "wheel radius ($radius) should be > 0" }
    }

    /**
     * The ratio between the motor torque and the force exerted by the wheel.
     */
    val motorTorquePerOutputForce: Double get() = transmission.motorTorquePerOutputTorque * radius
    /**
     * The ratio between the motor angular velocity and the wheel's tangential velocity.
     */
    val motorVelPerWheelVel: Double get() = transmission.motorAngVelPerOutputAngVel / radius
    /**
     * Gets the expected amount of volts per force applied, assuming the wheel is not moving.
     */
    val motorVoltsPerOutputForce: Double get() = transmission.motorVoltsPerOutputTorque * radius
    /**
     * Gets the expected amount of volts per velocity to maintain the wheel moving at a constant speed.
     */
    val voltsPerWheelVel: Double get() = transmission.voltsPerAngVel / radius
    /**
     * @see [TransmissionModel.voltsForFriction]
     */
    val stallVolts: Double get() = transmission.voltsForFriction

    companion object {
        /**
         * Constructs a [FixedWheelModel] using an orientation unit vector
         *
         *
         * @param transmission the transmission model
         * @param position where the wheel is located on a bot relative to the center of the bot
         * @param radius the wheel's radius
         * @param orientation a unit vector in the direction the wheel is facing, such that a positive transmission torque results
         *                     in a force applied to the bot in that direction. Directly north, in most cases.
         */
        fun fromOrientationVector(
            transmission: TransmissionModel,
            position: Vector2d,
            radius: Double,
            orientation: Vector2d
        ): FixedWheelModel {
            return FixedWheelModel(
                transmission,
                position,
                radius,
                orientation.normalized().also { require(it.isFinite()) })
        }

        /**
         * Constructs a [FixedWheelModel] using a wheel facing angle.
         *
         * @param transmission the transmission model
         * @param position where the wheel is located on a bot relative to the center of the bot
         * @param radius the wheel's radius
         * @param angle the way the wheel is facing, such that a positive transmission torque results in a force applied
         *          to the bot in that direction.
         */
        fun fromWheelAngle(
            transmission: TransmissionModel,
            position: Vector2d,
            radius: Double,
            angle: Double
        ): FixedWheelModel {
            return FixedWheelModel(transmission, position, radius, Vector2d.polar(1.0, angle))
        }
    }
}
