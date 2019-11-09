package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.Vector2d

/**
 * Represents the position and orientation and radius of a wheel that is _fixed_ to the bot (not swerve).
 *
 *
 * @param location where the wheel is located on a bot relative to the center of the bot
 * @param radius the wheel's radius
 * @param orientation a unit vector in the direction the wheel is facing, such that a positive transmission torque results
 *                     in a force applied to the bot in that direction. Directly north, in most cases.
 */
data class WheelLocation(
    val location: Vector2d,
    val radius: Double,
    val orientation: Vector2d
) {

    init {
        require(location.isFinite()) { "The wheel position vector ($location) must be finite" }
        require(radius > 0) { "wheel radius ($radius) should be > 0" }
    }
}

/**
 * A simple model for a wheel on a bot that cannot change location or direction on the bot.
 *
 * @param transmission the transmission model
 * @param wheelLocation the wheel's location on the bot.
 */
data class WheelModel(
    val transmission: TransmissionModel,
    val wheelLocation: WheelLocation
) {

    /**
     * The ratio between the motor torque and the force exerted by the wheel.
     */
    val motorTorquePerOutputForce: Double get() = transmission.motorTorquePerOutputTorque * wheelLocation.radius
    /**
     * The ratio between the motor angular velocity and the wheel's tangential velocity.
     */
    val motorVelPerOutputVel: Double get() = transmission.motorVelPerOutputVel / wheelLocation.radius
    /**
     * Gets the expected amount of volts per force applied, assuming the wheel is not moving.
     */
    val voltsPerOutputForce: Double get() = transmission.voltsPerOutputTorque * wheelLocation.radius
    /**
     * Gets the expected amount of volts per velocity to maintain the wheel moving at a constant speed.
     */
    val voltsPerOutputVel: Double get() = transmission.voltsPerOutputVel / wheelLocation.radius
    /**
     * @see [TransmissionModel.voltsForFriction]
     */
    val voltsForFriction: Double get() = transmission.voltsForFriction
}
