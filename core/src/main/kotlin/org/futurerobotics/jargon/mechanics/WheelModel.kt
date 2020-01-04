package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.epsEq

/**
 * Represents the position and orientation and radius of a wheel that is _fixed_ to the bot (not swerve).
 *
 *
 * @param locationAboutCenter where the wheel is located on a bot relative to the center of the bot
 * @param radius the wheel's radius
 * @param orientation a unit vector in the direction the wheel is facing, such that a positive transmission torque results
 *                     in a force applied to the bot in that direction. Directly north, in most cases.
 */
data class WheelPosition(
    val locationAboutCenter: Vector2d,
    val orientation: Vector2d,
    val radius: Double
) {

    init {
        require(locationAboutCenter.isFinite()) { "The wheel position ($locationAboutCenter) must be finite" }
        require(orientation.length epsEq 1.0) { "The orientation vector ($orientation)must be a unit vector" }
        require(radius > 0) { "wheel radius ($radius) must be > 0" }
    }

    /**
     * The ratio of the wheel's tangential velocity compared to the bot's
     */
    val tangentVelPerBotVel: Double = locationAboutCenter cross orientation
}

/**
 * A simple model for a wheel on a bot that cannot change location or direction on the bot.
 *
 * @param transmission the transmission model
 * @param wheelPosition the wheel's location on the bot.
 */
data class WheelModel(
    val wheelPosition: WheelPosition,
    val transmission: TransmissionModel
) {

    /**
     * The ratio between the motor angular velocity and the wheel's tangential velocity.
     */
    val motorVelPerOutputVel: Double get() = transmission.gearRatio / wheelPosition.radius

    /**
     * Gets the expected amount of volts per force applied, assuming the wheel is not moving.
     */
    val voltsPerOutputForce: Double
        get() = transmission.voltsPerOutputTorque * wheelPosition.radius
    /**
     * Gets the expected amount of volts per velocity to maintain the wheel moving at a constant speed.
     */
    val voltsPerOutputVel: Double get() = transmission.voltsPerOutputVel / wheelPosition.radius

    /**
     * The additional amount of force needed to be applied to compensate for friction, in the direction of motion.
     */
    val forceForFriction: Double get() = transmission.torqueForFriction / wheelPosition.radius
}
