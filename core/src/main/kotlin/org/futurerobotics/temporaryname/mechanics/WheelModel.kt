package org.futurerobotics.temporaryname.mechanics

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.epsEq

/**
 * A wheel model for a wheel on a bot that cannot change location or direction.
 *
 * @param position where the wheel is located on a bot relative to the center of the bot
 * @param radius the wheel's radius
 * @param orientation a unit vector in the direction the wheel is facing, such that a positive transmission torque results
 *                          in a force applied to the bot in that direction. Directly north, in most cases.
 */
data class FixedWheelModel(
    val transmission: TransmissionModel,
    val position: Vector2d,
    val radius: Double,
    val orientation: Vector2d
) {

    init {
        require(position.isFinite()) { "The wheel position vector ($position) must be finite" }
        require(orientation.length epsEq 1.0) { "The wheelOrientation ($orientation) must be a unit vector" }
        require(radius > 0) { "wheel radius ($radius) should be > 0" }
    }

    constructor(
        transmission: TransmissionModel,
        wheelPosition: Vector2d,
        wheelRadius: Double,
        wheelAngle: Double
    ) : this(
        transmission,
        wheelPosition,
        wheelRadius,
        Vector2d.polar(1.0, wheelAngle)
    )

    /**
     * The ratio between the motor torque and hte force exerted by the wheel.
     */
    val motorTorquePerForce: Double get() = transmission.motorTorquePerOutputTorque * radius
    /**
     * The ratio between the motor angular velocity and the wheel's tangential velocity.
     */
    val motorAngVelPerVelocity: Double get() = transmission.motorAngVelPerOutputAngVel / radius
    /**
     * Gets the expected amount of volts per force applied, assuming the wheel is not moving.
     */
    val voltsPerForce: Double get() = transmission.voltsPerTorque * radius
    /**
     * Gets the expected amount of volts per velocity to maintain the wheel moving at a constant speed.
     */
    val voltsPerVelocity: Double get() = transmission.voltsPerAngVel / radius
    /**
     * @see [TransmissionModel.stallVolts]
     */
    val stallVolts: Double get() = transmission.stallVolts
}
