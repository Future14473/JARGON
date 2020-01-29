package org.futurerobotics.jargon.hardware

/**
 * Represents a gyroscope reading.
 */
interface Gyro {

    /**
     * Gets the current measurement angle in radians.
     */
    val angle: Double

    /**
     * Gets the current angular velocity in radians per second.
     */
    val angularVelocity: Double
}
