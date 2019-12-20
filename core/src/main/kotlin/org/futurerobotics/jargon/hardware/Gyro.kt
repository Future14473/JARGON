package org.futurerobotics.jargon.hardware

/**
 * Represents a gyroscope reading.
 */
interface Gyro {

    /**
     * Gets the current measurement angle in radians. It is recommended to normalize the angle.
     */
    val angle: Double
    /**
     * Gets the current angular velocity in radians per second.
     *
     * If unknown, return 0.
     */
    val angularVelocity: Double
}
