package org.futurerobotics.jargon.hardware

/** Represents a gyroscope, with only [currentAngle] readings. */
interface Gyro {

    /** Gets the current angle in radians. It is recommended to normalize the angle. */
    val currentAngle: Double
}
