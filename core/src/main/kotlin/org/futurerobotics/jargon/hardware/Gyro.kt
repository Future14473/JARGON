package org.futurerobotics.jargon.hardware

import org.futurerobotics.jargon.system.InitStoppable


/** Represents a gyroscope */
interface Gyro : InitStoppable {
    /** Gets the current angle in radians. */
    val currentAngle: Double
}