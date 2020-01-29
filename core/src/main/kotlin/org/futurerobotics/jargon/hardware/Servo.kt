package org.futurerobotics.jargon.hardware

/**
 * Represents a servo. Can link directly to hardware.
 */
interface Servo {

    /** Sets the angle of this servo, **in radians**, or gets the previously set value */
    var angle: Double
}
