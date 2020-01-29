package org.futurerobotics.jargon.hardware

/**
 * Represents a DcMotor. Can link directly to hardware.
 */
interface Motor {

    /**
     * The max voltage this motor can take. Often 12.
     */
    val maxVoltage: Double

    /**
     * Sets the voltage to this motor, or gets the previously set value.
     */
    var voltage: Double

    /**
     * Gets the motor position **in radians** of this motor, in the same direction as [voltage].
     */
    val angle: Double

    /**
     * Gets the motor velocity **in radians per second** of this motor, in the same direction as [voltage].
     */
    val velocity: Double

    /**
     * Resets the angle measurement so that the current angle is 0.0. Following [angle] values will
     * be relative to the current value.
     */
    fun resetAngle()

    /**
     * Performs any necessary configurations on this motor before start.
     */
    fun init()

    /**
     * Stops the motor.
     */
    fun stop() {
        voltage = 0.0
    }
}
