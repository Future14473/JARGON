package org.futurerobotics.jargon.hardware

/**
 * Represents a DcMotor. Can link directly to hardware.
 */
interface Motor {

    /**
     * The max voltage this motor can take.
     */
    val maxVoltage: Double

    /**
     * Sets the voltage to this motor, or gets the previously set value.
     */
    var voltage: Double

    /**
     * Gets the motor position **in radians** of this motor, with the same sign direction as [voltage].
     */
    val position: Double

    /**
     * Gets the motor velocity **in radians per second** of this motor, with the same sign direction as [voltage].
     */
    val velocity: Double

    /**
     * Resets the position so that the current position is 0.0. Following [position] values will
     * be relative to the current value.
     */
    fun resetPosition()

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
