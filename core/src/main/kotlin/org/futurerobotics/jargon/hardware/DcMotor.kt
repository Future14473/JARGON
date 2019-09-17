package org.futurerobotics.jargon.hardware

import org.futurerobotics.jargon.system.StartStoppable

/**
 * Represents a DcMotor. Links directly to hardware (or simulation).
 */
interface DcMotor : StartStoppable {

    /**
     * The max voltage this motor can take.
     */
    val maxVoltage: Double

    /**
     * Powers the motor with the supplied [volts] normalizing in bounds if necessary.
     */
    fun setVoltage(volts: Double)

    /**
     * Gets the position ***in radians*** of this motor; with the same sign direction as [setVoltage]
     */
    fun getPosition(): Double

    /**
     * Resets the position so that the current position is 0.0; Following [getPosition] calls will be relative
     * to this position.
     */
    fun resetPosition()

    /**
     * Performs any necessary configurations on this motor before start.
     */
    override fun start()

    /**
     * Stops this motor.
     */
    override fun stop() {
        setVoltage(0.0)
    }
}