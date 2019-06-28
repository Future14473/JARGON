package org.futurerobotics.temporaryname.hardware

interface Motor {
    //set
    /**
     * Sets the motor's power for a value from 0-1. This is a lower-level interface, so sets on actual
     * velocity/acceleration must be handled separately.
     */
    fun setPower(power: Double)

    /**
     * Immediately brakes this motor.
     * Naive implementation simply [setPower] to 0/
     */
    fun brake()
    //get
    /**
     * Gets current angular velocity in _radians per second_
     */
    fun getCurrentAngularVelocity(): Double

    /**
     * Gets current position in RAIDANS
     */
    fun getCurrentPosition(): Double

    /**
     * Resets current recorded position to 0; that is, [getCurrentPosition] should return 0 immediately after
     * calling this.
     */
    fun resetPosition()
}