package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.coerceLengthAtMost


/**
 * _Experimental_ PID controller that works with [Vector2d] values instead of 1d values, for holonomic drives, so
 * derivative, integral, and output bounds work together.
 *
 * Only the upper bound of [PIDCoefficients.inputBounds] or [PIDCoefficients.outputBounds] will be used.
 *
 * @param coefficients the [PIDCoefficients] to use
 * @param clock the [Clock] that I and D coefficients are calculated with.
 */
class PIDController2d(
    private val coefficients: org.futurerobotics.temporaryname.control.PIDCoefficients,
    private val clock: org.futurerobotics.temporaryname.control.Clock = org.futurerobotics.temporaryname.control.Clock.Default
) {
    private var hasPrevTime = false
    private var prevTime = 0L
    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO
    /**
     * The setPoint of this PIDController, as a [Vector2d]
     */
    var setPoint: Vector2d = Vector2d.ZERO

    /**
     * Updates internal state given the current [input], and returns this PIDController's output, as a Vector2d
     */
    fun getOutput(input: Vector2d = Vector2d.ZERO): Vector2d {
        val input = input.coerceLengthAtMost(coefficients.inputBounds.b)

        val curError = setPoint - input
        val curTime = clock.nanoTime()

        if (!hasPrevTime) {
            prevError = curError
            prevTime = curTime
            hasPrevTime = true
            return Vector2d.ZERO
        }
        val dt = (curTime - prevTime) / 1e9

        errorSum = if (curError.length <= coefficients.integralActivationThreshold) {
            val curI = (curError + prevError) * (dt / 2)
            (this.errorSum + curI).coerceLengthAtMost(coefficients.maxErrorSum)
        } else Vector2d.ZERO

        val p = curError * coefficients.p
        val i = errorSum * coefficients.i
        val d = (curError - prevError) * (coefficients.d / dt)

        prevError = curError
        prevTime = curTime
        return (p + i + d).coerceLengthAtMost(coefficients.outputBounds.b)
    }

    /**
     * Resets this controller (integral error sum and timing)
     */
    fun reset() {
        hasPrevTime = false
        errorSum = Vector2d.ZERO
    }
}