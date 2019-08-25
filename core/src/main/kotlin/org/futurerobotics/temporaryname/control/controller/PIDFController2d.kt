package org.futurerobotics.temporaryname.control.controller

import org.futurerobotics.temporaryname.control.Clock
import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.coerceLengthAtMost

/**
 * _Experimental_ PID [Controller] that works with [Vector2d] values instead of 1d values, for holonomic drives, so
 * derivative, integral, and output bounds work together.
 *
 * Only the upper bound of [PIDFCoefficients.inputBounds] or [PIDFCoefficients.outputBounds] will be used.
 *
 * @param coefficients the [PIDFCoefficients] to use
 * @param clock the [Clock] that I and D coefficients are calculated with.
 */
class PIDFController2d(
    private val coefficients: PIDFCoefficients, private val clock: Clock = Clock.Default
) : Controller<Vector2d, Vector2d, Vector2d> {
    private var hasPrevTime = false

    private var prevTime = 0L
    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO

    override fun process(
        current: Vector2d,
        reference: Vector2d,
        referenceDeriv: Vector2d?,
        elapsedNanos: Long
    ): Vector2d {

        val curError = reference - current.coerceLengthAtMost(coefficients.inputBounds.b)
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
        val f = referenceDeriv?.let { it * coefficients.f } ?: Vector2d.ZERO
        prevError = curError
        prevTime = curTime
        return (p + i + d + f).coerceLengthAtMost(coefficients.outputBounds.b)
    }

    /**
     * Resets this controller (integral error sum and timing)
     */
    override fun reset() {
        hasPrevTime = false
        errorSum = Vector2d.ZERO
    }
}