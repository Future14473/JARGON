package org.futurerobotics.temporaryname.control.controller

import org.futurerobotics.temporaryname.control.Clock
import org.futurerobotics.temporaryname.math.avg
import org.futurerobotics.temporaryname.math.coerceIn

/**
 * Simple PID [Controller], with some extra bells and whistles defined in [PIDFCoefficients]
 *
 * @param coefficients the [PIDFCoefficients] to use
 * @param clock the [Clock] that I and D coefficients are calculated with.
 */
class PIDFController(
    private val coefficients: PIDFCoefficients, private val clock: Clock = Clock.Default
) : Controller<Double, Double, Double> {
    private var hasPrevTime = false

    private var prevTime = clock.nanoTime()
    private var prevError = 0.0
    private var errorSum = 0.0

    override fun process(
        current: Double,
        reference: Double,
        referenceDeriv: Double?,
        elapsedNanos: Long
    ): Double {
        val curError = reference - current.coerceIn(coefficients.inputBounds)
        val curTime = clock.nanoTime()

        if (!hasPrevTime) {
            prevError = curError
            prevTime = curTime
            hasPrevTime = true
            return 0.0
        }
        val dt = (curTime - prevTime) / 1e9

        errorSum = if (curError <= coefficients.integralActivationThreshold) {
            val curI = avg(curError, prevError) * dt
            (this.errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
        } else 0.0

        val p = curError * coefficients.p
        val i = errorSum * coefficients.i
        val d = (curError - prevError) * (coefficients.d / dt)
        val f = referenceDeriv?.let { it * coefficients.f } ?: 0.0
        prevError = curError
        prevTime = curTime
        return (p + i + d + f).coerceIn(coefficients.outputBounds)
    }

    /**
     * Resets this controller (integral error sum and timing)
     */
    override fun reset() {
        hasPrevTime = false
        errorSum = 0.0
    }
}