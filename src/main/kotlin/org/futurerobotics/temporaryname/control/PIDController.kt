package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.avg
import org.futurerobotics.temporaryname.math.coerceIn

/**
 * Simple PID controller also using other options defined in [PIDCoefficients]
 *
 * @param coefficients the [PIDCoefficients] to use
 * @param clock the [Clock] that I and D coefficients are calculated with.
 */
class PIDController(
    private val coefficients: PIDCoefficients, private val clock: Clock = Clock.Default
) {
    private var hasPrevTime = false
    private var prevTime = clock.nanoTime()
    private var prevError = 0.0
    private var errorSum = 0.0
    /**
     * The setPoint of this PIDController
     */
    var setPoint: Double = 0.0

    /**
     * Updates internal state given the current [input], and returns this PIDController's Output.
     */
    fun getOutput(input: Double = 0.0): Double {

        val curError = setPoint - input.coerceIn(coefficients.inputBounds)
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

        prevError = curError
        prevTime = curTime
        return (p + i + d).coerceIn(coefficients.outputBounds)
    }

    /**
     * Resets this controller (integral error sum and timing)
     */
    fun reset() {
        hasPrevTime = false
        errorSum = 0.0
    }
}