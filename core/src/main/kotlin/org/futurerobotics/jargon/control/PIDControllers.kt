package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.coerceIn

/**
 * A PID controller that works with double values.
 *
 * If the passed coefficient is a [ExtendedPIDCoefficients], that is also supported.
 * @see HeadingPIDController
 */
open class PIDController(coefficients: PIDCoefficients) : SimpleController<Double, Double, Double> {

    private val coefficients = coefficients.toExtendedCoefficients()
    private var prevError = 0.0
    private var errorSum = 0.0
    /**
     * The last outputted signal.
     */
    final override var signal: Double = 0.0
        private set

    /**
     * Resets this pose controller.
     */
    override fun reset() {
        errorSum = 0.0
        prevError = 0.0
        signal = 0.0
    }

    /**
     * Given the [reference] value, [currentState] value, and the [elapsedTimeInNanos],
     * updates the controller and returns the [signal].
     */
    override fun update(reference: Double, currentState: Double, elapsedTimeInNanos: Long): Double {
        val curError = reference - currentState
        return doPID(elapsedTimeInNanos / 1e9, curError)
            .also { signal = it }
    }

    /** Does PID. Room for modification in here. */

    protected open fun doPID(elapsedTime: Double, curError: Double): Double {
        val error = curError.coerceIn(coefficients.errorBounds)
        return if (elapsedTime == 0.0) {
            prevError = error
            0.0
        } else {
            errorSum = if (error <= coefficients.integralActivationThreshold) {
                val curI = (error + prevError) * (elapsedTime / 2)
                (errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
            } else errorSum
            val p = error * coefficients.p
            val i = errorSum * coefficients.i
            val d = (error - prevError) * (coefficients.d / elapsedTime)
            prevError = error
            (p + i + d).coerceIn(coefficients.outputBounds)
        }
    }
}

/**
 * A [PIDController] that doesn't go haywire when it sees a heading error with magnitude greater
 * than PI -- it normalizes error.
 */
class HeadingPIDController(coefficients: PIDCoefficients) : PIDController(coefficients) {

    override fun doPID(elapsedTime: Double, curError: Double): Double = super.doPID(elapsedTime, angleNorm(curError))
}

/**
 * A PID controller for poses that uses separate [PIDController]s for x, y, and heading components of
 * a pose.
 *
 * @param xCoeff the axial PID coefficients
 * @param yCoeff the lateral PID coefficients
 * @param hCoeff the heading PID coefficients
 */
class PosePIDController(xCoeff: PIDCoefficients, yCoeff: PIDCoefficients, hCoeff: PIDCoefficients) :
    SimpleController<Pose2d, Pose2d, Pose2d> {

    private val xController = PIDController(xCoeff)
    private val yController = PIDController(yCoeff)
    private val hController = HeadingPIDController(hCoeff)

    override val signal: Pose2d get() = Pose2d(xController.signal, yController.signal, hController.signal)

    override fun reset() {
        xController.reset()
        yController.reset()
        hController.reset()
    }

    override fun update(reference: Pose2d, currentState: Pose2d, elapsedTimeInNanos: Long): Pose2d {
        val (rx, ry, rh) = reference
        val (sx, sy, sh) = currentState
        val x = xController.update(rx, sx, elapsedTimeInNanos)
        val y = yController.update(ry, sy, elapsedTimeInNanos)
        val h = hController.update(rh, sh, elapsedTimeInNanos)
        return Pose2d(x, y, h)
    }
}
