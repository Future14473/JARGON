package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm

/**
 * A PID controller that works with double values.
 *
 * Note: if you are using a this to control heading, see [HeadingPIDController]
 *
 * @see HeadingPIDController
 */
open class PIDController(private val coefficients: PIDCoefficients) : SimpleController<Double, Double, Double> {

    private var prevError = 0.0
    private var errorSum = 0.0

    /**
     * The last outputted signal.
     */
    final override var signal: Double = 0.0
        private set

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
        val error = curError/*.coerceIn(coefficients.errorBounds)*/
        return if (elapsedTime == 0.0) {
            prevError = error
            0.0
        } else {
//            errorSum = if (error <= coefficients.integralActivationThreshold) {
//                val curI = (error + prevError) * (elapsedTime / 2)
//                (errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
//            } else errorSum
            val p = error * coefficients.p
            val i = errorSum * coefficients.i
            val d = (error - prevError) * (coefficients.d / elapsedTime)
            prevError = error
            (p + i + d)/*.coerceIn(coefficients.outputBounds)*/
        }
    }
}

/**
 * A [PIDController] that normalizes the error to between within -Pi to Pi, so it always
 * turns in the closest direction.
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

    override fun update(reference: Pose2d, currentState: Pose2d, elapsedTimeInNanos: Long): Pose2d {
        val (rx, ry, rh) = reference
        val (sx, sy, sh) = currentState
        val x = xController.update(rx, sx, elapsedTimeInNanos)
        val y = yController.update(ry, sy, elapsedTimeInNanos)
        val h = hController.update(rh, sh, elapsedTimeInNanos)
        return Pose2d(x, y, h)
    }
}
