package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.coerceIn
import org.futurerobotics.jargon.math.coerceLengthAtMost
import org.futurerobotics.jargon.mechanics.*

/**
 * Simple PIDF [Controller], with some extra bells and whistles defined in [PIDFCoefficients].
 * Setpoint (measurement) is a Double, reference is [MotionState3]<Double>, output signal is also a Double with the same
 * sign as the direction the measurement needs to move.
 *
 * Considering using the [PIDPassController] family instead, as it passes on velocity/acceleration feedforwards,
 * which may be better used when controllers are chained.
 *
 * @param coefficients the [PIDFCoefficients] to use
 */
class PIDFController(
    private val coefficients: PIDFCoefficients
) : BaseStandardController<Double, Double>() {

    private var prevError = 0.0
    private var errorSum = 0.0

    override fun getSignal(reference: MotionState3<Double>, currentState: Double, elapsedSeconds: Double): Double {
        val (s, v, a) = reference
        val curError = (s - currentState) coerceIn coefficients.errorBounds

        if (elapsedSeconds.isNaN()) {
            prevError = curError
            return 0.0
        }

        errorSum = if (curError <= coefficients.integralActivationThreshold) {
            val curI = (curError + prevError) * (elapsedSeconds / 2)
            (this.errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
        } else 0.0

        val p = curError * coefficients.p
        val i = errorSum * coefficients.i
        val d = (curError - prevError) * (coefficients.d / elapsedSeconds)
        val f = v * coefficients.fv + a * coefficients.fa
        prevError = curError
        return (p + i + d + f).coerceIn(coefficients.outputBounds)
    }

    override fun start() {
        invalidateSignal()
    }

    override fun stop() {
        errorSum = 0.0
        prevError = 0.0
    }
}

/**
 * Experimental PIDF [Controller] that works with [Vector2d] values. Math is similar to [PIDFController]
 *
 * Considering using the [VecPIDPassController] instead, as it passes on velocity/acceleration feedforwards,
 * which may be better used when controllers are chained.
 *
 * @param coefficients the [PIDFCoefficients] to use
 * @see PIDFController
 */
class VecPIDFController(
    private val coefficients: PIDFCoefficients
) : BaseStandardController<Vector2d, Vector2d>() {

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO
    override fun getSignal(
        reference: MotionState3<Vector2d>,
        currentState: Vector2d,
        elapsedSeconds: Double
    ): Vector2d {
        val (s, v, a) = reference
        val curError = (s - currentState) coerceLengthAtMost coefficients.errorBounds.b

        if (elapsedSeconds.isNaN()) {
            prevError = curError
            return Vector2d.ZERO
        }

        errorSum = if (curError.length <= coefficients.integralActivationThreshold) {
            val curI = (curError + prevError) * (elapsedSeconds / 2)
            (this.errorSum + curI).coerceLengthAtMost(coefficients.maxErrorSum)
        } else Vector2d.ZERO

        val p = curError * coefficients.p
        val i = errorSum * coefficients.i
        val d = (curError - prevError) * (coefficients.d / elapsedSeconds)
        val f = v * coefficients.fv + a * coefficients.fa
        prevError = curError
        return (p + i + d + f).coerceLengthAtMost(coefficients.outputBounds.b)
    }

    override fun start() {
        stop()
    }

    /**
     * Resets this controller (integral error sum and timing)
     */
    override fun stop() {
        invalidateSignal()
        errorSum = Vector2d.ZERO
        prevError = Vector2d.ZERO
    }
}

/**
 * A PIDF controller for [Pose2d]'s that uses a [PIDFController] for heading, and a [VecPIDFController] for translation.
 *
 * Input can be either global or local position, and output is a Pose2d (velocity) that is supposed to move the state
 * towards the reference, _in the same coordinate frame_.
 *
 * @param translationalCoeff the translational PIDF coefficients
 * @param headingCoeff the heading PIDF coefficients
 */
class TwoPartPIDFController(
    translationalCoeff: PIDFCoefficients,
    headingCoeff: PIDFCoefficients
) : BaseStandardController<Pose2d, Pose2d>() {

    private val translational = VecPIDFController(translationalCoeff)
    private val rotational = PIDFController(headingCoeff)

    override fun getSignal(reference: MotionState3<Pose2d>, currentState: Pose2d, elapsedSeconds: Double): Pose2d {
        return Pose2d(
            translational.updateAndGetSignal(reference.vec(), currentState.vec, elapsedSeconds),
            rotational.updateAndGetSignal(reference.heading(), currentState.heading, elapsedSeconds)
        )
    }

    override fun start() {
        translational.start()
        rotational.start()
    }

    override fun stop() {
        invalidateSignal()
        translational.stop()
        rotational.stop()
    }
}


/**
 * A PIDF controller for [Pose2d]'s that uses separate [PIDFController]s for axial, lateral, and heading components of
 * a pose.
 *
 * Input can be either global or local position, and output is a Pose2d (velocity) that is supposed to move the state
 * towards the reference, _in the same coordinate frame_.
 *
 * Keep in mind this uses North-West-Up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
 * (left/right).
 *
 * @param axialCoeff the axial PIDF coefficients
 * @param lateralCoeff the lateral PIDF coefficients
 * @param headingCoeff the heading PIDF coefficients
 */
class ThreePartPIDFController(
    axialCoeff: PIDFCoefficients,
    lateralCoeff: PIDFCoefficients,
    headingCoeff: PIDFCoefficients
) : BaseStandardController<Pose2d, Pose2d>() {

    private val axial = PIDFController(axialCoeff) //x
    private val lateral = PIDFController(lateralCoeff) //y
    private val heading = PIDFController(headingCoeff)

    override fun getSignal(reference: MotionState3<Pose2d>, currentState: Pose2d, elapsedSeconds: Double): Pose2d {
        return Pose2d(
            axial.updateAndGetSignal(reference.x(), currentState.x, elapsedSeconds),
            lateral.updateAndGetSignal(reference.y(), currentState.y, elapsedSeconds),
            heading.updateAndGetSignal(reference.heading(), currentState.heading, elapsedSeconds)
        )
    }

    override fun start() {
        axial.start()
        lateral.start()
        heading.start()
    }

    override fun stop() {
        invalidateSignal()
        axial.stop()
        lateral.stop()
        heading.stop()
    }
}
