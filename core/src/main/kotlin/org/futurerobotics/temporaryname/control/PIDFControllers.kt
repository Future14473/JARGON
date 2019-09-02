package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.coerceIn
import org.futurerobotics.temporaryname.math.coerceLengthAtMost
import org.futurerobotics.temporaryname.mechanics.LinearMotionState
import org.futurerobotics.temporaryname.mechanics.component1
import org.futurerobotics.temporaryname.mechanics.component2
import org.futurerobotics.temporaryname.mechanics.component3

/**
 * Simple PIDF [Controller], with some extra bells and whistles defined in [PIDFCoefficients].
 * Setpoint (measurement) is a double, reference is [LinearMotionState], output signal is also a Double in the direction
 * that the measurement needs to move.
 *
 * Considering using the [PIDPassController] family instead, as it passes on velocity/acceleration feedfowards,
 * which may be better used when controllers are chained.
 *
 * @param coefficients the [PIDFCoefficients] to use
 */
class PIDFController(
    private val coefficients: PIDFCoefficients
) : BaseController<LinearMotionState, Double, Double>() {

    private var prevError = 0.0
    private var errorSum = 0.0
    override fun getSignal(reference: LinearMotionState, currentState: Double, elapsedSeconds: Double): Double {
        val (x, v, a) = reference
        val curError = (x - currentState) coerceIn coefficients.errorBounds

        if (elapsedSeconds.isNaN()) {
            prevError = curError
            return 0.0
        }
        val dt = elapsedSeconds

        errorSum = if (curError <= coefficients.integralActivationThreshold) {
            val curI = (curError + prevError) * (dt / 2)
            (this.errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
        } else 0.0
        val p = curError * coefficients.p
        val i = errorSum * coefficients.i
        val d = (curError - prevError) * (coefficients.d / dt)
        val f = v * coefficients.fv + a * coefficients.fa
        prevError = curError
        return (p + i + d + f).coerceIn(coefficients.outputBounds)
    }

    override fun start() {
        stop()
    }

    /**
     * Resets this controller (integral error sum and timing)
     */
    override fun stop() {
        invalidateSignal()
        errorSum = 0.0
        prevError = 0.0
    }
}

/**
 * Experimental PIDF [Controller] for [Vector2d]. see [PIDFController]
 *
 * Considering using the [VecPIDPassController] instead, as it passes on velocity/acceleration feedfowards,
 * which may be better used when controllers are chained.
 *
 * @param coefficients the [PIDFCoefficients] to use
 *
 * @see PIDFController
 */
class VecPIDFController(
    private val coefficients: PIDFCoefficients
) : BaseController<VectorMotionState, Vector2d, Vector2d>() {

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO
    override fun getSignal(reference: VectorMotionState, currentState: Vector2d, elapsedSeconds: Double): Vector2d {
        val (x, v, a) = reference
        val curError = (x - currentState) coerceLengthAtMost coefficients.errorBounds.b

        if (elapsedSeconds.isNaN()) {
            prevError = curError
            return Vector2d.ZERO
        }
        val dt = elapsedSeconds

        errorSum = if (curError.length <= coefficients.integralActivationThreshold) {
            val curI = (curError + prevError) * (dt / 2)
            (this.errorSum + curI).coerceLengthAtMost(coefficients.maxErrorSum)
        } else Vector2d.ZERO
        val p = curError * coefficients.p
        val i = errorSum * coefficients.i
        val d = (curError - prevError) * (coefficients.d / dt)
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
 * Input can be either global or local position, and output is a Pose2d that is supposed to move the robot
 * towards the reference (velocity).
 *
 * @param translationalCoeff the translational PIDF coefficients
 * @param headingCoef the heading PIDF coefficients
 */
class TwoPartPIDFController(
    translationalCoeff: PIDFCoefficients,
    headingCoef: PIDFCoefficients
) : BaseController<PoseMotionState, Pose2d, Pose2d>() {

    private val translational = VecPIDFController(translationalCoeff)
    private val rotational = PIDFController(headingCoef)
    override fun getSignal(reference: PoseMotionState, currentState: Pose2d, elapsedSeconds: Double): Pose2d {
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
 * A PIDF controller for [Pose2d]'s that uses separate [PIDFController]s for axial, lateral, and heading components of a pose.
 *
 * Keep in mind this uses Northwest up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
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
) : BaseController<PoseMotionState, Pose2d, Pose2d>() {

    private val axial = PIDFController(axialCoeff) //x
    private val lateral = PIDFController(lateralCoeff) //y
    private val heading = PIDFController(headingCoeff)
    override fun getSignal(reference: PoseMotionState, currentState: Pose2d, elapsedSeconds: Double): Pose2d {
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