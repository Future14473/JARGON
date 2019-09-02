package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.coerceIn
import org.futurerobotics.temporaryname.math.coerceLengthAtMost
import org.futurerobotics.temporaryname.mechanics.*

/**
 * A controller for [LinearMotionState] that uses PID for positional error,
 * and passes velocity and acceleration feed forwards to its output.
 *
 * The correction is added to the output's velocity.
 *
 * This may be more useful than [PIDPassController], if used in a chained controller system where the next
 * controller handles feed-forwards more intelligently.
 *
 * @see [VecPIDPassController]
 */
class PIDPassController(private val coefficients: PIDCoefficients) :
    BaseController<LinearMotionState, Double, LinearMotion>() {

    private inline val zero get() = 0.0
    private var prevError = zero
    private var errorSum = zero
    override fun getSignal(reference: LinearMotionState, currentState: Double, elapsedSeconds: Double): LinearMotion {
        val (x, v, a) = reference
        val pid = processPID(currentState, elapsedSeconds, x)
        return LinearMotion(v + pid, a)
    }

    private fun processPID(current: Double, elapsedSeconds: Double, x: Double): Double {
        val curError = (x - current) coerceIn coefficients.errorBounds
        return if (elapsedSeconds.isNaN()) {
            prevError = curError
            zero
        } else {
            val dt = elapsedSeconds

            errorSum = if (curError <= coefficients.integralActivationThreshold) {
                val curI = (curError + prevError) * (dt / 2)
                (errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
            } else zero
            val p = curError * coefficients.p
            val i = errorSum * coefficients.i
            val d = (curError - prevError) * (coefficients.d / dt)
            prevError = curError
            p + i + d
        }
    }

    override fun start() {
        stop()
    }

    override fun stop() {
        errorSum = zero
        prevError = zero
    }
}

/**
 * An experimental controller for for [VectorMotionState] that passes velocity and acceleration feed forwards to its output
 * [VectorMotion], and uses PID to correct for positional error which is added to the output's velocity.
 *
 * TODO FINISH DOC
 */
class VecPIDPassController(private val coefficients: PIDCoefficients) :
    BaseController<VectorMotionState, Vector2d, VectorMotion>() {

    private inline val zero get() = Vector2d.ZERO
    private var prevError = zero
    private var errorSum = zero
    override fun getSignal(reference: VectorMotionState, currentState: Vector2d, elapsedSeconds: Double): VectorMotion {
        val (x, v, a) = reference
        val pid = processPID(currentState, elapsedSeconds, x)
        return VectorMotion(v + pid, a)
    }

    private fun processPID(current: Vector2d, elapsedSeconds: Double, x: Vector2d): Vector2d {
        val curError = (x - current) coerceLengthAtMost coefficients.errorBounds.b
        return if (elapsedSeconds.isNaN()) {
            prevError = curError
            zero
        } else {
            val dt = elapsedSeconds

            errorSum = if (curError.length <= coefficients.integralActivationThreshold) {
                val curI = (curError + prevError) * (dt / 2)
                (errorSum + curI) coerceLengthAtMost -coefficients.maxErrorSum
            } else zero
            val p = curError * coefficients.p
            val i = errorSum * coefficients.i
            val d = (curError - prevError) * (coefficients.d / dt)
            prevError = curError
            p + i + d
        }
    }

    override fun start() {
        stop()
    }

    override fun stop() {
        errorSum = zero
        prevError = zero
    }
}

/**
 * A PID controller for [Pose2d]'s that uses a [PIDPassController] for heading, and a [VecPassPIDController] for translation.
 *
 * Input can be either global or local position, and output is a Pose2d that is supposed to move the robot
 * towards the reference (velocity).
 *
 * @param translationalCoeff the translational PID coefficients
 * @param headingCoeff the heading PID coefficients
 */
class TwoPartPIDPassController(
    translationalCoeff: PIDCoefficients,
    headingCoeff: PIDCoefficients
) : BaseController<PoseMotionState, Pose2d, PoseMotion>() {

    private val translational = VecPIDPassController(translationalCoeff)
    private val rotational = PIDPassController(headingCoeff)
    override fun getSignal(reference: PoseMotionState, currentState: Pose2d, elapsedSeconds: Double): PoseMotion {
        return PoseMotion(
            translational.updateAndGetSignal(reference.vec(), currentState.vec, elapsedSeconds),
            rotational.updateAndGetSignal(reference.heading(), currentState.heading, elapsedSeconds)
        )
    }

    override fun start() {
        translational.start()
        rotational.start()
    }

    override fun stop() {
        translational.stop()
        rotational.stop()
    }
}

/**
 * A PID controller for [Pose2d]'s that uses separate [PIDPassController]s for axial, lateral, and heading components of a pose.
 *
 * Keep in mind this uses Northwest up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
 * (left/right).
 *
 * @param axialCoeff the axial PID coefficients
 * @param lateralCoeff the lateral PID coefficients
 * @param headingCoeff the heading PID coefficients
 */
class ThreePartPIDPassController(
    axialCoeff: PIDCoefficients,
    lateralCoeff: PIDCoefficients,
    headingCoeff: PIDCoefficients
) : BaseController<PoseMotionState, Pose2d, PoseMotion>() {

    private val axial = PIDPassController(axialCoeff) //x
    private val lateral = PIDPassController(lateralCoeff) //y
    private val heading = PIDPassController(headingCoeff)
    override fun getSignal(reference: PoseMotionState, currentState: Pose2d, elapsedSeconds: Double): PoseMotion {
        return PoseMotion(
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
        axial.stop()
        lateral.stop()
        heading.stop()
    }
}