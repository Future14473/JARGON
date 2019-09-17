package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.coerceIn
import org.futurerobotics.jargon.math.coerceLengthAtMost
import org.futurerobotics.jargon.mechanics.*

/**
 * A controller for [LinearState] that uses PID for positional error,
 * and passes velocity and acceleration feed forwards to its output.
 *
 * The PID correction is added to the output's velocity.
 *
 * This may be more useful than [PIDPassController], if used in a chained controller system where the next
 * controller handles feed-forwards more intelligently.
 *
 * @see [VecPIDPassController]
 */
class PIDPassController(private val coefficients: PIDCoefficients) :
    BasePassingMotionController<Double>() {

    private inline val zero get() = 0.0

    private var prevError = zero
    private var errorSum = zero

    override fun getSignal(reference: State<Double>, currentState: Double, elapsedSeconds: Double): Motion<Double> {
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
            errorSum = if (curError <= coefficients.integralActivationThreshold) {
                val curI = (curError + prevError) * (elapsedSeconds / 2)
                (errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
            } else zero
            val p = curError * coefficients.p
            val i = errorSum * coefficients.i
            val d = (curError - prevError) * (coefficients.d / elapsedSeconds)
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
 * An experimental controller that works with [Vector2d] values, math is similar to [PIDPassController]
 *
 * The PID correction is added to the output's velocity.
 *
 * This may be more useful than [PIDPassController], if used in a chained controller system where the next
 * controller handles feed-forwards more intelligently.
 *
 * @see [VecPIDPassController]
 */
class VecPIDPassController(private val coefficients: PIDCoefficients) :
    BasePassingMotionController<Vector2d>() {

    private inline val zero get() = Vector2d.ZERO
    private var prevError = zero
    private var errorSum = zero
    override fun getSignal(
        reference: State<Vector2d>,
        currentState: Vector2d,
        elapsedSeconds: Double
    ): Motion<Vector2d> {
        val (x, v, a) = reference
        val pid = processPID(currentState, elapsedSeconds, x)
        return ValueMotion(v + pid, a)
    }

    private fun processPID(current: Vector2d, elapsedSeconds: Double, x: Vector2d): Vector2d {
        val curError = (x - current) coerceLengthAtMost coefficients.errorBounds.b
        return if (elapsedSeconds.isNaN()) {
            prevError = curError
            zero
        } else {

            errorSum = if (curError.length <= coefficients.integralActivationThreshold) {
                val curI = (curError + prevError) * (elapsedSeconds / 2)
                (errorSum + curI) coerceLengthAtMost -coefficients.maxErrorSum
            } else zero
            val p = curError * coefficients.p
            val i = errorSum * coefficients.i
            val d = (curError - prevError) * (coefficients.d / elapsedSeconds)
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
 * A PIDPass controller for [Pose2d]'s that uses a [PIDPassController] for heading, and a [VecPIDPassController] for
 * translation.
 *
 * Input can be either global or local position, and output is a Pose Motion that is supposed to move the state
 * towards the reference (velocity), _in the same coordinate frame_.
 *
 * @param translationalCoeff the translational PID coefficients
 * @param headingCoeff the heading PID coefficients
 */
class TwoPartPIDPassController(
    translationalCoeff: PIDCoefficients,
    headingCoeff: PIDCoefficients
) : BasePassingMotionController<Pose2d>() {

    private val translational = VecPIDPassController(translationalCoeff)
    private val rotational = PIDPassController(headingCoeff)

    override fun getSignal(reference: State<Pose2d>, currentState: Pose2d, elapsedSeconds: Double): Motion<Pose2d> {
        val (vv, va) = translational.updateAndGetSignal(reference.vec(), currentState.vec, elapsedSeconds)
        val (hv, ha) = rotational.updateAndGetSignal(reference.heading(), currentState.heading, elapsedSeconds)
        return ValueMotion(Pose2d(vv, hv), Pose2d(va, ha))
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
 * A PIDPass controller for [Pose2d]'s that uses separate [PIDPassController]s for axial, lateral, and heading components o
 * f a pose.
 *
 * Input can be either global or local position, and output is a Pose Motion that is supposed to move the state
 * towards the reference, _in the same coordinate frame_.
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
) : BasePassingMotionController<Pose2d>() {

    private val axial = PIDPassController(axialCoeff) //x
    private val lateral = PIDPassController(lateralCoeff) //y
    private val heading = PIDPassController(headingCoeff)

    override fun getSignal(reference: State<Pose2d>, currentState: Pose2d, elapsedSeconds: Double): Motion<Pose2d> {
        val (xv, xa) = axial.updateAndGetSignal(reference.x(), currentState.x, elapsedSeconds)
        val (yv, ya) = lateral.updateAndGetSignal(reference.y(), currentState.y, elapsedSeconds)
        val (hv, ha) = heading.updateAndGetSignal(reference.heading(), currentState.heading, elapsedSeconds)
        return ValueMotion(Pose2d(xv, yv, hv), Pose2d(xa, ya, ha))
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