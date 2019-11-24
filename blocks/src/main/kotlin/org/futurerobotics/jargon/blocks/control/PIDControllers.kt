package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.BlockArrangementBuilder
import org.futurerobotics.jargon.blocks.CompositeBlock
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.blocks.functional.CreatePose
import org.futurerobotics.jargon.blocks.functional.SplitPose
import org.futurerobotics.jargon.math.*

private typealias ToilAndTrouble = Double

/**
 * A PID [Controller] that works with double values, as specified in [PidCoefficients].
 * @see FeedForwardWrapper
 * @see HeadingPidController
 */
open class PidController(private val coefficients: PidCoefficients) : PrincipalOutputBlock<Double>(ALWAYS),
                                                                      Controller<Double, Double, ToilAndTrouble> {

    /** The reference motion input */
    override val reference: Input<Double> = newInput()
    /** The state input */
    override val state: Input<Double> = newInput()

    override val signal: Output<Double> get() = super.output
    private var prevError = 0.0
    private var errorSum = 0.0

    override fun init() {
        errorSum = 0.0
        prevError = 0.0
    }

    override fun Context.getOutput(): Double {
        val ref = reference.get
        val currentState = state.get
        val curError = ref - currentState
        return doPID(loopTime, curError)
    }

    /** Does PID. Room for modification in here. */

    protected open fun doPID(loopTime: Double, curError: Double): Double {
        val error = curError.coerceIn(coefficients.errorBounds)
        return if (loopTime == 0.0) {
            prevError = error
            0.0
        } else {
            errorSum = if (error <= coefficients.integralActivationThreshold) {
                val curI = (error + prevError) * (loopTime / 2)
                (errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
            } else 0.0
            val p = error * coefficients.p
            val i = errorSum * coefficients.i
            val d = (error - prevError) * (coefficients.d / loopTime)
            prevError = error
            (p + i + d).coerceIn(coefficients.outputBounds)
        }
    }
}

/**
 * A [PidController] that doesn't go haywire when it sees a heading error with magnitude greater
 * than PI -- it normalizes error.
 */
class HeadingPidController(coefficients: PidCoefficients) : PidController(coefficients) {

    override fun doPID(loopTime: Double, curError: Double): Double = super.doPID(loopTime, angleNorm(curError))
}

/**
 * A PID [Controller] that works with [Vector2d] values, as specified in [PidCoefficients]. This may provide
 * interesting results.
 *
 * @see FeedForwardWrapper
 */
class VectorPidController(private val coefficients: PidCoefficients) : PrincipalOutputBlock<Vector2d>(ALWAYS),
                                                                       Controller<Vector2d, Vector2d, Vector2d> {

    override val reference: Input<Vector2d> = newInput()
    override val state: Input<Vector2d> = newInput()
    override val signal: Output<Vector2d> get() = super.output

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO

    override fun init() {
        prevError = Vector2d.ZERO
        errorSum = Vector2d.ZERO
    }

    override fun Context.getOutput(): Vector2d {
        val ref = reference.get
        val currentState = state.get
        val curError = (ref - currentState) coerceLengthAtMost coefficients.errorBounds.b
        val loopTime = loopTime
        return if (loopTime == 0.0) {
            prevError = curError
            Vector2d.ZERO
        } else {
            errorSum = if (curError.length <= coefficients.integralActivationThreshold) {
                val curI = (curError + prevError) * (loopTime / 2)
                (errorSum + curI).coerceLengthAtMost(coefficients.maxErrorSum)
            } else Vector2d.ZERO
            val p = curError * coefficients.p
            val i = errorSum * coefficients.i
            val d = (curError - prevError) * (coefficients.d / loopTime)
            prevError = curError
            (p + i + d).coerceLengthAtMost(coefficients.outputBounds.b)
        }
    }
}

/**
 * A PID controller for poses that uses separate [PidController]s for axial, lateral, and heading components of
 * a pose.
 *
 * Input can be _either global or bot_ position,
 * and output is a Pose2d velocity that is supposed to move the state towards the reference
 * _in the same coordinate frame_.
 *
 * Keep in mind this uses North-West-Up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
 * (left/right).
 *
 * @param xCoeff the axial PID coefficients
 * @param yCoeff the lateral PID coefficients
 * @param headingCoeff the heading PID coefficients
 * @see PosePidController
 * @see FeedForwardWrapper
 */
class PosePidController(xCoeff: PidCoefficients, yCoeff: PidCoefficients, headingCoeff: PidCoefficients) :
    CompositeBlock(ALWAYS), Controller<Pose2d, Pose2d, Pose2d> {

    override val reference: Input<Pose2d> = newInput()
    override val state: Input<Pose2d> = newInput()
    override val signal: Output<Pose2d> = newOutput()
    private val xController = PidController(xCoeff) //x
    private val yController = PidController(yCoeff) //y
    private val headingController = HeadingPidController(headingCoeff)

    override fun SubsystemMapper.configSubsystem(builder: BlockArrangementBuilder): Unit = with(builder) {
        val ref = SplitPose().config { input from reference.subOutput }
        val state = SplitPose().config { input from state.subOutput }

        xController.reference from ref.x; xController.state from state.x
        yController.reference from ref.y; yController.state from state.y
        headingController.reference from ref.heading; headingController.state from state.heading

        signal.subInput from CreatePose().config {
            x from xController.signal
            y from yController.signal
            heading from headingController.signal
        }.output
    }
}
