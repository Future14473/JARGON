@file:Suppress("UNCHECKED_CAST", "DuplicatedCode")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.coerceIn
import org.futurerobotics.jargon.math.coerceLengthAtMost
import org.futurerobotics.jargon.mechanics.MotionState


/**
 * A PIDF controller that works with Doubles, as specified in [PIDFCoefficients]
 *
 * Inputs:
 * 1. reference: [MotionState] of Double.
 * 2. current state: Double value
 *
 * Outputs:
 * 1. Signal: Double
 */
class PIDFController(
    private val coefficients: PIDFCoefficients
) : SingleOutputBlock<Double>(2, IN_FIRST_ALWAYS) {

    private var prevError = 0.0
    private var errorSum = 0.0

    /** The reference motion [BlockInput] */
    val reference: BlockInput<MotionState<Double>> = inputIndex(0)
    /** The state [BlockInput] */
    val state: BlockInput<Double> = inputIndex(1)

    override fun doInit(): Double? {
        errorSum = 0.0
        prevError = 0.0
        return null
    }

    override fun getOutput(inputs: List<Any?>): Double {
        val reference = inputs[0] as MotionState<Double>
        val currentState = inputs[1] as Double
        val loopTime = inputs[2] as Double
        val (s, v, a) = reference
        val curError = (s - currentState).coerceIn(coefficients.errorBounds)
        return if (loopTime.isNaN()) {
            prevError = curError
            0.0
        } else {

            errorSum = if (curError <= coefficients.integralActivationThreshold) {
                val curI = (curError + prevError) * (loopTime / 2)
                (errorSum + curI).coerceIn(-coefficients.maxErrorSum, coefficients.maxErrorSum)
            } else 0.0

            val p = curError * coefficients.p
            val i = errorSum * coefficients.i
            val d = (curError - prevError) * (coefficients.d / loopTime)
            val f = v * coefficients.fv + a * coefficients.fa
            prevError = curError
            (p + i + d + f).coerceIn(coefficients.outputBounds)
        }
    }
}


/**
 * A PIDF controller that works with [Vector2d] values, specified in [PIDFCoefficients]
 *
 * Inputs:
 * 1. reference: [MotionState] of [Vector2d].
 * 2. current state: Vector2d
 *
 * Outputs:
 * 1. Signal: [Vector2d].
 */
class VecPIDFController(
    private val coefficients: PIDFCoefficients
) : SingleOutputBlock<Vector2d>(2, IN_FIRST_ALWAYS) {

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO

    /** The reference motion [BlockInput] */
    val reference: BlockInput<MotionState<Vector2d>> = inputIndex(0)
    /** The state [BlockInput] */
    val state: BlockInput<Vector2d> = inputIndex(1)

    override fun doInit(): Vector2d? {
        prevError = Vector2d.ZERO
        errorSum = Vector2d.ZERO
        return null
    }

    override fun getOutput(inputs: List<Any?>): Vector2d {
        val reference = inputs[0] as MotionState<Vector2d>
        val currentState = inputs[1] as Vector2d
        val loopTime = inputs[2] as Double
        val (s, v, a) = reference
        val curError = (s - currentState) coerceLengthAtMost coefficients.errorBounds.b
        return if (loopTime.isNaN()) {
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
            val f = v * coefficients.fv + a * coefficients.fa
            prevError = curError
            (p + i + d + f).coerceLengthAtMost(coefficients.outputBounds.b)
        }
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
 * @param xCoeff the axial PIDF coefficients
 * @param yCoeff the lateral PIDF coefficients
 * @param headingCoeff the heading PIDF coefficients
 */
class PosePIDFController(
    xCoeff: PIDFCoefficients,
    yCoeff: PIDFCoefficients,
    headingCoeff: PIDFCoefficients
) : CompositeBlock(2, 1, IN_FIRST_ALWAYS), BlockOutput<Pose2d> {

    private val axial = PIDFController(xCoeff) //x
    private val lateral = PIDFController(yCoeff) //y
    private val heading = PIDFController(headingCoeff)

    /** Reference pose MotionState [BlockInput] */
    val reference: BlockInput<MotionState<Pose2d>> get() = inputIndex(0)
    /** State pose [BlockInput] */
    val state: BlockInput<Pose2d> get() = inputIndex(1)

    override fun BlocksConfig.buildSubsystem(sources: List<BlockOutput<Any?>>, outputs: List<BlockInput<Any?>>) {
        val ref = SplitPoseMotionState().apply { connectFrom(sources[0] as BlockOutput<MotionState<Pose2d>>) }
        val state = SplitPose().apply { connectFrom(sources[1] as BlockOutput<Pose2d>) }

        axial.reference connectFrom ref.x; axial.state connectFrom state.x
        lateral.reference connectFrom ref.y; lateral.state connectFrom state.y
        heading.reference connectFrom ref.heading; heading.state connectFrom state.heading

        outputs[0] connectFrom CreatePoseFromComp().apply {
            x connectFrom axial
            y connectFrom lateral
            heading connectFrom this@PosePIDFController.heading
        }
    }

    override fun init() {
        axial.init()
        lateral.init()
        heading.init()
    }

    override val block: Block get() = this
    override val outputIndex: Int get() = 0
}
