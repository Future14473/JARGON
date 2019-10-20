@file:Suppress("UNCHECKED_CAST", "DuplicatedCode")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.functional.CreatePoseFromComp
import org.futurerobotics.jargon.blocks.functional.SplitPose
import org.futurerobotics.jargon.blocks.functional.SplitPoseMotionState
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.mechanics.MotionState

private typealias ToilAndTrouble = Double

/**
 * A PID [Controller] that works with Doubles as a reference, as specified in [PIDCoefficients].
 *
 * Inputs:
 * 1. reference: Double value
 * 2. state: Double value
 *
 * Outputs:
 * 1. Signal: Double
 * @see PIDFController
 * @see FeedForwardController
 * @see HeadingPIDController
 */
open class PIDController(
    private val coefficients: PIDCoefficients
) : SingleOutputBlock<Double>(2, IN_FIRST_ALWAYS),
    Controller<Double, Double, ToilAndTrouble> {

    private var prevError = 0.0
    private var errorSum = 0.0


    override fun doInit(): Double? {
        errorSum = 0.0
        prevError = 0.0
        return null
    }

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Double {
        val ref = inputs[0] as Double
        val currentState = inputs[1] as Double
        val curError = ref - currentState

        val loopTime = systemValues.loopTime
        return doPID(loopTime, curError)
    }

    /** Does the PID. Room for modification in here. */
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

    /** The reference motion [BlocksConfig.Input] */
    override val reference: BlocksConfig.Input<Double> get() = configInput(0)
    /** The state [BlocksConfig.Input] */
    override val state: BlocksConfig.Input<Double> get() = configInput(1)

}

/**
 * A [PIDController] that doesn't go haywire when it sees different heading that are of a magnitude greater
 * than PI -- it normalizes error.
 */
class HeadingPIDController(coefficients: PIDCoefficients) : PIDController(coefficients) {
    override fun doPID(loopTime: Double, curError: Double): Double = super.doPID(loopTime, angleNorm(curError))
}

private typealias ToilAndTrector2d = Vector2d

/**
 * A PID [Controller] that works with [Vector2d] values, specified in [PIDCoefficients]. This may provide
 * interesting results.
 *
 * Inputs:
 * 1. reference: [Vector2d] value
 * 2. state: [Vector2d] value
 *
 * Outputs:
 * 1. Signal: [Vector2d].
 * @see VecPIDFController
 * @see FeedForwardController
 */
class VecPIDController(
    private val coefficients: PIDFCoefficients
) : SingleOutputBlock<Vector2d>(2, IN_FIRST_ALWAYS),
    Controller<Vector2d, Vector2d, ToilAndTrector2d> {

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO

    override fun doInit(): Vector2d? {
        prevError = Vector2d.ZERO
        errorSum = Vector2d.ZERO
        return null
    }

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Vector2d {
        val ref = inputs[0] as Vector2d
        val currentState = inputs[1] as Vector2d
        val loopTime = inputs[2] as Double

        val curError = (ref - currentState) coerceLengthAtMost coefficients.errorBounds.b
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

    /** The reference motion [BlocksConfig.Input] */
    override val reference: BlocksConfig.Input<Vector2d> = configInput(0)
    /** The state [BlocksConfig.Input] */
    override val state: BlocksConfig.Input<Vector2d> = configInput(1)
}


/**
 * A PID controller for [Pose2d]'s that uses separate [PIDController]s for axial, lateral, and heading components of
 * a pose.
 *
 * Input can be either global or bot position (behavior will be slightly different),
 * and output is a Pose2d (velocity) that is supposed to move the state towards the reference
 * _in the same coordinate frame_.
 *
 * Keep in mind this uses North-West-Up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
 * (left/right).
 *
 * @param xCoeff the axial PID coefficients
 * @param yCoeff the lateral PID coefficients
 * @param headingCoeff the heading PID coefficients
 * @see PosePIDFController
 * @see FeedForwardController
 */
class PosePIDController(
    xCoeff: PIDCoefficients,
    yCoeff: PIDCoefficients,
    headingCoeff: PIDCoefficients
) : CompositeBlock(2, 1, IN_FIRST_ALWAYS),
    Controller<Pose2d, Pose2d, Pose2d> {

    private val xController = PIDController(xCoeff) //x
    private val yController = PIDController(yCoeff) //y
    private val headingController = HeadingPIDController(headingCoeff)


    override fun BlocksConfig.buildSubsystem(
        sources: List<BlocksConfig.Output<Any?>>,
        outputs: List<BlocksConfig.Input<Any?>>
    ) {
        val ref = SplitPose()
            .apply { from(sources[0] as BlocksConfig.Output<Pose2d>) }
        val state = SplitPose()
            .apply { from(sources[1] as BlocksConfig.Output<Pose2d>) }

        xController.reference from ref.x; xController.state from state.x
        yController.reference from ref.y; yController.state from state.y
        headingController.reference from ref.heading; headingController.state from state.heading

        outputs[0] from CreatePoseFromComp().apply {
            x from xController
            y from yController
            heading from headingController
        }
    }

    override fun init() {
        xController.init()
        yController.init()
        headingController.init()
    }

    /** The reference motion [BlocksConfig.Input] */
    override val reference: BlocksConfig.Input<Pose2d> = configInput(0)
    /** The state [BlocksConfig.Input] */
    override val state: BlocksConfig.Input<Pose2d> = configInput(1)

    override val block: Block get() = this
    override val index: Int get() = 0
}

/**
 * A PIDF controller that works with Doubles, as specified in [PIDFCoefficients]
 *
 * Inputs:
 * 1. reference: [MotionState] of Double.
 * 2. state: Double value
 *
 * Outputs:
 * 1. Signal: Double
 * @see PIDController
 * @see FeedForwardController
 * @see HeadingPIDFController
 */
open class PIDFController(
    private val coefficients: PIDFCoefficients
) : SingleOutputBlock<Double>(2, IN_FIRST_ALWAYS),
    Controller<MotionState<Double>, Double, Double> {

    private var prevError = 0.0
    private var errorSum = 0.0


    override fun doInit(): Double? {
        errorSum = 0.0
        prevError = 0.0
        return null
    }

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Double {
        val ref = inputs[0] as MotionState<Double>
        val currentState = inputs[1] as Double
        val loopTime = inputs[2] as Double
        val (s, v, a) = ref
        val curError = s - currentState
        return doPID(loopTime, curError, v, a)
    }

    /** Does the PID. Room for modification in here. */
    protected open fun doPID(loopTime: Double, curError: Double, v: Double, a: Double): Double {
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
            val f = v * coefficients.fv + a * coefficients.fa
            prevError = error
            (p + i + d + f).coerceIn(coefficients.outputBounds)
        }
    }

    /** The reference motion [BlocksConfig.Input] */
    override val reference: BlocksConfig.Input<MotionState<Double>> get() = configInput(0)
    /** The state [BlocksConfig.Input] */
    override val state: BlocksConfig.Input<Double> get() = configInput(1)

}

/**
 * A [PIDFController] that doesn't go haywire when it sees different heading that are of a magnitude greater
 * than PI -- it normalizes error.
 */
class HeadingPIDFController(coefficients: PIDFCoefficients) : PIDFController(coefficients) {
    override fun doPID(loopTime: Double, curError: Double, v: Double, a: Double): Double =
        super.doPID(loopTime, angleNorm(curError), v, a)
}


/**
 * A PID**F** controller that works with [Vector2d] values, specified in [PIDFCoefficients]
 *
 * Inputs:
 * 1. reference: [MotionState] of [Vector2d].
 * 2. state: Vector2d
 *
 * Outputs:
 * 1. Signal: [Vector2d].
 * @see VecPIDController
 * @see FeedForwardController
 */
class VecPIDFController(
    private val coefficients: PIDFCoefficients
) : SingleOutputBlock<Vector2d>(2, IN_FIRST_ALWAYS),
    Controller<MotionState<Vector2d>, Vector2d, Vector2d> {

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO

    override fun doInit(): Vector2d? {
        prevError = Vector2d.ZERO
        errorSum = Vector2d.ZERO
        return null
    }

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Vector2d {
        val ref = inputs[0] as MotionState<Vector2d>
        val currentState = inputs[1] as Vector2d
        val loopTime = inputs[2] as Double
        val (s, v, a) = ref
        val curError = (s - currentState) coerceLengthAtMost coefficients.errorBounds.b
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
            val f = v * coefficients.fv + a * coefficients.fa
            prevError = curError
            (p + i + d + f).coerceLengthAtMost(coefficients.outputBounds.b)
        }
    }

    /** The reference motion [BlocksConfig.Input] */
    override val reference: BlocksConfig.Input<MotionState<Vector2d>> = configInput(0)
    /** The state [BlocksConfig.Input] */
    override val state: BlocksConfig.Input<Vector2d> = configInput(1)
}


/**
 * A PIDF controller for [Pose2d]'s that uses separate [PIDFController]s for axial, lateral, and heading components of
 * a pose.
 *
 * Input can be either global or bot reference, and output is a Pose2d (velocity) that is supposed to move the state
 * towards the reference, _in the same coordinate frame_.
 *
 * Keep in mind this uses North-West-Up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
 * (left/right).
 *
 * @param xCoeff the axial PIDF coefficients
 * @param yCoeff the lateral PIDF coefficients
 * @param headingCoeff the heading PIDF coefficients
 * @see PosePIDController
 * @see FeedForwardController
 */
class PosePIDFController(
    xCoeff: PIDFCoefficients,
    yCoeff: PIDFCoefficients,
    headingCoeff: PIDFCoefficients
) : CompositeBlock(2, 1, IN_FIRST_ALWAYS),
    Controller<MotionState<Pose2d>, Pose2d, Pose2d> {

    private val xController = PIDFController(xCoeff) //x
    private val yController = PIDFController(yCoeff) //y
    private val headingController = HeadingPIDFController(headingCoeff)


    override fun BlocksConfig.buildSubsystem(
        sources: List<BlocksConfig.Output<Any?>>,
        outputs: List<BlocksConfig.Input<Any?>>
    ) {
        val ref = SplitPoseMotionState()
            .apply { from(sources[0] as BlocksConfig.Output<MotionState<Pose2d>>) }
        val state = SplitPose()
            .apply { from(sources[1] as BlocksConfig.Output<Pose2d>) }

        xController.reference from ref.x; xController.state from state.x
        yController.reference from ref.y; yController.state from state.y
        headingController.reference from ref.heading; headingController.state from state.heading

        outputs[0] from CreatePoseFromComp().apply {
            x from xController
            y from yController
            heading from headingController
        }
    }

    override fun init() {
        xController.init()
        yController.init()
        headingController.init()
    }

    /** The reference motion [BlocksConfig.Input] */
    override val reference: BlocksConfig.Input<MotionState<Pose2d>> = configInput(0)
    /** The state [BlocksConfig.Input] */
    override val state: BlocksConfig.Input<Pose2d> = configInput(1)

    override val block: Block get() = this
    override val index: Int get() = 0
}
