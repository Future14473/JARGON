@file:Suppress("UNCHECKED_CAST", "DuplicatedCode")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
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
//
//
///**
// * A PIDF controller for [Pose2d]'s that uses separate [PIDFController]s for axial, lateral, and heading components of
// * a pose.
// *
// * Input can be either global or local position, and output is a Pose2d (velocity) that is supposed to move the state
// * towards the reference, _in the same coordinate frame_.
// *
// * Keep in mind this uses North-West-Up orientation, so axial is the x-axis (up/down) and lateral is the y-axis
// * (left/right).
// *
// * @param axialCoeff the axial PIDF coefficients
// * @param lateralCoeff the lateral PIDF coefficients
// * @param headingCoeff the heading PIDF coefficients
// */
//class ThreePartPIDFController(
//    axialCoeff: PIDFCoefficients,
//    lateralCoeff: PIDFCoefficients,
//    headingCoeff: PIDFCoefficients
//) : BaseStandardController<Pose2d, Pose2d>() {
//
//    private val axial = PIDFController(axialCoeff) //x
//    private val lateral = PIDFController(lateralCoeff) //y
//    private val heading = PIDFController(headingCoeff)
//
//    override fun getSignal(reference: MotionState<Pose2d>, currentState: Pose2d, loopTime: Double): Pose2d {
//        return Pose2d(
//            axial.updateAndGetSignal(reference.x(), currentState.x, loopTime),
//            lateral.updateAndGetSignal(reference.y(), currentState.y, loopTime),
//            heading.updateAndGetSignal(reference.heading(), currentState.heading, loopTime)
//        )
//    }
//
//    override fun init() {
//        axial.init()
//        lateral.init()
//        heading.init()
//        output = null
//    }
//}
