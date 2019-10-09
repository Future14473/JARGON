@file:Suppress("UNCHECKED_CAST", "DuplicatedCode")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.coerceIn
import org.futurerobotics.jargon.math.coerceLengthAtMost
import org.futurerobotics.jargon.mechanics.MotionState3


/**
 * A PIDF controller that works with Doubles, as specified in[PIDFCoefficients]
 *
 * Inputs:
 * 1. reference: a Double value, or a [MotionState2] of Double, or a [MotionState3] of Double.
 * 2. current double value
 *
 * Outputs:
 * 1. Signal
 */
class PIDFController(
    private val coefficients: PIDFCoefficients
) : AbstractBlock(2, 1, IN_FIRST, ALWAYS) {

    private var prevError = 0.0
    private var errorSum = 0.0

    override fun init(outputs: MutableList<Any?>) {
        errorSum = 0.0
        prevError = 0.0
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val reference = inputs[0]!!.castToMotionState3(0.0)
        val currentState = inputs[1] as Double
        val loopTime = inputs[2] as Double
        val (s, v, a) = reference
        val curError = (s - currentState).coerceIn(coefficients.errorBounds)
        outputs[0] = if (loopTime.isNaN()) {
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
 * 1. reference: a [Vector2d] value, or a [MotionState2]/[MotionState3] of [Vector2d] values.
 * 2. the current state
 *
 * Outputs:
 * 1. [Vector2d] signal to drive the current state to the reference.
 */
class VecPIDFController(
    private val coefficients: PIDFCoefficients
) : AbstractBlock(2, 1, IN_FIRST, ALWAYS) {

    private var prevError = Vector2d.ZERO
    private var errorSum = Vector2d.ZERO

    override fun init(outputs: MutableList<Any?>) {
        prevError = Vector2d.ZERO
        errorSum = Vector2d.ZERO
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val reference = inputs[0]!!.castToMotionState3(Vector2d.ZERO)
        val currentState = inputs[1] as Vector2d
        val loopTime = inputs[2] as Double
        val (s, v, a) = reference
        val curError = (s - currentState) coerceLengthAtMost coefficients.errorBounds.b
        outputs[0] = if (loopTime.isNaN()) {
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


//TODO: composite blocks
///**
// * A PIDF controller for [Pose2d]'s that uses a [PIDFController] for heading, and a [VecPIDFController] for translation.
// *
// * Input can be either global or local position, and output is a Pose2d (velocity) that is supposed to move the state
// * towards the reference, _in the same coordinate frame_.
// *
// * @param translationalCoeff the translational PIDF coefficients
// * @param headingCoeff the heading PIDF coefficients
// */
//class TwoPartPIDFController(
//    translationalCoeff: PIDFCoefficients,
//    headingCoeff: PIDFCoefficients
//) : BaseStandardController<Pose2d, Pose2d>() {
//
//    private val translational = VecPIDFController(translationalCoeff)
//    private val rotational = PIDFController(headingCoeff)
//
//    override fun getSignal(reference: MotionState3<Pose2d>, currentState: Pose2d, loopTime: Double): Pose2d {
//        return Pose2d(
//            translational.updateAndGetSignal(reference.vec(), currentState.vec, loopTime),
//            rotational.updateAndGetSignal(reference.heading(), currentState.heading, loopTime)
//        )
//    }
//
//    override fun init() {
//        output = null
//        translational.init()
//        rotational.init()
//    }
//
//}
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
//    override fun getSignal(reference: MotionState3<Pose2d>, currentState: Pose2d, loopTime: Double): Pose2d {
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
