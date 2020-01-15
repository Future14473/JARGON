package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.control.*
import org.futurerobotics.jargon.math.Pose2d

private typealias ToilAndTrouble = Double

/**
 * Common parts of [PIDControllerBlock] and [HeadingPIDControllerBlock]
 */
abstract class BasePIDControllerBlock : PrincipalOutputBlock<Double>(ALWAYS),
                                        ControllerBlock<Double, Double, ToilAndTrouble> {

    override val reference: Input<Double> = newInput()
    override val state: Input<Double> = newInput()
    override val signal: Output<Double> get() = super.output
    /** The inner controller used. */
    protected abstract val controller: PIDController

    override fun init() {
        controller.reset()
    }

    override fun Context.getOutput(): Double {
        val ref = reference.get
        val state = state.get
        return controller.update(ref, state, elapsedTimeInNanos)
    }
}

/**
 * A blocks version of a [PIDController].
 *
 * @see HeadingPIDControllerBlock
 * @see PosePIDControllerBlock
 * @see FeedForwardWrapperBlock
 */
open class PIDControllerBlock(coefficients: PIDCoefficients) : BasePIDControllerBlock() {

    override val controller: PIDController = PIDController(coefficients)
}

/**
 * A blocks version of a [HeadingPIDController].
 *
 * @see PIDControllerBlock
 * @see PosePIDControllerBlock
 * @see FeedForwardWrapperBlock
 */
class HeadingPIDControllerBlock(coefficients: PIDCoefficients) : BasePIDControllerBlock() {

    override val controller: PIDController = HeadingPIDController(coefficients)
}

/**
 * A blocks version of a [PosePIDController].
 *
 * If coefficients are [ExtendedPIDCoefficients], that is also supported.
 *
 * @param xCoeff the axial PID coefficients
 * @param yCoeff the lateral PID coefficients
 * @param hCoeff the heading PID coefficients
 */
class PosePIDControllerBlock(
    xCoeff: PIDCoefficients,
    yCoeff: PIDCoefficients,
    hCoeff: PIDCoefficients
) : PrincipalOutputBlock<Pose2d>(ALWAYS),
    ControllerBlock<Pose2d, Pose2d, Pose2d> {

    override val reference: Input<Pose2d> = newInput()
    override val state: Input<Pose2d> = newInput()
    override val signal: Output<Pose2d> get() = super.output
    private val controller = PosePIDController(xCoeff, yCoeff, hCoeff)

    override fun init() {
        controller.reset()
    }

    override fun Context.getOutput(): Pose2d {
        val ref = reference.get
        val state = state.get
        return controller.update(ref, state, elapsedTimeInNanos)
    }
}
