@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.CombineBlock
import org.futurerobotics.jargon.blocks.CompositeBlock
import org.futurerobotics.jargon.blocks.control.FeedForwardController.Companion.withAdder
import org.futurerobotics.jargon.blocks.functional.SplitMotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotionState
import org.futurerobotics.jargon.mechanics.ValueMotionOnly

/**
 * A block that represents a controller, taking in a [Reference], comparing it with [State], and producing a [Signal].
 *
 * This interface is so it can be recognized by other things.
 */
interface Controller<Reference, State, Signal> : Block,
    BlocksConfig.Output<Signal> {

    /** The reference [BlocksConfig.Input] */
    val reference: BlocksConfig.Input<Reference>
    /** The state [BlocksConfig.Input] */
    val state: BlocksConfig.Input<State>
}


/**
 * A controller component that only outputs up to three values depending if the current state is less than, equal to,
 * or greater than the reference.
 *
 * Inputs:
 *  1. reference
 *  2. current state
 *
 * Outputs:
 *  1. the signal
 *
 * @param lessThanOutput the signal to output if the current state is less than the reference
 * @param greaterThanOutput the signal to output if the current state is greater than the reference
 * @param equalOutput the signal to output if the current state is equal to the reference
 */
class BangBangController<State, Signal>(
    private val lessThanOutput: Signal,
    private val greaterThanOutput: Signal,
    private val equalOutput: Signal
) : CombineBlock<State, State, Signal>(IN_FIRST_LAZY),
    Controller<State, State, Signal>
        where State : Comparable<State>, State : Any {

    override val reference: BlocksConfig.Input<State> get() = firstInput
    override val state: BlocksConfig.Input<State> get() = secondInput
    override fun combine(a: State, b: State): Signal {
        val comp = b.compareTo(a)
        return when {
            comp < 0 -> lessThanOutput
            comp > 0 -> greaterThanOutput
            else -> equalOutput
        }
    }
}

/**
 * A open-loop controller for a [FixedDriveModel], that takes the [MotionOnly] of Pose as references,
 * and produces the modeled motor voltages as a list of doubles, using the [model].
 *
 *Inputs:
 *  1. the current pose [MotionOnly]
 *
 * Outputs
 *  1. the modeled motor voltages as a [List] of Doubles
 */
class FixedDriveOpenController(private val model: FixedDriveModel) :
    CombineBlock<MotionOnly<Pose2d>, Any, List<Double>>(IN_FIRST_LAZY),
    Controller<MotionOnly<Pose2d>, Any, List<Double>> {

    override fun combine(a: MotionOnly<Pose2d>, b: Any): List<Double> = model.getModeledVoltages(a)

    override val reference: BlocksConfig.Input<MotionOnly<Pose2d>> get() = firstInput
    /** Do not use; connects to nothing */
    @Deprecated("This doesn't connect to anything", ReplaceWith(""), DeprecationLevel.WARNING)
    override val state: BlocksConfig.Input<Any>
        get() = secondInput
}

/**
 * A controller that wraps another nonFFController that outputs _velocities_ of [T], and turns it into
 * a feed forward controller that takes in [MotionState] as reference and outputs [MotionOnly].
 *
 * It does it by splitting the [MotionState] into its components, passing the value component
 * into the controller, taking the control output, and adding that to the reference velocity to get the final velocity.
 *
 * The acceleration is simply passed on.
 *
 * A function [plus] still needs to be defined. Can also be created directly using [withAdder]
 */
abstract class FeedForwardController<T : Any>(private val nonFFController: Controller<T, T, T>) :
    CompositeBlock(
        2, 1,
        Block.Processing.IN_FIRST_ALWAYS
    ),
    Controller<MotionState<T>, T, MotionOnly<T>> {
    override fun BlocksConfig.buildSubsystem(
        sources: List<BlocksConfig.Output<Any?>>,
        outputs: List<BlocksConfig.Input<Any?>>
    ) {
        val reference = sources[0] as BlocksConfig.Output<MotionState<T>>
        val state = sources[1] as BlocksConfig.Output<T>

        val (refS, refV, refA) = SplitMotionState<T>()() { this from reference }

        nonFFController.let { it.reference from refS; it.state from state }

        val finalV = refV.combine(nonFFController) { this@combine + it }
        outputs[0] from finalV.combine(refA) { ValueMotionOnly(this@combine, it) }
    }

    /** Adds t1 and t2 together. Unfortunately 'Double' doesn't implement an addable interface so we sad. */
    protected abstract operator fun T.plus(other: T): T

    override val reference: BlocksConfig.Input<MotionState<T>> get() = configInput(0)
    override val state: BlocksConfig.Input<T> get() = configInput(1)
    override val block: Block get() = this
    override val index: Int get() = 0

    companion object {
        /**
         * Creates a [FeedForwardController] with the given [adder] function to add.
         */
        @JvmStatic
        inline fun <T : Any> withAdder(
            nonFFController: Controller<T, T, T>,
            crossinline adder: T.(T) -> T
        ): FeedForwardController<T> =
            object : FeedForwardController<T>(nonFFController) {
                override fun T.plus(other: T): T = adder(other)
            }
    }
}