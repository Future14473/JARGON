@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.control.FeedForwardWrapper.Companion.withAdder
import org.futurerobotics.jargon.blocks.functional.SplitMotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotionState
import org.futurerobotics.jargon.mechanics.ValueMotionOnly

/**
 * A block that represents a controller, taking in a [Reference], comparing it with [State], and producing a [Signal].
 *
 * Inputs:
 * - [reference]: reference value
 * - [state]: state input
 *
 * Outputs:
 * - `this`: signal output.
 *
 * This interface exists so that it can be recognized by other things.
 */
interface Controller<Reference, State, Signal> : Block, BlocksConfig.Output<Signal> {

    /** The reference input. */
    val reference: BlocksConfig.Input<Reference>
    /** The state input. */
    val state: BlocksConfig.Input<State>
}

/**
 * A [Controller], which only outputs three different values depending on if the state is less than,
 * equal to, or greater than the reference.
 *
 * @param lessThanOutput the signal to output if the current state is less than the reference
 * @param greaterThanOutput the signal to output if the current state is greater than the reference
 * @param equalOutput the signal to output if the current state is equal to the reference
 */
class BangBangController<State, Signal>(
    private val lessThanOutput: Signal, private val greaterThanOutput: Signal, private val equalOutput: Signal
) : CombineBlock<State, State, Signal>(IN_FIRST_LAZY),
    Controller<State, State, Signal> where State : Comparable<State> {

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
 * An open-loop controller for a [FixedDriveModel], that takes in the bot's pose [Motion][MotionOnly]
 * and produces modeled motor voltages as a list of doubles, using the given [model].
 */
class FixedDriveOpenController(private val model: FixedDriveModel) :
    CombineBlock<MotionOnly<Pose2d>, Any, List<Double>>(IN_FIRST_LAZY),
    Controller<MotionOnly<Pose2d>, Any, List<Double>> {

    override val reference: BlocksConfig.Input<MotionOnly<Pose2d>> get() = firstInput
    /** Do not use; connects to nothing */
    @Deprecated("This doesn't connect to anything", ReplaceWith(""), DeprecationLevel.WARNING)
    override val state: BlocksConfig.Input<Any>
        get() = secondInput

    override fun combine(a: MotionOnly<Pose2d>, b: Any): List<Double> = model.getModeledVoltages(a)
    override fun prepareAndVerify(config: BlocksConfig): Unit = config.run {
        if (!inputIsConnected(0)) throw IllegalBlockConfigurationException("All inputs to ${this@FixedDriveOpenController} must be connected.")
    }
}

/**
 * A [Controller] that wraps another [non feed-forward controller][nonFFController], and creates a feed-forward
 * controller that takes in [MotionState] as reference and outputs [MotionOnly].
 *
 * The [non feed-forward controller][nonFFController] should take in values for state and reference, and output
 * _velocities_ of [T].
 *
 * It does it by splitting the [MotionState] into its components, passing the value component into the non
 * feed-forward controller, taking its output, and adding that to the reference velocity to get the final velocity.
 *
 * The acceleration is simply passed on.
 *
 * A [plus] function still needs to be defined. Can also be created directly using [withAdder].
 */
abstract class FeedForwardWrapper<T : Any>(private val nonFFController: Controller<T, T, T>) :
    CompositeBlock(2, 1, IN_FIRST_ALWAYS), Controller<MotionState<T>, T, MotionOnly<T>> {

    override val reference: BlocksConfig.Input<MotionState<T>> get() = configInput(0)
    override val state: BlocksConfig.Input<T> get() = configInput(1)
    override val block: Block get() = this
    override val index: Int get() = 0
    /** Adds t1 and t2 together. Unfortunately `Double` doesn't implement an addable interface or the like.*/
    protected abstract operator fun T.plus(other: T): T

    override fun configSubsystem(
        sources: List<BlocksConfig.Output<Any?>>, outputs: List<BlocksConfig.Input<Any?>>
    ): BlocksConfig = BaseBlocksConfig().apply {
        val reference = sources[0] as BlocksConfig.Output<MotionState<T>>
        val state = sources[1] as BlocksConfig.Output<T>

        val (refS, refV, refA) = SplitMotionState<T>()() { this from reference }

        nonFFController.let { it.reference from refS; it.state from state }
        val finalV = refV.combine(nonFFController) { this@combine + it }
        outputs[0] from finalV.combine(refA) { ValueMotionOnly(this@combine, it) }
    }

    companion object {
        /** Creates a [FeedForwardWrapper] with the given [adder] function to add. */
        @JvmStatic
        inline fun <T : Any> withAdder(
            nonFFController: Controller<T, T, T>, crossinline adder: T.(T) -> T
        ): FeedForwardWrapper<T> = object : FeedForwardWrapper<T>(nonFFController) {
            override fun T.plus(other: T): T = adder(other)
        }
    }
}