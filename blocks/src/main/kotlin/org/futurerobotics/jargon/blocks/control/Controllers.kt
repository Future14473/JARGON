package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.CompositeBlock
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.blocks.config.BCBuilder
import org.futurerobotics.jargon.blocks.config.BlockConfig
import org.futurerobotics.jargon.blocks.control.FeedForwardWrapper.Companion.withAdder
import org.futurerobotics.jargon.blocks.functional.SplitMotionState
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.ValueMotionOnly

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
interface Controller<Reference, State, Signal> {

    /** The reference input. */
    val reference: Block.Input<Reference>
    /** The state input. */
    val state: Block.Input<State>
    /** The signal output. */
    val signal: Block.Output<Signal>
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
) : SingleOutputBlock<Signal>(LAZY),
    Controller<State, State, Signal> where State : Comparable<State> {

    override val reference: Input<State> = newInput()
    override val state: Input<State> = newInput()
    override val signal: Output<Signal> get() = super.output

    override fun init() {}

    override fun stop() {}

    override fun Context.getOutput(): Signal {
        val comp = state.get.compareTo(reference.get)
        return when {
            comp < 0 -> lessThanOutput
            comp > 0 -> greaterThanOutput
            else -> equalOutput
        }
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
    CompositeBlock(ALWAYS), Controller<MotionState<T>, T, MotionOnly<T>> {

    override val reference: Input<MotionState<T>> = newInput()
    override val state: Input<T> = newInput()
    override val signal: Output<MotionOnly<T>> = newOutput()

    /** Adds t1 and t2 together. Unfortunately `Double` doesn't implement an addable interface or the like.*/
    protected abstract operator fun T.plus(other: T): T

    override fun SubsystemMapper.configSubsystem(): BlockConfig = BCBuilder().build {
        val refInput = reference.subOutput()
        val stateInput = state.subOutput()

        val (refS, refV, refA) = SplitMotionState<T>()() { input from refInput }

        val nonFFSignal = nonFFController.signal
        nonFFController.run { reference from refS; state from stateInput }

        signal.subInput() from generate {
            ValueMotionOnly(
                refV.get + nonFFSignal.get,
                refA.get
            )
        }
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
