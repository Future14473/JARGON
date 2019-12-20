package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.BlockArrangement
import org.futurerobotics.jargon.blocks.BlockArrangementBuilder
import org.futurerobotics.jargon.blocks.CompositeBlock
import org.futurerobotics.jargon.blocks.control.FeedForwardWrapper.Companion.withAdder
import org.futurerobotics.jargon.blocks.functional.SplitMotionState
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState

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
    CompositeBlock(Processing.ALWAYS),
    Controller<MotionState<T>, T, MotionOnly<T>> {

    override val reference: Input<MotionState<T>> = newInput()
    override val state: Input<T> = newInput()
    override val signal: Output<MotionOnly<T>> = newOutput()

    /** Adds t1 and t2 together. Unfortunately `Double` doesn't implement an addable interface or the like.*/
    protected abstract operator fun T.plus(other: T): T

    override fun SubsystemMapper.configSubsystem(): BlockArrangement = BlockArrangementBuilder().build {
        val refInput = reference.subOutput
        val stateInput = state.subOutput

        val (refS, refV, refA) = SplitMotionState<T>().apply { input from refInput }

        val nonFFSignal = nonFFController.signal
        nonFFController.run { reference from refS; state from stateInput }

        signal.subInput from generate {
            MotionOnly(
                refV.get + nonFFSignal.get,
                refA.get
            )
        }
    }

    override fun toString(): String = "FeedForwardWrapper{$nonFFController}"

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
