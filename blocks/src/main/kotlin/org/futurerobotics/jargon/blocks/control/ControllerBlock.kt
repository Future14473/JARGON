package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.control.FeedForwardWrapper
import org.futurerobotics.jargon.control.SimpleController
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState

/**
 * A block that represents a controller:
 * taking in a [Reference], comparing it with [State], and producing a [Signal].
 */
interface ControllerBlock<Reference, State, Signal> {

    /** The reference input. */
    val reference: Block.Input<Reference>
    /** The state input. */
    val state: Block.Input<State>
    /** The signal output. */
    val signal: Block.Output<Signal>
}

/**
 * A [ControllerBlock] that wraps around non-blocks [innerController].
 */
open class BaseControllerBlock<Reference, State, Signal>(
    private val innerController: SimpleController<Reference, State, Signal>
) : Block(Processing.ALWAYS), ControllerBlock<Reference, State, Signal> {

    override val reference: Input<Reference> = newInput()
    override val state: Input<State> = newInput()
    override val signal: Output<Signal> = newOutput()
    override fun init() {
        innerController.reset()
    }

    override fun Context.process() {
        signal.set = innerController.update(reference.get, state.get, elapsedTimeInNanos)
    }
}

/**
 * A [ControllerBlock] that wraps around a [FeedForwardWrapper] (see doc there).
 */
open class FeedForwardWrapperBlock<T>(
    velocityController: SimpleController<T, T, T>,
    plus: (T, T) -> T
) : BaseControllerBlock<MotionState<T>, T, MotionOnly<T>>(
    FeedForwardWrapper<T>(velocityController, plus)
)
