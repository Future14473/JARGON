package org.futurerobotics.jargon.learning

import org.futurerobotics.jargon.blocks.BaseBlock
import org.futurerobotics.jargon.blocks.Block

/**
 * A block that runs a stochastic update with the given [predictor] and [stochasticFitter]
 * every loop.
 *
 * This will only update if the value given to [inputValue] is not null.
 */
class StochasticUpdatingBlock<Input, Output, Pred : Predictor<Input, Output>>(
    private val predictor: Pred, private val stochasticFitter: StochasticFitter<Input, Output, Pred>
) : BaseBlock(Processing.ALWAYS) {

    /** Where the x/input value is given */
    val inputValue: Block.Input<Input?> = newInput()
    /** Where the y/output value is given */
    val outputValue: Block.Input<Output> = newInput()

    override fun Context.process() {
        val x = inputValue.get ?: return
        val y = outputValue.get
        stochasticFitter.stochasticUpdate(predictor, x, y)
    }
}
