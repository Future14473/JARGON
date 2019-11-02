package org.futurerobotics.jargon.learning

import org.futurerobotics.jargon.blocks.AbstractBlock
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.SystemValues
import org.futurerobotics.jargon.util.uncheckedCast

/**
 * A block that runs a stochastic update with the given [predictor] and [stochasticFitter]
 * every loop.
 */
class StochasticUpdatingBlock<Input, Output, Pred : Predictor<Input, Output>>(
    private val predictor: Pred, private val stochasticFitter: StochasticFitter<Input, Output, Pred>
) : AbstractBlock(2, 0, IN_FIRST_ALWAYS) {

    /** Where the x/input value is given */
    val valueInput: BlocksConfig.Input<Input> get() = configInput(0)
    /** Where the y/output value is given */
    val valueOutput: BlocksConfig.Input<Input> get() = configInput(1)

    override fun init() {
    }

    override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        val x = inputs[0].uncheckedCast<Input>()
        val y = inputs[1].uncheckedCast<Output>()
        stochasticFitter.stochasticUpdate(predictor, x, y)
    }

    override fun getOutput(index: Int): Any? = throw IndexOutOfBoundsException(index)
}
