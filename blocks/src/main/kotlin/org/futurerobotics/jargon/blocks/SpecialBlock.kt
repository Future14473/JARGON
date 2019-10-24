package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS

/**
 * Special blocks are treated specially by [BlocksSystem]s.
 *
 * There can only be at most of each kind of special block per system.
 *
 * As of now the only [SpecialBlock] is [Shutdown]
 */
sealed class SpecialBlock(numInputs: Int, numOutputs: Int, processing: Block.Processing) :
    AbstractBlock(numInputs, numOutputs, processing)

/**
 * This [SpecialBlock], when inputted a value of `true`, will tell the system to shutdown.
 */
class Shutdown : SpecialBlock(1, 0, IN_FIRST_ALWAYS), BlocksConfig.Input<Boolean?> {

    /** If this block received shutdown signal or not. */
    var shutDownSignal: Boolean = false
        private set

    override fun init() {
        shutDownSignal = false
    }

    override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        shutDownSignal = inputs[0] as Boolean? == true
    }

    override fun getOutput(index: Int): Any? = throw IndexOutOfBoundsException(index)
    override val block: Block get() = this
    override val index: Int get() = 0
}