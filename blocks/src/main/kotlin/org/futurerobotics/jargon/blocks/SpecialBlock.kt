package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS

/**
 * Special blocks are treated specially by [BlockSystem]s.
 *
 * As of now the only [SpecialBlock] is [Shutdown]
 */
sealed class SpecialBlock(processing: Processing) : BaseBlock(processing)

/**
 * This [SpecialBlock], when inputted a value of `true`, will tell the system to shutdown.
 */
class Shutdown : SpecialBlock(ALWAYS) {

    /** Shutdown signal: When received true, tells the system to shut down. */
    val signal: Input<Boolean?> = newInput()
    /** If this block received shutdown signal or not. */
    internal var shutDownSignal: Boolean = false
        private set

    override fun init() {
        shutDownSignal = false
    }

    override fun Context.process() {
        shutDownSignal = signal.get ?: false
    }

    override fun stop() {
        shutDownSignal = false
    }
}
