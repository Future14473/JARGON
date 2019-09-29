package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.control.Block.Processing.LAZY


/**
 * The common interface implemented by special components.
 *
 * A special component has a special function in a [BlockSystem], including:
 *
 *  - Determining when to turn off the system
 *  - Information about loop time
 *
 *  These are handled specially by the system.
 */
interface SpecialBlock : Block

/**
 * A [SpecialBlock]. When inputted true through its only input, will tell the system to shut down.
 */
class Shutdown internal constructor() : AbstractBlock(
    1, 0, IN_FIRST, ALWAYS
), SpecialBlock {
    internal var shouldShutdown: Boolean = false
        private set

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val input = inputs[0]
        shouldShutdown = input as? Boolean ?: false
    }

    override fun init(outputs: MutableList<Any?>) {
        shouldShutdown = false
    }
}

/**
 * A [SpecialBlock]. It's single output is the number of the current loop; starting with 0.
 */
class LoopNumber internal constructor() : AbstractBlock(
    0, 1, IN_FIRST, LAZY
), SpecialBlock {
    internal var loopNumber: Int = 0

    override fun init(outputs: MutableList<Any?>) {
        loopNumber = 0
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = loopNumber
    }
}

/**
 * A [SpecialBlock]. It's single output is time elapsed of the previous loop in seconds; or
 * Double.NaN if this is the first loop run.
 */
class LoopTime internal constructor() : AbstractBlock(
    0, 1, IN_FIRST, LAZY
), SpecialBlock {
    internal var loopTime: Double = Double.NaN

    override fun init(outputs: MutableList<Any?>) {
        loopTime = Double.NaN
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = loopTime
    }
}