package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.mechanics.MotionState2
import org.futurerobotics.jargon.mechanics.MotionState3


/**
 * A block that takes a [MotionState3] or [MotionState2] and splits it into its components in order.
 *
 * Inputs:
 * 1. A [MotionState2]/[MotionState3], must not be null.
 *
 * Outputs:
 * 1. value
 * 2. velocity
 * 3. acceleration, if given value is [MotionState3], else null.
 */
class SplitMotionStateBlock :
    AbstractBlock(1, 3, Block.InOutOrder.IN_FIRST, Block.Processing.LAZY) {
    override fun init(outputs: MutableList<Any?>) {
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        when (val t = inputs[0]!!) {
            is MotionState2<*> -> {
                outputs[0] = t.s
                outputs[1] = t.v
                if (t is MotionState3<*>)
                    outputs[2] = t.a
            }
            else -> throw ClassCastException()
        }
    }
}