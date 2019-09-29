package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.mechanics.MotionState2
import org.futurerobotics.jargon.mechanics.MotionState3
import org.futurerobotics.jargon.mechanics.ValueMotionState3

/**
 * Attempts to convert the input into a [MotionState3] representation of <T>;
 *
 * @throws ClassCastException if impossible.
 */
@Suppress("UNCHECKED_CAST")
fun <T : Any> Any.toMotionState3(zero: T): MotionState3<T> = when (this) {
    !is MotionState2<*> -> ValueMotionState3(this as T, zero, zero)
    is MotionState3<*> -> this as MotionState3<T>
    else -> ValueMotionState3(s as T, v as T, zero)
}

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