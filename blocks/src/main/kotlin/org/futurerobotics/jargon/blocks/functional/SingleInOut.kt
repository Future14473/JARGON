package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Block.Processing.*
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.SingleInputBlock
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.util.asUnmodifiableList

/**
 * A block with one constant output [value].
 *
 * This is itself a [Block.Output] representing this block's only output.
 *
 * @param value the constant value
 */
class Constant<T>(private val value: T) : SingleOutputBlock<T>(LAZY) {

    override fun Context.getOutput(): T = value

    override fun toString(): String = "Constant($value)"
}

/**
 * A block with only one output [value], which can be changed externally.
 *
 * This is itself a [Block.Output] representing this block's only output.
 *
 * @param value the value outputted
 */
class ExternalValue<T>(@Volatile var value: T) : SingleOutputBlock<T>(LAZY) {

    override fun Context.getOutput(): T = value

    override fun toString(): String = "ExternalConstant($value)"
}

/**
 * A block with only one input, and stores the value inputted in [value]. Useful for extracting information
 * out of a system.
 *
 * This is itself a [Block.Input] representing its only input.
 */
@Suppress("UNCHECKED_CAST")
class Monitor<T> : SingleInputBlock<T>(ALWAYS) {

    /**
     * The last value given to this monitor. Will be `null` if nothing has been received yet (or the given value
     * is null).
     */
    @Volatile
    var value: T? = null
        private set

    override fun init() {
        value = null
    }

    override fun Context.process(input: T) {
        value = input
    }

    override fun toString(): String = "Monitor($value)"
}

/**
 * A block that simply stores its input, and outputs it the next loop; so it is [OUT_FIRST].
 * This is useful for breaking up loops.
 *
 * An [initialValue] must be given, which will be the first output given when the system first started, before
 * any inputs have been given.
 */
class Delay<T>(private val initialValue: T) : PipeBlock<T, T>(OUT_FIRST) {

    override fun Context.pipe(
        input: T
    ): T = if (isFirstTime) initialValue else input
}

/**
 * A block with a single input that records the values it received every loop.
 * Usually used for graphing.
 */
class Recording<T> : SingleInputBlock<T>(ALWAYS) {

    private val _values = ArrayList<T>()

    /** The values recorded by this block */
    val values: List<T> = _values.asUnmodifiableList()

    override fun Context.process(input: T) {
        _values += input
    }
}
