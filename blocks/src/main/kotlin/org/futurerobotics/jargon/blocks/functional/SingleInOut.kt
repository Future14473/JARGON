package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.util.asUnmodifiableList

/**
 * A block with one constant output [value].
 *
 * @param value the constant value
 */
class Constant<T>(private val value: T) : PrincipalOutputBlock<T>(Processing.LAZY) {

    override fun Context.getOutput(): T = value

    override fun toString(): String = "Constant($value)"
}

/**
 * A block with only one output [value], which can be changed externally.
 *
 * @param value the value outputted
 */
class ExternalValue<T>(@Volatile var value: T) : PrincipalOutputBlock<T>(Processing.LAZY) {

    override fun Context.getOutput(): T = value

    override fun toString(): String = "ExternalConstant($value)"
}

/**
 * A block with only one input, and stores the value inputted in [value]. Useful for extracting information
 * out of a system.
 */
@Suppress("UNCHECKED_CAST")
class Monitor<T> : Block(Processing.ALWAYS) {

    /** The [input] into this monitor. */
    val input: Input<T> = newInput()
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

    override fun Context.process() {
        value = input.get
    }

    override fun toString(): String = "Monitor($value)"
}

/**
 * A block that simply stores its input, and outputs it the next loop; so it is [Block.Processing.OUT_FIRST]
 * This is useful for breaking up loops.
 *
 * An [initialValue] must be given, which will be the first output given when the system first started, before
 * any inputs have been given.
 */
class Delay<T>(private val initialValue: T) : PipeBlock<T, T>(Processing.OUT_FIRST) {

    override fun Context.pipe(
        input: T
    ): T = if (isFirstTime) initialValue else input
}

/**
 * A block with a single input that records the values it received every loop.
 * Usually used for graphing.
 */
class Recording<T> : Block(Processing.ALWAYS) {

    /** The [input] into this [Recording]. */
    val input: Input<T> = newInput()

    private val _values = ArrayList<T>()

    /** The values recorded by this block */
    val values: List<T> = _values.asUnmodifiableList()

    override fun Context.process() {
        _values += input.get
    }
}

/**
 * A class that simply passes its input into its output.
 *
 * Useful in a limited number of cases.
 */
class Pass<T> : PrincipalOutputBlock<T>(Processing.LAZY) {

    /** The input into this [Pass] block. */
    val input: Input<T> = newInput()

    override fun Context.getOutput(): T = input.get
}
