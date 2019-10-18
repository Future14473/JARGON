package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST_ALWAYS


/**
 * A block with one constant output [value].
 *
 * This is itself a [BlocksConfig.Output] representing this block's only output.
 *
 * @param value the constant value
 */
class Constant<T>(private val value: T) : SingleOutputBlock<T>(0, IN_FIRST_LAZY) {
    override fun doInit(): T? = value
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): T = value
    override fun toString(): String = "Constant($value)"
}

/**
 * A block with only one output [value], which can be changed externally.
 *
 * This is itself a [BlocksConfig.Output] representing this block's only output.
 *
 * @param value the value outputted
 */
class ExternalValue<T>(@Volatile var value: T) : SingleOutputBlock<T>(0, IN_FIRST_LAZY) {
    override fun doInit(): T? = null
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): T = value
    override fun toString(): String = "ExternalConstant($value)"
}

/**
 * A block with only one input, and stores the value inputted in [value]. Useful for extracting information
 * out of a system.
 *
 * This is itself a [BlocksConfig.Input] representing its only input.
 */
@Suppress("UNCHECKED_CAST")
class Monitor<T> : SingleInputBlock<T>(
    1, Block.Processing.IN_FIRST_ALWAYS
) {
    @Volatile
    private var _value: T? = null
    /**
     * The last value given to this monitor. Will be `null` if nothing has been received yet (or the given value
     * is null).
     */
    val value: T? get() = _value

    override fun init() {
        _value = null
    }

    override fun processInput(input: T, systemValues: SystemValues) {
        _value = input
    }

    override fun getOutput(index: Int): Any? = throw IndexOutOfBoundsException(index)

    override fun toString(): String = "Monitor($value)"
}

/**
 * A block that simply stores its input, and outputs it the next loop; so it is [OUT_FIRST_ALWAYS].
 * This is useful for breaking up loops.
 *
 * An [initialValue] must be given.
 *
 * This is also usable with with [BlocksConfig.delay]
 */
class Delay<T>(private val initialValue: T) : Pipe<T, T>(OUT_FIRST_ALWAYS) {
    override fun doInit(): T? = initialValue
    override fun pipe(input: T): T = input
}
