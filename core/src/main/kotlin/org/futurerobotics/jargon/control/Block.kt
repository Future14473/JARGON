package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.InOutOrder.OUT_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.control.Block.Processing.LAZY


/**
 * A `Block` in designing a control system. This can represents anything with a notion of _inputs_ and
 * _outputs_: any process, calculation, source, measurement,
 * etc., whatsoever.
 *
 * A `Block` can take any number of _input_ and _outputs_; including 0, defined by [numInputs] and
 * [numOutputs]. These inputs/outputs can then be connected with each other to form a [BlockSystem] that runs it.
 * Type checking of inputs/outputs is done at runtime; although can be _assisted_ at compile time using [BlocksBuilder]
 *
 * When a `Block` is processed, [process] will be called with a given input and output list to place outputs.
 *
 * Input and output values should _not_ be `null` as that causes too many problems. Instead, to specify an absence of
 * a value, use [kotlin.Unit] (kotlin.Unit.INSTANCE in java) instead.
 *
 * Updating of blocks is _lazy_, a block is not required to poll all of its inputs; and a block
 * may not be processed at all if no block requests it's outputs. This allows for more dynamic behavior.
 *
 *
 * Subclasses should explain in documentation what each input and output is and the block's behavior.
 *
 * A block also [inOutOrder]; See there for details.
 *
 * Some common blocks have been integrated into the [BlocksBuilder]
 *
 * There are some [SpecialBlock]s that have special purposes that tap into the life of the [BlockSystem] itself;
 * the outputs/inputs of these can be accessed directly within the [BlocksBuilder].
 *
 * @see AbstractBlock
 */
interface Block {

    /** the InOutOrder of this block. See [InOutOrder] */
    val inOutOrder: InOutOrder

    /** The processing policy of this block. See [Processing] */
    val processing: Processing

    /** The number of inputs to this block */
    val numInputs: Int

    /** The number of outputs to this block */
    val numOutputs: Int

    /**
     * Resets and initializes this block; Only called when the _entire_ system first starts.
     *
     * Optionally, if this component has a [inOutOrder] of [OUT_FIRST], set the initial inputs using the outputs list.
     */
    fun init(outputs: MutableList<Any?>)

    //do we need a stop()?

    /**
     * Processes using the given [inputs] list, and stores the in the given [outputs] list.
     *
     * If a input is not connected, the value [kotlin.Unit] will be given. It is also possible for another component
     * to output this value.
     *
     * @throws ClassCastException if the given inputs do not conform to the right class
     */
    fun process(inputs: List<Any?>, outputs: MutableList<Any?>)

    /**
     * Specifies what order inputs/outputs are given; either [IN_FIRST], [OUT_FIRST].
     *
     * [IN_FIRST] outputs need to have [process] called before getting outputs,
     * while [OUT_FIRST] will always have outputs stored before input is processed.
     *
     * At least one block in a loop needs to be [OUT_FIRST], else it is impossible to process.
     */
    enum class InOutOrder {
        /** Input is required first. See [InOutOrder] */
        IN_FIRST,
        /** Processes output first. SEe [InOutOrder] */
        OUT_FIRST,
    }

    /**
     * Defines how this component is run, which can either be [LAZY] or [ALWAYS]
     *
     * - A block with a processing [ALWAYS] will always be processed every cycle.
     *
     * - A block with [LAZY] processing is not run unless another other block connected to it requests its output.
     *  This means it potentially may not be run every cycle. potentially allows for dynamic behavior and speedup.
     * (Beware that if this block is also out_first (see [InOutOrder]), this means the block first output values
     * based on inputs several cycles before and the output may be stale).
     *
     * At least one block in a component system must must [ALWAYS] process; since if everyone is lazy nobody
     * will process.
     */
    enum class Processing {
        /** This block is always processed. See [Processing] */
        ALWAYS,
        /** This block is lazily processed. See [Processing] */
        LAZY,
    }
}

/**
 * Base implementation of [Block]; where constants can be inputted via constructor.
 */
abstract class AbstractBlock @JvmOverloads constructor(
    final override val numInputs: Int,
    final override val numOutputs: Int,
    final override val inOutOrder: Block.InOutOrder = IN_FIRST,
    final override val processing: Block.Processing = LAZY
) : Block

/**
 * A block with one constant output, which is the value given.
 * @param value the constant value
 */
class Constant<T>(private val value: T) : AbstractBlock(
    0, 1, IN_FIRST, LAZY
) {

    override fun init(outputs: MutableList<Any?>) {
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = value
    }

    override fun toString(): String {
        return "Constant($value)"
    }
}

/**
 * A block with only one output, [value], which can be changed externally.
 *
 * @param value the value outputted
 */
class ExternalInput<T>(@Volatile var value: T) : AbstractBlock(
    0, 1, IN_FIRST, LAZY
) {
    override fun init(outputs: MutableList<Any?>) {
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = value
    }

    override fun toString(): String {
        return "ExternalConstant($value)"
    }
}

/**
 * A block with only one input, and stores the value inputted in [value]. Useful for extracting information
 * out of a system. This includes a class for type checking.
 */
class Monitor<out T> : AbstractBlock(
    1, 0, IN_FIRST, ALWAYS
) {
    /**
     * The last value given to this monitor. Will be `null` if nothing has been received yet.]
     */
    val value: T? get() = _value

    @Volatile
    private var _value: T? = null


    override fun init(outputs: MutableList<Any?>) {
        _value = null
    }

    @Suppress("UNCHECKED_CAST")
    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        _value = inputs[0] as T?
    }

    override fun toString(): String {
        return "Monitor($value)"
    }
}

/** A block that has 1 input and 1 output, where the output is strictly the input run through the [pipe] function. */
abstract class PipeBlock<T, R> : AbstractBlock(
    1, 1, IN_FIRST, LAZY
) {
    final override fun init(outputs: MutableList<Any?>) {
    }

    @Suppress("UNCHECKED_CAST")
    final override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = pipe(inputs[0] as T)
    }

    /**
     * Transforms the input value to the output value.
     */
    protected abstract fun pipe(input: T): R

    companion object {

        /**
         * Creates a [PipeBlock] using the given [pipe] function. May throw ClassCast exception
         */
        @JvmStatic
        inline operator fun <T, R> invoke(crossinline pipe: (T) -> R): PipeBlock<T, R> =
            object : PipeBlock<T, R>() {
                override fun pipe(input: T): R {
                    return pipe(input)
                }
            }
    }
}

/** A block that has 1 input and 1 output, where the output is a the input run through the [combine] function. */
abstract class CombineBlock<A, B, R> :
    AbstractBlock(2, 1, IN_FIRST, LAZY) {

    @Suppress("UNCHECKED_CAST")
    final override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = combine(inputs[0] as A, inputs[1] as B)
    }

    final override fun init(outputs: MutableList<Any?>) {
    }

    /**
     * Combines two input values to the output value.
     */
    protected abstract fun combine(a: A, b: B): R

    companion object {

        /**
         * Creates a [CombineBlock] using the given [combine] function. May throw [ClassCastException]
         */
        @JvmStatic
        inline operator fun <A, B, R> invoke(crossinline combine: (A, B) -> R): CombineBlock<A, B, R> =
            object : CombineBlock<A, B, R>() {
                override fun combine(a: A, b: B): R {
                    return combine(a, b)
                }
            }
    }
}
