package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.*
import org.futurerobotics.jargon.util.fixedSizeMutableListOfNulls


/**
 * A `Block` in designing a control system. This can represents anything with a notion of _inputs_ or
 * _outputs_: any process, calculation, value, measurement, interaction, etc.
 *
 * A `Block` can take any number of _input_ and _outputs_; including 0, defined by [numInputs] and
 * [numOutputs]. Several blocks can then be connected during _configuration_ within the context of a [BlocksConfig],
 * to form a [BlocksSystem] which then can be run. **Blocks can only be run after they have been configured**
 *
 * Each input and output to a block is associated with a given index; corresponding to the index in [BlockInput] and
 * [BlockOutput]. This is the same index in the list given as inputs, and the index when polled in [getOutput]
 *
 * **Subclasses should provide methods for retrieving [BlockInput]/[BlockOutput]s for configuration**.
 * A `Block` can have more inputs and outputs than it exposes publicly, as some inputs/outputs can be [selfConfig]ed.
 *
 * Type checking of inputs/outputs is only done at runtime; but can be _assisted_ at compile time using
 * the types of [BlockInput] and [BlockOutput]
 *
 * When a block is run within a [BlocksSystem], it will be run repeatedly in _loops_.
 * - [init] will always be run when the entire system first starts.
 * - The way a block is processed each _loop_ depend on the [processing] of this block, see there for specifications.
 *   It may be possible for a block to not process every loop, allowing for more dynamic behavior.
 *
 * Subclasses should explain in documentation what each input and output is and the block's behavior.
 *
 * There is also a [SystemValues] with special purposes into the life of the [BlocksSystem] itself;
 * which can be accessed directly within the [BlocksSystemBuilder].
 *
 * @see AbstractBlock
 * @see ListStoreBlock
 * @see SingleOutputBlock
 * @see SystemValues
 */
interface Block {
    //config.
    /** The number of total inputs to this block;*/
    val numInputs: Int

    /** The number of outputs to this block */
    val numOutputs: Int

    /** The processing policy of this block. See [Processing] */
    val processing: Processing

    /**
     * Defines how this component is run, which can be:
     *
     * - [IN_FIRST_LAZY]
     * - [IN_FIRST_ALWAYS]
     * - [OUT_FIRST_ALWAYS]
     *
     *  These options allow for more dynamic behavior, as blocks may not be run every loop.
     *
     * At least one block in a component system must be _not_ [IN_FIRST_LAZY] since if everyone is lazy nobody
     * will process.
     *
     * @property isAlwaysProcess if this [Processing] is an _always process_.
     */
    enum class Processing(val isAlwaysProcess: Boolean) {
        /**
         * A block with [IN_FIRST_LAZY] will only [process] or poll [getOutput] if another block requests its outputs,
         * and inputs must be retrieved and given before there are outputs.
         *
         * Blocks that do nothing more than process their inputs directly into output without storing information
         * of any kind should have this processing.
         * @see [Processing]
         */
        IN_FIRST_LAZY(false),
        /**
         * A block with [IN_FIRST_ALWAYS] processing will always be [process]ed every loop, and inputs must be given
         * before outputs are extracted. However, [getOutput] will still only be called if necessary.
         *
         * Blocks that require that it receives information every single loop should have this kind of processing.
         * @see [Processing]
         */
        IN_FIRST_ALWAYS(true),
        /**
         * A block with [OUT_FIRST_ALWAYS] will first have _all outputs extracted_ before [process]
         * is called, every loop. Outputs should be ready
         * upon [init], and the given inputs will always be for the _previous_ loop.
         *
         * At least one block in a loop of blocks must be [OUT_FIRST_ALWAYS]; It is recommended for a block that directly
         * interacts with hardware or external sources to be this kind of processing.
         * @see [Processing]
         */
        OUT_FIRST_ALWAYS(true);
    }

    /**
     * Resets and initializes this block; Called when the _entire_ system first starts.
     *
     * For example, resets can be done here, or pre-processing for [OUT_FIRST_ALWAYS] blocks can be done here.
     */
    fun init()

    /**
     * Processes this block to prepare for outputs. Only called at most once per loop, and before any [getOutput] calls
     * (if this block is not [OUT_FIRST_ALWAYS]).
     *
     * It may be possible for [process] to be called but not [getOutput] if this block is [IN_FIRST_ALWAYS]
     *
     * @param inputs the current inputs to the block.
     */
    fun process(inputs: List<Any?>)

    /**
     * Gets an output of this block by [index]. A given output index will only be requested at most once per loop.
     *
     * It is possible that not all outputs will be requested if this block is _not_ [OUT_FIRST_ALWAYS].
     *
     * [inputs] the current inputs also given to the block if the block wants to request more inputs that it did not
     * in [process].
     *
     * If this block is [OUT_FIRST_ALWAYS] the inputs will be an empty list (get everything required in [process]).
     */
    fun getOutput(index: Int, inputs: List<Any?>): Any?

    /**
     * Performs any possible automatic configurations, for example to [SystemValues]s, or to set internal state.
     * Called whenever this block is added to a [BlocksConfig].
     */
    fun selfConfig(config: BlocksConfig)

    /**
     * Checks that the current configuration on the given [BlocksConfig] is valid (for example, all required
     * inputs are connected). Otherwise, should throw an [IllegalBlockConfigurationException].
     * Called whenever this block is added to a [BlocksConfig].
     */
    fun verifyConfig(config: BlocksConfig)

}

/**
 * Base implementation of [Block]; where constants can be inputted via constructor.
 *
 * Note that on [verifyConfig], it checks to make sure _all_ inputs are connected. If this is not desired,
 * override that function/method.
 */
abstract class AbstractBlock constructor(
    final override val numInputs: Int,
    final override val numOutputs: Int,
    final override val processing: Block.Processing
) : Block {
    override fun selfConfig(config: BlocksConfig) {}

    /** Verifies that all inputs are connected. */
    override fun verifyConfig(config: BlocksConfig): Unit = config.run {
        repeat(numInputs) {
            if (!BasicBlockInput<Any>(this@AbstractBlock, it).isConnected())
                throw IllegalBlockConfigurationException("All inputs to ${this@AbstractBlock} must be connected.")
        }
    }

    /**
     * Gets a [BlockOutput] for this block, with the given input [index].
     */
    protected open fun <T> inputIndex(index: Int): BlockInput<T> {
        if (index !in 0..numInputs) throw IndexOutOfBoundsException(index)
        return BasicBlockInput(this, index)
    }

    /**
     * Gets a [BlockOutput] for this block, with the given output [index].
     */
    protected open fun <T> outputIndex(index: Int): BlockOutput<T> {
        if (index !in 0..numOutputs) throw IndexOutOfBoundsException(index)
        return BasicBlockOutput(this, index)
    }
}


/**
 * An implementation of a [Block] that stores _all_ its outputs in a list.
 */
abstract class ListStoreBlock @JvmOverloads constructor(
    numInputs: Int,
    numOutputs: Int,
    processing: Block.Processing = IN_FIRST_LAZY
) : AbstractBlock(numInputs, numOutputs, processing) {
    private val outputs = fixedSizeMutableListOfNulls<Any>(numOutputs)

    final override fun init() {
        outputs.fill(null)
        init(outputs)
    }

    final override fun process(inputs: List<Any?>) {
        outputs.fill(null)
        return process(inputs, outputs)
    }

    final override fun getOutput(index: Int, inputs: List<Any?>): Any? {
        return outputs[index]
    }

    /**
     * Initializes this component; possibly filling out the [outputs] if this component is [OUT_FIRST_ALWAYS].
     *
     * Otherwise, changes to the [outputs] list will be ignored.
     */
    protected abstract fun init(outputs: MutableList<Any?>)

    /**
     * Processes this component, given the [inputs] as a list and the [outputs] as a value.
     */
    protected abstract fun process(inputs: List<Any?>, outputs: MutableList<Any?>)

}

/**
 * A block that has a single output, which simplifies processing.
 *
 * This is also itself a [BlockOutput] representing its only output.
 */
abstract class SingleOutputBlock<T> @JvmOverloads constructor(
    numInputs: Int,
    processing: Block.Processing = IN_FIRST_LAZY
) : AbstractBlock(numInputs, 1, processing), BlockOutput<T> {
    private var value: T? = null

    /**
     * The output of this [SingleOutputBlock].
     *
     * Returns, since this block itself is a [BlockOutput]
     */
    val output: BlockOutput<T> get() = this

    override val block: Block get() = this
    override val outputIndex: Int get() = 0

    final override fun init() {
        value = doInit()
    }

    final override fun process(inputs: List<Any?>) {
        value = getOutput(inputs)
    }

    final override fun getOutput(index: Int, inputs: List<Any?>): Any? {
        if (index != 0) throw IndexOutOfBoundsException(index)
        return value
    }

    /**
     * Initializes this block, and if this block is [OUT_FIRST_ALWAYS], returns the initial output
     *
     * Otherwise return value will be ignored so just return `null`
     */
    protected abstract fun doInit(): T?

    /**
     * Processes this block using the given [inputs] and returns the (only) output of this block.
     */
    protected abstract fun getOutput(inputs: List<Any?>): T
}

/**
 * A block with one constant output [value].
 *
 * This is itself a [BlockOutput] representing this block's only output.
 *
 * @param value the constant value
 */
class Constant<T>(private val value: T) : SingleOutputBlock<T>(
    0, IN_FIRST_LAZY
) {

    override fun doInit(): T? = value

    override fun getOutput(inputs: List<Any?>): T = value

    override fun toString(): String {
        return "Constant($value)"
    }
}

/**
 * A block with only one output, [value], which can be changed externally.
 *
 * This is itself a [BlockOutput] representing this block's only output.
 *
 * @param value the value outputted
 */
class ExternalValue<T>(@Volatile var value: T) : SingleOutputBlock<T>(
    0, IN_FIRST_LAZY
) {
    override fun doInit(): T? = null

    override fun getOutput(inputs: List<Any?>): T = value

    override fun toString(): String {
        return "ExternalConstant($value)"
    }
}

/**
 * A block with only one input, and stores the value inputted in [value]. Useful for extracting information
 * out of a system.
 *
 * This is itself a [BlockInput] representing its only input.
 */
@Suppress("UNCHECKED_CAST")
class Monitor<T> : AbstractBlock(
    1, 0, IN_FIRST_ALWAYS
), BlockInput<T> {
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

    override fun process(inputs: List<Any?>) {
        _value = inputs[0] as T?
    }

    override fun getOutput(index: Int, inputs: List<Any?>): Any? {
        throw IndexOutOfBoundsException(index)
    }

    override val block: Block get() = this
    override val inputIndex: Int get() = 0

    override fun toString(): String {
        return "Monitor($value)"
    }
}

/**
 * A block that has 1 input and 1 output, where the output is strictly the input run through the [pipe] function.
 *
 * This itself if both a [BlockInput] and [BlockOutput] representing its input and output.
 * */
abstract class PipeBlock<T, R> : SingleOutputBlock<R>(
    1, IN_FIRST_LAZY
), BlockInput<T> {
    override fun doInit(): R? = null

    @Suppress("UNCHECKED_CAST")
    override fun getOutput(inputs: List<Any?>): R = pipe(inputs[0] as T)

    /**
     * Transforms the input value to the output value.
     */
    protected abstract fun pipe(input: T): R

    override val inputIndex: Int get() = 0

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
abstract class CombineBlock<A, B, R> : SingleOutputBlock<R>(2, IN_FIRST_LAZY) {

    override fun doInit(): R? = null

    @Suppress("UNCHECKED_CAST")
    override fun getOutput(inputs: List<Any?>): R = combine(inputs[0] as A, inputs[1] as B)

    /**
     * Combines two input values to the output value.
     */
    protected abstract fun combine(a: A, b: B): R


    /** The first input to this combine block. */
    val first: BlockInput<A> get() = inputIndex(0)
    /** The second input to this second block. */
    val second: BlockInput<B> get() = inputIndex(1)

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
