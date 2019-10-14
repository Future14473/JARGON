package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.*
import org.futurerobotics.jargon.util.fixedSizeMutableListOfNulls
import kotlin.reflect.KProperty

/**
 * A `Block` in designing a control system. This can represents anything with a notion of _inputs_ or
 * _outputs_: any process, calculation, value, measurement, interaction, etc.
 *
 * A `Block` can take any number of _input_ and _outputs_; including 0, defined by [numInputs] and
 * [numOutputs]. Several blocks can then be connected during _configuration_ within the context of a [BlocksConfig],
 * to form a [BlocksSystem] which then can be run. **Blocks can only be run after they have been configured**
 *
 * Each input and output to a block is associated with a given index; corresponding to the index in [BlocksConfig.Input] and
 * [BlocksConfig.Output]. This input index is the same as in the list given during [process], and the output index the same
 * when polled in [getOutput].
 *
 * **Subclasses should provide methods for retrieving [BlocksConfig.Input]/[BlocksConfig.Output]s for configuration**.
 *
 * When a block is run within a [BlocksSystem], it will be run repeatedly in _loops_.
 * - [init] will always be run when the entire system first starts.
 * - The way a block is processed each _loop_ depend on the [processing] of this block, see there for specifications.
 *   It may be possible for a block to not process every loop, allowing for more dynamic behavior.
 *
 * Type checking of inputs/outputs is only done at runtime; but can be _assisted_ at compile time using
 * the types of [BlocksConfig.Input] and [BlocksConfig.Output]
 *
 * Subclasses should explain what each input and output is and the block's behavior.
 *
 * @see AbstractBlock
 * @see ListStoreBlock
 * @see SingleOutputBlock
 * @see SpecialBlock
 *
 * TODO REVISE DOC
 */
interface Block {
    /** The number of inputs to this block;*/
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
     * These options allow for more dynamic behavior, as blocks may not be run every loop.
     *
     * At least one block in a component system must be _not_ [IN_FIRST_LAZY] since if everyone is lazy nobody
     * will process.
     *
     * @property isAlwaysProcess if this [Processing] is an _always process_.
     * @property isOutFirst if this [Processing] is OutFirst.
     */
    enum class Processing(val isAlwaysProcess: Boolean, val isOutFirst: Boolean) {

        /**
         * A block with [IN_FIRST_LAZY] processing will only [process] or poll [getOutput] if another block requests its
         * outputs, otherwise it may not process.
         *
         * Blocks that do nothing more than process their inputs directly into output without storing information
         * of any kind should have this processing.
         * @see [Processing]
         */
        IN_FIRST_LAZY(false, false),
        /**
         * A block with [IN_FIRST_ALWAYS] processing will always be [process]ed every loop, and inputs must be given
         * before outputs are extracted. However, [getOutput] will still only be called if necessary.
         *
         * Blocks that require that it receives information every single loop should have this kind of processing.
         * @see [Processing]
         */
        IN_FIRST_ALWAYS(true, false),
        /**
         * A block with [OUT_FIRST_ALWAYS] processing will have its outputs polled _before_ [process] is called, every
         * loop, and [process] is only called at the very end of every loop.
         * **A block with [OUT_FIRST_ALWAYS] will always request _every single input, every loop_**. This is (one of)
         * the only ways to prevent values for next cycle being given out instead.
         *
         * At least one block in a loop of blocks must be [OUT_FIRST_ALWAYS]; For example a block that directly
         * interacts with hardware or external sources to be this kind of processing since measurements (outputs) are
         * usually taken _before_ signal (inputs)
         * @see [Processing]
         */
        OUT_FIRST_ALWAYS(true, true);

        //There is no OUT_FIRST_LAZY since that causes problems and hence violates the principle of least astonishment.
    }

    /**
     * Resets and initializes this block; Called when the _entire_ system first starts.
     */
    fun init()

    /**
     * Processes this block to prepare for outputs. Only called at most once per loop, and if this block is
     * _not_ [OUT_FIRST_ALWAYS], before any [getOutput] calls.
     *
     * It may be possible for [process] to be called but not [getOutput] if this block is [IN_FIRST_ALWAYS]
     *
     * @param inputs the current inputs to the block. A block is allowed to store the inputs but should not assume
     * that the inputs list will stay the same every loop as it might not be.
     */
    fun process(inputs: List<Any?>, systemValues: SystemValues)

    /**
     * Gets an output of this block by [index]. A given output index will only be requested at most once per loop.
     *
     * If this block is _not_ [OUT_FIRST_ALWAYS]:
     * - [process] will be called before any outputs are retrieved, per cycle.
     * - [getOutput] will _only_ be called if another block requests its outputs.
     */
    fun getOutput(index: Int): Any?

    /** A list of the [Class]es of this block's inputs; in order. Used to verify types.*/
    val inputTypes: List<Class<*>>
    /** A list of the [Class]es of this block's outputs; in order. Used to verify types.*/
    val outputTypes: List<Class<*>>

    val outputByIndex(index: Int):
    /**
     * Does any possible necessary preperation for this block to actualy run from the given [config]. Also, verifies
     * that the current configuration on the given [BlocksConfig] is valid (for example, all required inputs are
     * connected). Otherwise, should throw an [IllegalBlockConfigurationException].
     */
    fun prepareAndVerify(config: BlocksConfig)
}

/**
 * Base implementation of [Block]; where constants can be inputted via constructor.
 *
 * Note that on [prepareAndVerify], it checks to make sure _all_ inputs are connected. If this is not desired,
 * override that function/method.
 */
abstract class AbstractBlock constructor(
    final override val numInputs: Int,
    final override val numOutputs: Int,
    final override val processing: Block.Processing
) : Block {

    /**
     * Default implementation of [Block.prepareAndVerify] that runs [requireAllConnected]
     * Override if you do not want this behavior.
     */
    override fun prepareAndVerify(config: BlocksConfig): Unit = requireAllConnected(config)

    /**
     * Requires that all inputs are connected on the given [config], or else throws an
     * [IllegalBlockConfigurationException].
     */
    protected fun requireAllConnected(config: BlocksConfig): Unit = config.run {
        repeat(numInputs) {
            if (!BlocksConfig.Input<Any>(this@AbstractBlock, it).isConnected())
                throw IllegalBlockConfigurationException("All inputs to ${this@AbstractBlock} must be connected.")
        }
    }

    /**
     * Gets a [BlocksConfig.Output] for this block, with the given input [index].
     */
    protected open fun <T> inputIndex(index: Int): BlocksConfig.Input<T> {
        if (index !in 0..numInputs) throw IndexOutOfBoundsException(index)
        return BlocksConfig.Input(this, index)
    }

    /**
     * Gets a [BlocksConfig.Output] for this block, with the given output [index].
     */
    protected open fun <T> outputIndex(index: Int): BlocksConfig.Output<T> {
        if (index !in 0..numOutputs) throw IndexOutOfBoundsException(index)
        return BlocksConfig.Output(this, index)
    }
}

/**
 * An implementation of a [Block] that stores _all_ its outputs in a list when processed.
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

    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        outputs.fill(null)
        return process(inputs, outputs)
    }

    final override fun getOutput(index: Int): Any? = outputs[index]

    /**
     * Initializes this component; possibly filling the [outputs] if this component is [OUT_FIRST_ALWAYS].
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
 * This is also itself a [BlocksConfig.Output] representing its only output.
 */
abstract class SingleOutputBlock<T> @JvmOverloads constructor(
    numInputs: Int,
    processing: Block.Processing = IN_FIRST_LAZY
) : AbstractBlock(numInputs, 1, processing),
    BlocksConfig.Output<T> {

    private var value: T? = null

    final override fun init() {
        value = doInit()
    }

    /**
     * Initializes this block, and if this block is [OUT_FIRST_ALWAYS], returns the initial output
     *
     * Otherwise return value will be ignored so just return `null`
     */
    protected abstract fun doInit(): T?

    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        value = getOutput(inputs, systemValues)
    }

    /**
     * Processes this block using the given [inputs] and returns the (only) output of this block.
     */
    protected abstract fun getOutput(inputs: List<Any?>, systemValues: SystemValues): T


    final override fun getOutput(index: Int): Any? {
        if (index != 0) throw IndexOutOfBoundsException(index)
        return value
    }

    override val block: Block get() = this
    override val index: Int get() = 0
}

/**
 * A block implementation that implements input and outputs as Property Delegates,
 * retrieved using [input], [output].
 *
 * For example, for a block with 2 inputs and 2 outputs:
 * ```kotlin
 * class MyBlock : DelegatedPropertiesBlock(Block.Processing.IN_FIRST_LAZY){
 *      private val doubleInput: Double by input()
 *      private val intInput: Int by input()
 *
 *      private var doubleOutput: Double by output()
 *      private var boolOutput: Boolean? by output()
 *
 *      override fun process(){
 *          doubleOutput = doubleInput
 *          boolOutput = intInput == 3
 *      }
 * }
 *
 * ```
 */
@Suppress("NOTHING_TO_INLINE")
abstract class DelegatedPropertiesBlock(override val processing: Block.Processing) :
    Block {

    private val inputs: MutableList<InputDelegate<*>> = ArrayList()
    private val outputs: MutableList<OutputDelegate<*>> = ArrayList()
    final override val numInputs: Int = inputs.size
    final override val numOutputs: Int = outputs.size
    private var outsideInputs: List<Any?>? = null
    /** Specific delegates for system values. */
    protected val systemValues: SystemValueDelegates = SystemValueDelegates()

    /** Gets a [Input] delegate. The [value][Input.value] of this property will be the corresponding input. */
    protected fun <T> input(): Input<T> = InputDelegate()

    /**
     * Gets an [Output] delegate. Writing to this property (or setting the [value][Output.value]) of this property
     * will set the current output.
     */
    protected fun <T> output(): Output<T> = OutputDelegate()

    /** Represents a delegate for one of this block's inputs. Also is a [BlocksConfig.Input] for configuration. */
    protected interface Input<T> : BlocksConfig.Input<T> {

        /** Gets the value of the current input. */
        val value: T
    }

    /** operator fun for kotlin delegate */
    protected inline operator fun <T> Input<T>.getValue(thisRef: Any?, property: KProperty<*>): T = value

    @Suppress("UNCHECKED_CAST")
    private inner class InputDelegate<T> : Input<T> {

        override val index = numInputs //NOT get

        init {
            inputs += this
        }

        override val value: T = outsideInputs!![index] as T
        override val block: Block get() = this@DelegatedPropertiesBlock
    }

    /** Represents a delegate for one of this block's outputs.  Also is a [BlocksConfig.Output] for configuration.*/
    protected interface Output<T> : BlocksConfig.Output<T> {

        /** Sets the current value of this output. */
        var value: T
    }

    /** operator fun for kotlin delegate */
    protected inline operator fun <T> Output<T>.getValue(thisRef: Any?, property: KProperty<*>): T = value

    /** operator fun for kotlin delegate */
    protected inline operator fun <T> Output<T>.setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        this.value = value
    }

    @Suppress("UNCHECKED_CAST")
    private inner class OutputDelegate<T> : Output<T> {

        override val index = numOutputs //NOT get

        init {
            outputs += this
        }

        private var _value: T? = null
        override var value: T
            get() = _value as T
            set(value) {
                _value = value
            }
        override val block: Block get() = this@DelegatedPropertiesBlock
    }

    /** Specific delegates for system values. */
    protected inner class SystemValueDelegates internal constructor() {

        /** Shutdown output. */
        val shutdown: Output<Boolean?> by lazy { output<Boolean?>() }
        /** Loop number input */
        val loopNumber: Input<Int> by lazy { input<Int>() }
        /** Loop time input */
        val loopTime: Input<Double> by lazy { input<Double>() }
    }

    final override fun init() {
        outsideInputs = null
        doInit()
    }

    /** Performs initialization, possibly setting outputs if this block has a processing of [OUT_FIRST_ALWAYS]. */
    protected abstract fun doInit()

    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        outsideInputs = inputs
        doProcess()
    }

    /** Does the processing, using delegated input and output properties. */
    protected abstract fun doProcess()

    final override fun getOutput(index: Int): Any? = outputs[index].value

}

/**
 * A block that has 1 input and 1 output, where the output is strictly the input run through the [pipe] function.
 *
 * This itself if both a [BlocksConfig.Input] and [BlocksConfig.Output] representing its input and output.
 * */
abstract class Pipe<T, R> : SingleOutputBlock<R>(
    1, IN_FIRST_LAZY
), BlocksConfig.Input<T> {

    override fun doInit(): R? = null
    @Suppress("UNCHECKED_CAST")
    override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): R = pipe(inputs[0] as T)

    /**
     * Transforms the input value to the output value.
     */
    protected abstract fun pipe(input: T): R

    companion object {
        /**
         * Creates a [Pipe] using the given [pipe] function. May throw ClassCast exception
         */
        @JvmStatic
        inline operator fun <T, R> invoke(crossinline pipe: (T) -> R): Pipe<T, R> =
            object : Pipe<T, R>() {
                override fun pipe(input: T): R = pipe(input)
            }
    }
}


/**
 * Creates a block that pipes this output through the given [transform] (usually using a [Pipe] block), connects
 * it, and returns the pipe's output.
 */
inline fun <T, R> BlocksConfig.pipe(from: BlocksConfig.Output<T>, crossinline transform: (T) -> R): Pipe<T, R> =
    Pipe(transform).apply { from into this }


/** A block that has 1 input and 1 output, where the output is a the input run through the [combine] function. */
abstract class Combine<A, B, R> : SingleOutputBlock<R>(2, IN_FIRST_LAZY) {

    final override fun doInit(): R? = null
    @Suppress("UNCHECKED_CAST")
    final override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): R =
        combine(inputs[0] as A, inputs[1] as B)

    /**
     * Combines two input values to the output value.
     */
    protected abstract fun combine(a: A, b: B): R

    /** The first input to this combine block. */
    val first: BlocksConfig.Input<A> get() = inputIndex(0)
    /** The second input to this second block. */
    val second: BlocksConfig.Input<B> get() = inputIndex(1)

    companion object {
        /**
         * Creates a [Combine] using the given [combine] function. May throw [ClassCastException]
         */
        @JvmStatic
        inline operator fun <A, B, R> invoke(crossinline combine: (A, B) -> R): Combine<A, B, R> =
            object : Combine<A, B, R>() {
                override fun combine(a: A, b: B): R = combine(a, b)
            }
    }
}

/**
 * Combines the [firstOutput] and [secondOutput] [BlocksConfig.Output]s through the given [combine] function and
 * returns [BlocksConfig.Output] representing the result of that combination.
 *
 * This is done through a [Combine] block, where the inputs are already connected.
 */
inline fun <A, B, R> BlocksConfig.combine(
    firstOutput: BlocksConfig.Output<A>, secondOutput: BlocksConfig.Output<B>,
    crossinline combine: (A, B) -> R
): Combine<A, B, R> = Combine(combine).apply { firstOutput.into(first); secondOutput.into(second) }
