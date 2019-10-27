package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.*
import org.futurerobotics.jargon.util.fixedSizeMutableListOfNulls
import org.futurerobotics.jargon.util.unsafeCast

/**
 * Base implementation of [Block]; where constants are set via constructor.
 *
 * On [prepareAndVerify], [requireAllConnected] is called. If this is not desired, override.
 */
abstract class AbstractBlock constructor(
    final override val numInputs: Int,
    final override val numOutputs: Int,
    final override val processing: Block.Processing
) : Block {

    /**
     * Default implementation of [Block.prepareAndVerify] that runs [requireAllConnected].
     * Override if you do not want this behavior.
     */
    override fun prepareAndVerify(config: BlocksConfig): Unit = requireAllConnected(config)

    /**
     * Requires that all inputs are connected on the given [config], or else throws an
     * [IllegalBlockConfigurationException].
     */
    protected fun requireAllConnected(config: BlocksConfig): Unit = config.run {
        repeat(numInputs) {
            if (!inputIsConnected(it)) throw IllegalBlockConfigurationException("All inputs to ${this@AbstractBlock} must be connected.")
        }
    }

    /**
     * Gets a [BlocksConfig.Input] for this block, with the given [index].
     *
     * It is the user's responsibility to make sure that the generic is of the right type.
     *
     * Subclasses should use this to provide inputs/outputs.
     * @see [configOutput]
     */
    protected fun <T> configInput(index: Int): BlocksConfig.Input<T> {
        if (index !in 0..numInputs) throw IndexOutOfBoundsException(index)
        return BlocksConfig.Input.ofUnsafeCast(this, index)
    }

    /**
     * Gets a [BlocksConfig.Output] for this block, with the given [index].
     *
     * It is the user's responsibility to make sure that the generic is of the right type.
     *
     * Subclasses should use this to provide inputs/outputs.
     * @see [configInput]
     */
    protected fun <T> configOutput(index: Int): BlocksConfig.Output<T> {
        if (index !in 0..numOutputs) throw IndexOutOfBoundsException(index)
        return BlocksConfig.Output.ofUnsafeCast(this, index)
    }

    override fun stop() {
        //default: Do nothing.
    }
}

/**
 * An implementation of a [Block] that stores _all_ its outputs in a list when processed.
 *
 * The list is provided in the [init]/[process] functions.
 */
abstract class ListStoreBlock(numInputs: Int, numOutputs: Int, processing: Block.Processing) : AbstractBlock(
    numInputs, numOutputs, processing
) {

    private val outputs = fixedSizeMutableListOfNulls<Any>(numOutputs)
    final override fun init() {
        outputs.fill(null)
        init(outputs)
    }

    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        outputs.fill(null)
        return process(inputs, systemValues, outputs)
    }

    final override fun getOutput(index: Int): Any? = outputs[index]
    /**
     * Initializes this block; possibly filling the [outputs] if this block is [OUT_FIRST_ALWAYS].
     *
     * Otherwise, changes to the [outputs] list will be ignored.
     *
     * (as in [Block.init])
     */
    protected abstract fun init(outputs: MutableList<Any?>)

    /**
     * Processes this block, given the [inputs] as a list and the [outputs] as a mutable list.
     *
     * (as in [Block.process])
     */
    protected abstract fun process(
        inputs: List<Any?>, systemValues: SystemValues, outputs: MutableList<Any?>
    )
}

/**
 * A block that has a single output of type [R], which simplifies processing in [processOutput].
 *
 * This is also itself a [BlocksConfig.Output] representing its only output.
 */
abstract class SingleOutputBlock<R>(numInputs: Int, processing: Block.Processing) : AbstractBlock(
    numInputs, 1, processing
), BlocksConfig.Output<R> {

    private var value: R? = null
    final override val block: Block get() = this
    final override val index: Int get() = 0
    final override fun init() {
        value = initialValue()
    }

    /**
     * Initializes this block, and if this block is [OUT_FIRST_ALWAYS], returns the initial output
     *
     * Otherwise return value will be ignored so just return `null`
     *
     * (as in [Block.init])
     */
    protected abstract fun initialValue(): R?

    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        value = processOutput(inputs, systemValues)
    }

    /**
     * Processes this block using the given [inputs] and returns the (only) output of this block
     *
     * (as in [Block.process] where output is stored)
     */
    protected abstract fun processOutput(inputs: List<Any?>, systemValues: SystemValues): R

    final override fun getOutput(index: Int): Any? {
        if (index != 0) throw IndexOutOfBoundsException(index)
        return value
    }
}

/**
 * A block that has a single input of type [T], which simplifies processing in [processInput]
 *
 * This is also itself a [BlocksConfig.Input] representing its only input.
 */
abstract class SingleInputBlock<T>(numOutputs: Int, processing: Block.Processing) : AbstractBlock(
    1, numOutputs, processing
), BlocksConfig.Input<T> {

    private var input: T? = null
    final override val block: Block get() = this
    final override val index: Int get() = 0
    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        input = inputs[0].unsafeCast<T>()
        processInput(input.unsafeCast(), systemValues)
    }

    final override fun getOutput(index: Int): Any? = getOutput(input.unsafeCast(), index)
    /** Gets the output of this block by index, also with the given [input], as in [getOutput][Block.getOutput]. */
    protected abstract fun getOutput(input: T, index: Int): Any?

    /**
     * Processes this block with the single [input] value
     *
     * (as in [Block.process])
     */
    protected abstract fun processInput(input: T, systemValues: SystemValues)
}

/**
 * A block that has a single input of type [T], and stores _all_ its outputs in a list:
 * a combination of [ListStoreBlock] and [SingleInputBlock]
 */
abstract class SingleInputListStoreBlock<T>(numOutputs: Int, processing: Block.Processing) : ListStoreBlock(
    1, numOutputs, processing
), BlocksConfig.Input<T> {

    final override val block: Block get() = this
    final override val index: Int get() = 0

    final override fun process(
        inputs: List<Any?>, systemValues: SystemValues, outputs: MutableList<Any?>
    ): Unit = processInput(inputs[0].unsafeCast(), systemValues, outputs)

    /**
     * Processes this block, given the single [input] and the [outputs] as a value
     *
     * (as in [Block.process] or [ListStoreBlock.process])
     */
    protected abstract fun processInput(
        input: T, systemValues: SystemValues, outputs: MutableList<Any?>
    )
}

/**
 * A block that ONLY takes in a single input of type [T], and runs [processInput] with it.
 *
 * Will always be [IN_FIRST_ALWAYS].
 */
abstract class InputOnlyBlock<T> : SingleInputBlock<T>(0, IN_FIRST_ALWAYS) {

    /** Processes the input to this block. */
    abstract override fun processInput(input: T, systemValues: SystemValues)

    override fun init() {
        //default: do nothing
    }

    override fun getOutput(input: T, index: Int): Any? {
        throw IndexOutOfBoundsException(index)
    }

    companion object {
        /**
         * Creates an [InputOnlyBlock] that only runs the given [operation] on the inputs it receives.
         */
        inline fun <T> of(crossinline operation: (T) -> Unit): InputOnlyBlock<T> = object : InputOnlyBlock<T>() {
            override fun processInput(input: T, systemValues: SystemValues) {
                operation(input)
            }
        }
    }
}

/**
 * A block that has 1 input of type [T] and 1 output of type [R], where the output is strictly the input run through
 * the [pipe] function.
 *
 * This also defaults [initialValue] to return `null`; override if this is not desired.
 *
 * This itself if both a [BlocksConfig.Input] and [BlocksConfig.Output] representing its input and output.
 *
 * A lambda version of this is available in [BlocksConfig.combine] for easier use.
 * */
abstract class PipeBlock<T, R> @JvmOverloads constructor(processing: Block.Processing = IN_FIRST_LAZY) :
    SingleOutputBlock<R>(1, processing), BlocksConfig.Input<T> {

    override fun initialValue(): R? = null
    final override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): R = pipe(inputs[0].unsafeCast())

    /**
     * Transforms the input value to the output value.
     */
    protected abstract fun pipe(input: T): R

    companion object {
        /** Creates a [PipeBlock] using the given [pipe] function. */
        @JvmStatic
        inline fun <T, R> with(crossinline pipe: (T) -> R): PipeBlock<T, R> = object : PipeBlock<T, R>(IN_FIRST_LAZY) {
            override fun pipe(input: T): R = pipe(input)
        }
    }
}

/**
 * A block that has 2 inputs of types [A] and [B] and 1 output of type [R], where the output is a the input run
 * through the [combine] function.
 *
 * This also defaults [initialValue] to return `null`; override if this is not desired.
 *
 * This is itself a [BlocksConfig.Output] representing its only output,
 * and it also provides [firstInput] and [secondInput] properties representing its inputs.
 *
 * A lambda version of this is available in [BlocksConfig.combine] for easier use.
 */
abstract class CombineBlock<A, B, R>(processing: Block.Processing = IN_FIRST_LAZY) :
    SingleOutputBlock<R>(2, processing) {

    /** The first input to this combine block. */
    val firstInput: BlocksConfig.Input<A> get() = configInput(0)
    /** The second input to this second block. */
    val secondInput: BlocksConfig.Input<B> get() = configInput(1)

    override fun initialValue(): R? = null
    final override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): R =
        combine(inputs[0].unsafeCast(), inputs[1].unsafeCast())

    /**
     * Combines two input values to the output value.
     */
    protected abstract fun combine(a: A, b: B): R

    companion object {
        /** Creates a [CombineBlock] using the given [combine] function. */
        @JvmStatic
        inline fun <A, B, R> of(
            crossinline combine: (A, B) -> R
        ): CombineBlock<A, B, R> = object : CombineBlock<A, B, R>(IN_FIRST_LAZY) {
            override fun combine(a: A, b: B): R = combine(a, b)
        }
    }
}

/**
 * A block that contains within itself another block sub-system, configured in [configSubsystem]
 *
 * When this block is processed it will process the entire sub-system.
 *
 * The subsystem does not support [SpecialBlock]s.
 */
abstract class CompositeBlock(numInputs: Int, numOutputs: Int, processing: Block.Processing) :
    AbstractBlock(numInputs, numOutputs, processing) {

    private lateinit var subsystem: Subsystem

    final override fun init(): Unit = subsystem.init()
    final override fun process(inputs: List<Any?>, systemValues: SystemValues): Unit =
        subsystem.process(inputs, systemValues)

    final override fun getOutput(index: Int): Any? = subsystem.getOutput(index)

    /**
     * Builds and returns a [BlocksConfig] for the subsystem.
     * [sources] are outputs that correspond to the value given from the outside world,
     * and [outputs] are inputs that correspond to values given _to_ the outside world.
     */
    protected abstract fun configSubsystem(
        sources: List<BlocksConfig.Output<Any?>>, outputs: List<BlocksConfig.Input<Any?>>
    ): BlocksConfig

    final override fun prepareAndVerify(config: BlocksConfig) {
        val sources = Sources()
        val outputs = Outputs()
        val subConfig = configSubsystem(sources.allOutputs(), outputs.allInputs())
        subsystem = Subsystem(subConfig, sources, outputs)
        prepareAndVerifyMore(config)
    }

    /** Performs additional [prepareAndVerify] actions. Only way to enforce "super call". */
    protected open fun prepareAndVerifyMore(config: BlocksConfig) {
        super.prepareAndVerify(config)
    }

    private inner class Subsystem(config: BlocksConfig, private val sources: Sources, private val outputs: Outputs) :
        AbstractBlocksRunner(config) {

        override lateinit var systemValues: SystemValues
        public override fun init() = super.init()
        public override fun stop() = super.stop()

        fun process(outsideSources: List<Any?>?, systemValues: SystemValues) {
            this.systemValues = systemValues
            sources.outsideSources = outsideSources
            processOnce()
        }

        fun getOutput(index: Int): Any? = outputs.extractFromSystem!![index]
    }

    private inner class Sources : AbstractBlock(0, numInputs, IN_FIRST_LAZY) {
        var outsideSources: List<Any?>? = null

        fun allOutputs() = List(this.numOutputs) { configOutput<Any?>(it) }
        override fun init() {
            outsideSources = null
        }

        override fun process(inputs: List<Any?>, systemValues: SystemValues) {}
        override fun getOutput(index: Int): Any? = outsideSources!![index]
    }

    private inner class Outputs : AbstractBlock(numOutputs, 0, IN_FIRST_ALWAYS) {
        var extractFromSystem: List<Any?>? = null
            private set

        fun allInputs() = List(this.numInputs) { configInput<Any?>(it) }
        override fun init() {
            extractFromSystem = null
        }

        override fun process(inputs: List<Any?>, systemValues: SystemValues) {
            extractFromSystem = inputs
        }

        override fun getOutput(index: Int): Any? {
            throw IndexOutOfBoundsException(index)
        }
    }
}
