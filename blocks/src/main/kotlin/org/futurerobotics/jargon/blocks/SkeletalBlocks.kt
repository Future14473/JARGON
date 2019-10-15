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
            if (!inputIsConnected(it))
                throw IllegalBlockConfigurationException("All inputs to ${this@AbstractBlock} must be connected.")
        }
    }

    /**
     * Gets a [BlocksConfig.Input] for this block, with the given [index].
     *
     * It is the users responsibility to make sure that the generic is of the right type.
     *
     * Subclasses should use this to provide inputs/outputs.
     * @see [configOutput]
     */
    protected open fun <T> configInput(index: Int): BlocksConfig.Input<T> {
        if (index !in 0..numInputs) throw IndexOutOfBoundsException(index)
        return BlocksConfig.Input.ofUnsafeCast(this, index)
    }


    /**
     * Gets a [BlocksConfig.Output] for this block, with the given [index].
     *
     * It is the users responsibility to make sure that the generic is of the right type.
     *
     * Subclasses should use this to provide inputs/outputs.
     * @see [configInput]
     */
    protected open fun <T> configOutput(index: Int): BlocksConfig.Output<T> {
        if (index !in 0..numOutputs) throw IndexOutOfBoundsException(index)
        return BlocksConfig.Output.ofUnsafeCast(this, index)
    }

}

/**
 * An implementation of a [Block] that stores _all_ its outputs in a list when processed.
 *
 * The list is provided in the [init]/[process] functions.
 */
abstract class ListStoreBlock(
    numInputs: Int,
    numOutputs: Int,
    processing: Block.Processing
) : AbstractBlock(numInputs, numOutputs, processing) {

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
     * Initializes this component; possibly filling the [outputs] if this component is [OUT_FIRST_ALWAYS].
     *
     * Otherwise, changes to the [outputs] list will be ignored.
     *
     * (as in [Block.init])
     */
    protected abstract fun init(outputs: MutableList<Any?>)

    /**
     * Processes this component, given the [inputs] as a list and the [outputs] as a mutable list.
     *
     * (as in [Block.process])
     */
    protected abstract fun process(
        inputs: List<Any?>,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    )
}

/**
 * A block that has a single output of type [R], which simplifies processing in [processOutput].
 *
 * This is also itself a [BlocksConfig.Output] representing its only output.
 */
abstract class SingleOutputBlock<R>(
    numInputs: Int,
    processing: Block.Processing
) : AbstractBlock(numInputs, 1, processing), BlocksConfig.Output<R> {

    private var value: R? = null

    final override fun init() {
        value = doInit()
    }

    /**
     * Initializes this block, and if this block is [OUT_FIRST_ALWAYS], returns the initial output
     *
     * Otherwise return value will be ignored so just return `null`
     *
     * (as in [Block.init])
     */
    protected abstract fun doInit(): R?

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

    final override val block: Block get() = this
    final override val index: Int get() = 0
}

/**
 * A block that has a single input of type [T], which simplifies processing in [processInput]
 *
 * This is also itself a [BlocksConfig.Input] representing its only input.
 */
abstract class SingleInputBlock<T>(
    numOutputs: Int,
    processing: Block.Processing
) : AbstractBlock(1, numOutputs, processing), BlocksConfig.Input<T> {
    final override fun process(inputs: List<Any?>, systemValues: SystemValues): Unit =
        processInput(inputs[0].unsafeCast(), systemValues)

    /**
     * Processes this block with the single [input] value
     *
     * (as in [Block.process])
     */
    protected abstract fun processInput(input: T, systemValues: SystemValues)

    final override val block: Block get() = this
    final override val index: Int get() = 0
}


/**
 * A block that has a single input of type [T], and stores _all_ its outputs in a list:
 * a combination of [ListStoreBlock] and [SingleInputBlock]
 */
abstract class SingleInputListStoreBlock<T>(
    numOutputs: Int,
    processing: Block.Processing
) : ListStoreBlock(1, numOutputs, processing), BlocksConfig.Input<T> {

    final override fun process(
        inputs: List<Any?>,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    ): Unit = processInput(inputs[0].unsafeCast(), systemValues, outputs)

    /**
     * Processes this component, given the single [input] and the [outputs] as a value
     *
     * (as in [Block.process] or [ListStoreBlock.process])
     */
    protected abstract fun processInput(
        input: T,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    )

    final override val block: Block get() = this
    final override val index: Int get() = 0
}

/**
 * A block that has 1 input of type [T] and 1 output of type [R], where the output is strictly the input run through
 * the [pipe] function.
 *
 * This also defaults [doInit] to return `null`; override if this is not desired.
 *
 * This itself if both a [BlocksConfig.Input] and [BlocksConfig.Output] representing its input and output.
 *
 * A lambda version of this is available in [BlocksConfig.pipe] for easier use.
 * */
abstract class Pipe<T, R>(processing: Block.Processing) : SingleOutputBlock<R>(
    1, processing
), BlocksConfig.Input<T> {

    final override fun doInit(): R? = null
    final override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): R = pipe(inputs[0].unsafeCast())

    /**
     * Transforms the input value to the output value.
     */
    protected abstract fun pipe(input: T): R

    companion object {
        /** Creates a [Pipe] using the given [pipe] function. */
        @JvmStatic
        inline fun <T, R> of(crossinline pipe: (T) -> R): Pipe<T, R> =
            object : Pipe<T, R>(IN_FIRST_LAZY) {
                override fun pipe(input: T): R = pipe(input)
            }

    }
}


/**
 * A block that has 2 inputs of types [A] and [B] and 1 output of type [R], where the output is a the input run
 * through the [combine] function.
 *
 * This also defaults [doInit] to return `null`; override if this is not desired.
 *
 * This is itself a [BlocksConfig.Output] representing its only output,
 * and also provides [first] and [second] values representing its inputs.
 *
 * A lambda version of this is available in [BlocksConfig.combine] for easier use.
 */
abstract class Combine<A, B, R> :
    SingleOutputBlock<R>(2, IN_FIRST_LAZY) {

    final override fun doInit(): R? = null
    final override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): R =
        combine(inputs[0].unsafeCast(), inputs[1].unsafeCast())

    /**
     * Combines two input values to the output value.
     */
    protected abstract fun combine(a: A, b: B): R

    /** The first input to this combine block. */
    val first: BlocksConfig.Input<A> get() = configInput(0)
    /** The second input to this second block. */
    val second: BlocksConfig.Input<B> get() = configInput(1)

    companion object {
        /** Creates a [Combine] using the given [combine] function. */
        @JvmStatic
        inline fun <A, B, R> of(
            crossinline combine: (A, B) -> R
        ): Combine<A, B, R> =
            object : Combine<A, B, R>() {
                override fun combine(a: A, b: B): R = combine(a, b)
            }

    }
}


/**
 * A block that contains within itself another block sub-system, configured in [buildSubsystem]
 *
 * When this block is processed it will process the entire sub-system.
 *
 * The subsystem does not support [SpecialBlock]s.
 */
//experimental
abstract class CompositeBlock(numInputs: Int, numOutputs: Int, processing: Block.Processing) :
    AbstractBlock(numInputs, numOutputs, processing) {
    private lateinit var subsystem: Subsystem
    override fun init(): Unit = subsystem.init()

    override fun process(inputs: List<Any?>, systemValues: SystemValues): Unit = subsystem.process(inputs, systemValues)

    override fun getOutput(index: Int): Any? = subsystem.getOutput(index)

    private inner class Subsystem(
        connections: Collection<BaseBlocksConfig.BlockConnections>,
        private val sources: Sources,
        private val outputs: Outputs
    ) : AbstractBlocksRunner(connections) {
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

    /**
     * Configures the sub-system using the given [BlocksConfig], the [BlocksConfig.Output] from the giving value [sources] from
     * the outside world, and [BlocksConfig.Input] for given [outputs] to the outside world.
     *
     * Note that these inputs/outputs only work in the given subsystem. Having blocks in both the input and the output
     * subsystems leads to undefined behavior.
     *
     *
     * note: using [BaseBlocksConfig] for inline functions for those who care
     */
    protected abstract fun BlocksConfig.buildSubsystem(
        sources: List<BlocksConfig.Output<Any?>>,
        outputs: List<BlocksConfig.Input<Any?>>
    )

    override fun prepareAndVerify(config: BlocksConfig) {
        subsystem = SubsystemConfig().run {
            buildSubsystem(sources.allOutputs(), outputs.allInputs())
            getSystem()
        }
        doVerifyConfig(config)
    }

    /** Same thing as [doVerifyConfig] */
    protected open fun doVerifyConfig(config: BlocksConfig) {
        super.prepareAndVerify(config)
    }

    private inner class SubsystemConfig : BaseBlocksConfig() {
        val sources = Sources()
        val outputs = Outputs()
        fun getSystem() = Subsystem(connections, sources, outputs)
    }
}



