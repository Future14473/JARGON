package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.config.BlockConfig
import org.futurerobotics.jargon.blocks.config.IllegalBlockConfigurationException
import org.futurerobotics.jargon.blocks.config.ReadOnlyBlockConfig
import org.futurerobotics.jargon.util.uncheckedCast

/**
 * Base implementation of [Block].
 *
 * Subclasses can call [newInput]/[newOutput] and store them as public (final) fields/properties during
 * construction/initialization. These can then be used by client code during configuration, and internally
 * during [process]. This provides a generics-safe method of customizing any number of inputs/outputs.
 *
 *
 */
abstract class BaseBlock(override val processing: Processing) : Block() {

    /**
     * Creates a new input to this block with the specified type [T], and given name [name].
     *
     * If passed [name] is null, will attempt to find name via reflection.
     */
    @JvmOverloads
    protected fun <T> newInput(name: String? = null): Input<T> = Input(name, false)

    /**
     * Creates a new input to this block with the specified type [T], that can also be [isOptional].
     *
     * If an input is optional and not connected, the given values will be `null`.
     *
     * If passed [name] is null, will attempt to find name via reflection.
     */
    @JvmOverloads
    protected fun <T> newInput(isOptional: Boolean, name: String? = null): Input<T?> = Input(name, isOptional)

    /**
     * Creates a new output to this block with the specified type [T].
     *
     * If passed [name] is null, will attempt to find name via reflection
     */
    @JvmOverloads
    protected fun <T> newOutput(name: String? = null): Output<T> = Output(name)
}

/**
 * A block that only takes one [input] of type [T] and one [output] of type [R], that is ran through a [pipe] function.
 */
abstract class PipeBlock<T, R>
@JvmOverloads constructor(
    override val processing: Processing = LAZY
) : Block() {

    /** Input to this [PipeBlock] */
    val input: Input<T> = Input(null, false)
    /** Output to this [PipeBlock] */
    val output: Output<R> = Output(null)

    /** Pipes the input to the output. */
    protected abstract fun Context.pipe(input: T): R

    final override fun Context.process() {
        output.set = pipe(input.get)
    }

    final override fun finalizeConfig(config: ReadOnlyBlockConfig) {
        super.finalizeConfig(config)
    }

    companion object {

        /**
         * Creates a new [PipeBlock] with the given [piping] function.
         */
        inline fun <T, R> with(crossinline piping: Context.(T) -> R): PipeBlock<T, R> = object : PipeBlock<T, R>() {
            override fun Context.pipe(input: T): R = piping(input)
        }
    }
}

/**
 * A block which has only one [input] of type [T], that is _not_ optional.
 */
abstract class SingleInputBlock<T> constructor(override val processing: Processing) : Block() {

    /** The input to this block. */
    @Suppress("LeakingThis")
    val input: Input<T> = Input(null, false)

    /**
     * Creates a new output to this block with the specified type [T].
     *
     * If passed [name] is null, will attempt to find name via reflection
     */
    @JvmOverloads
    protected fun <T> newOutput(name: String? = null): Output<T> = Output(name)

    /** Processes with the given [input] */
    protected abstract fun Context.process(input: T)

    final override fun Context.process() {
        this.process(input.get)
    }

    final override fun finalizeConfig(config: ReadOnlyBlockConfig) {
        super.finalizeConfig(config)
    }
}

/**
 * A block which has only one [output] of type [T].
 */
abstract class SingleOutputBlock<T>(override val processing: Processing) : Block() {

    /** The input to this block. */
    @Suppress("LeakingThis")
    val output: Output<T> = Output(null)

    /**
     * Creates a new input to this block with the specified type [T], and given name [name].
     *
     * If passed [name] is null, will attempt to find name via reflection.
     */
    @JvmOverloads
    protected fun <T> newInput(name: String? = null): Input<T> = Input(name, false)

    /**
     * Creates a new input to this block with the specified type [T], that can also be [isOptional].
     *
     * If an input is optional and not connected, the given values will be `null`.
     *
     * If passed [name] is null, will attempt to find name via reflection.
     */
    @JvmOverloads
    protected fun <T> newInput(isOptional: Boolean, name: String? = null): Input<T?> = Input(name, isOptional)

    /** Gets the output of this block. */
    protected abstract fun Context.getOutput(): T

    final override fun Context.process() {
        output.set = getOutput()
    }

    final override fun finalizeConfig(config: ReadOnlyBlockConfig) {
        super.finalizeConfig(config)
    }
}

/**
 * A block that contains within itself another block sub-system, configured in [configSubsystem]
 *
 * When this block is processed it will process the entire sub-system.
 *
 * The subsystem does not support [SpecialBlock]s.
 */
abstract class CompositeBlock(processing: Processing) : BaseBlock(processing) {

    private lateinit var subsystem: Subsystem
    private var outsideContext: Context? = null
    final override fun init() {
        subsystem.init()
    }

    final override fun Context.process() {
        subsystem.process(this)
    }

    final override fun stop() {
        subsystem.stop()
    }

    /**
     * Builds and returns a [BlockConfig] for the subsystem, using the given a [SubsystemMapper] to map _this_ blocks
     * inputs/outputs into the _subsystem's_ outputs.inputs.
     *
     * Try not to confuse inputs/outputs as that may result in errors.
     */
    protected abstract fun SubsystemMapper.configSubsystem(): BlockConfig

    final override fun finalizeConfig(config: ReadOnlyBlockConfig) {
        super.finalizeConfig(config)
        val mapper = object : SubsystemMapper {
            val outputsArray = arrayOfNulls<Output<*>>(numOutputs)
            private val outputs = Outputs(outputsArray.uncheckedCast())
            override fun <T> Input<T>.subOutput(): Output<T> {
                require(block === this@CompositeBlock) {
                    throw IllegalBlockConfigurationException(
                        "In composite block ${this@CompositeBlock}: " +
                                "Attempted to map from another block: ${this}"
                    )
                }
                return Source(this).subOutput
            }

            override fun <T> Output<T>.subInput(): Input<T> {
                require(block === this@CompositeBlock) {
                    throw IllegalBlockConfigurationException(
                        "In composite block ${this@CompositeBlock}: " +
                                "Attempted to map from another block: ${this}"
                    )
                }
                val index = index
                outputsArray[index] = this
                return outputs.subInputs[index].uncheckedCast()
            }
        }
        val subConfig = mapper.configSubsystem() //confirms that all outputs need to be connected.
        subsystem = Subsystem(subConfig)
    }

    /**
     * Maps a [CompositeBlock]'s inputs/outputs into the subsystem's outputs.inputs.
     */
    protected interface SubsystemMapper {

        /**
         * Gets the _subsystem_ output corresponding _this block's_ input.
         */
        fun <T> Input<T>.subOutput(): Output<T>

        /**
         * Gets the _subsystem_ input corresponding to _this block's_ output.
         */
        fun <T> Output<T>.subInput(): Input<T>
    }

    private inner class Subsystem(config: BlockConfig) :
        BlockRunner(config) {

        override val systemValues: SystemValues get() = outsideContext!!
        public override fun init() = super.init()
        public override fun stop() = super.stop()
        fun process(context: Context) {
            outsideContext = context
            processOnce()
        }
    }

    private inner class Source<T>(private val outsideInput: Input<T>) : BaseBlock(LAZY) {
        val subOutput = newOutput<T>()

        override fun Context.process() {
            subOutput.set = outsideContext!![outsideInput]
        }
    }

    private inner class Outputs(private val outsideOutputs: Array<Output<Any?>>) : BaseBlock(ALWAYS) {

        val subInputs = Array(this@CompositeBlock.numOutputs) { newInput<Any?>() }

        override fun Context.process() {
            repeat(subInputs.size) {
                outsideContext!![outsideOutputs[it]] = subInputs[it].get
            }
        }
    }
}
