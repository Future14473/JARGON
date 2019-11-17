package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Context
import org.futurerobotics.jargon.blocks.Block.Processing
import org.futurerobotics.jargon.blocks.Block.Processing.*
import org.futurerobotics.jargon.blocks.config.BCBuilder
import org.futurerobotics.jargon.blocks.config.BlockConfig
import org.futurerobotics.jargon.blocks.config.IllegalBlockConfigurationException
import org.futurerobotics.jargon.blocks.config.ReadOnlyBlockConfig
import kotlin.reflect.KClass
import kotlin.reflect.KProperty
import kotlin.reflect.KVisibility
import kotlin.reflect.full.isSubclassOf
import kotlin.reflect.jvm.javaType

/**
 * # Blocks
 * This system was inspired by making block diagrams more literal.
 *
 * The root of the blocks system is the [Block]. A [Block] can represent anything with a notion of _inputs_
 * or _outputs_: any process, calculation, value, measurement, interaction, etc.
 *
 * A [Block] can have any number of _input_ and _outputs_, including 0, defined by [numInputs] and
 * [numOutputs]. Each input and output is usually associated with a given _type/class_. The outputs of some blocks
 * can then be connected to the inputs of others, in a way you might look at a block diagram.
 *
 * An arrangement of connected blocks is a [BlockConfig], and can be created using a [BCBuilder] (with dsl).
 *
 * ## Running blocks and Block Systems.
 *
 * A [BlockConfig] can then be converted into a [BlockSystem] that runs it, (see "Configuring blocks" below), where
 * ever block will be processed in a series of _loops_ (discretely). In every loop, blocks will (maybe) be processed
 * and have input/output values transferred accordingly.
 *
 * - [init] and [stop] are called when an entire system first starts/stops.
 * - [process] will then (maybe) be called every loop, with a [Context] interface providing a way to get inputs to
 * blocks.
 *
 * The exact way a block is processed is defined by its [processing]; see there for more details.
 *
 * ## Creating a block implementation
 * One should usually subclass [BaseBlock] for a generic multi-input, multi-output block, but [PrincipleInputBlock],
 * [PrincipalOutputBlock], and [PipeBlock] also exists for convenience with blocks with only one input/output.
 * Within these blocks, one can define inputs/outputs using [newOptionalInput]/[newOutput] which creates [Block.Input]
 * and [Block.Output] and store these in public fields. Client code can then use these to link configuration together.
 *
 * By default, all inputs must be connected for a block to function. One can instead create _optional_ inputs
 * using [newOptionalInput], in which case the value of `null` will be given for not connected inputs (will be `nullable` in
 * kotlin). Note that blocks can also possibly output these values too.
 *
 * The actual getting and setting of a block's inputs/outputs during [process] is provided through the [Context]
 * interface, using since [Block.Input] and [Block.Output] have generics, this provides a type safe yet type generic
 * way to get inputs/outputs.
 *
 * ## Other
 *
 * There are also [SpecialBlock]s which receive special treatment from [BlockSystem].
 *
 * @property processing The [Processing] of this block.
 */
abstract class Block(val processing: Processing) : BlockIndicator {

    // --- inputs/outputs ---
    internal val inputs: MutableList<Input<*>> = ArrayList()
    internal val outputs: MutableList<Output<*>> = ArrayList()
    private val ioProps: List<KProperty<*>> by lazy {
        this::class.members
            .filterIsInstance<KProperty<*>>()
            .filter { it.visibility == KVisibility.PUBLIC }
            .filter { prop ->
                (prop.returnType.classifier as? KClass<*>)?.isSubclassOf(AnyIO::class) ?: false
            }
    }

    /** The number of inputs to this block. */
    val numInputs: Int get() = inputs.size
    /** The number of outputs to this block. */
    val numOutputs: Int get() = outputs.size

    /**
     * Creates a new input to this block with the specified type [T], and given name [name].
     *
     * If passed [name] is null, will attempt to find name via reflection.
     */
    @JvmOverloads
    protected fun <T> newInput(name: String? = null): Input<T> =
        Input(name, false)

    /**
     * Creates a new input to this block with the specified type [T], that can also be [isOptional].
     *
     * If an input is optional and not connected, the values returned will be `null`.
     *
     * If passed [name] is null, will attempt to find name via reflection.
     */
    @JvmOverloads
    protected fun <T> newOptionalInput(name: String? = null, isOptional: Boolean = true): Input<T?> =
        Input(name, isOptional)

    /**
     * Creates a new output to this block with the specified type [T].
     *
     * If passed [name] is null, will attempt to find name via reflection
     */
    @JvmOverloads
    protected open fun <T> newOutput(name: String? = null): Output<T> = Output(name)

    /** Common components of [Input] and [Output]. */
    abstract inner class AnyIO<T> internal constructor(name: String?, internal val index: Int) {

        /**  block the [Block] that this input belongs to */
        val block: Block get() = this@Block

        init {
            check(!finalized) { "Block is already used in a config, cannot create new ${javaClass.simpleName}" }
        }

        private val reflectProperty: KProperty<*>? by lazy {
            ioProps.find { prop ->
                prop.call(this@Block) === this
            }
        }
        private val _name: String? = name
        /**
         * The name of this input/output; either given explicitly, or attempted to find via reflection,
         * else 'null' if both fails.
         */
        val name: String? get() = _name ?: reflectProperty?.name

        private val type by lazy {
            reflectProperty?.returnType?.arguments?.firstOrNull()?.type
        }
        /**
         * The type of this input, found via reflection, rendered as a kotlin type.
         *
         * 'null' if reflection fails.
         */
        val typeName: String? get() = type?.toString()
        /**
         * The type of this input, found via reflection, rendered as a java type.
         *
         * 'null' if reflection fails.
         */
        val javaTypeName: String? get() = type?.javaType?.typeName

        /**
         * A string representation of this input/output shorter than [toString].
         *
         * This includes the type and name of this block, _if_ it can be found via reflection by looking up fields.
         */
        abstract fun shortName(): String

        /**
         *  A name that includes the block that this belongs to's name, and the type and name of this input/output
         *  (if found by reflection).
         */
        override fun toString(): String = "${this@Block}: ${shortName()}"
    }

    /**
     * Represents an input to a block.
     *
     * @property isOptional if this input is optional, and will not complain when not connected.
     */
    inner class Input<T> internal constructor(name: String?, val isOptional: Boolean) : AnyIO<T>(name, numInputs) {

        init {
            inputs += this
        }

        override fun shortName(): String {
            val name = name ?: "??, index=$index"
            val typeName = typeName?.let {
                if (javaTypeName != null && it != javaTypeName) "$it/$javaTypeName" else it
            } ?: "??"
            return "Input<$typeName>[name=$name, isOptional=$isOptional]"
        }
    }

    /**
     * Represents an output from a block.
     *
     * @property index the index of this output; used internally
     */
    inner class Output<T> internal constructor(name: String?) : AnyIO<T>(name, numOutputs) {

        init {
            check(!finalized) { "Block is already used in a config, cannot create new output" }
            outputs += this
        }

        /** A shorter name representation of this block than [toString]. */
        override fun shortName(): String {
            val name = name ?: "??, index=$index"
            val typeName = typeName?.let {
                if (javaTypeName != null && it != javaTypeName) "$it/$javaTypeName" else it
            } ?: "??"
            return "Output<$typeName>[name=$name]"
        }
    }

// --- processing ---

    /**
     * Defines how this component is run, which can be:
     *
     * - [LAZY]
     * - [ALWAYS]
     * - [OUT_FIRST]
     *
     * These options allow for more dynamic behavior, as blocks may not be run every loop.
     *
     * At least one block in a component system must be _not_ [LAZY] since if everyone is lazy nobody
     * will process.
     */
    enum class Processing {

        /**
         * A block with [LAZY] processing will only [process] if another blocks requests its outputs
         * by calling `get` that sources to this block, otherwise will not process.
         *
         * Blocks that do nothing more than process their inputs directly into output without storing information
         * of any kind should have this processing.
         * @see [Processing]
         */
        LAZY,
        /**
         * A block with [ALWAYS] processing will always be [process]ed every loop.
         *
         * Blocks that require that it receives information every single loop should have this kind of processing.
         * @see [Processing]
         */
        ALWAYS,
        /**
         * A block with [OUT_FIRST] processing will only receive inputs of the _previous_ loop when called on
         * [process]. The first time the block is processed, all inputs may be `null` **even if not marked nullable**.
         * **A block with [OUT_FIRST] will always request _every single input, every loop_**. This is (one of)
         * the only ways to prevent values for next cycle being given out instead.
         *
         * At least one block in a loop of blocks must be [OUT_FIRST]; For example a block that directly
         * interacts with hardware or external sources to be this kind of processing since measurements (outputs) are
         * usually taken _before_ signal (inputs)
         * @see [Processing]
         */
        OUT_FIRST;
        //There is no OUT_FIRST_LAZY since that causes problems and is rarely needed.
    }

    /**
     * Called when the _entire_ system first starts. Meant for initialization.
     */
    open fun init() {}

    /**
     * Processes the current block. Input/output interface is given by [Context]. Also, values given
     * by [SystemValues] are supported.
     */
    abstract fun Context.process()

    /** A block context. Used to get or set block values. Also supports values from [SystemValues] */
    interface Context : SystemValues {

        /**
         * If this is the first time this block is being processed this loop.
         *
         * Usually used in [Processing.OUT_FIRST]
         */
        val isFirstTime: Boolean

        /** Gets the current value of an [input] to this block. */
        @JvmDefault
        operator fun <T> get(input: Input<T>): T = input.get

        /** Sets the [value] of an [output] to this block. */
        @JvmDefault
        operator fun <T> set(output: Output<T>, value: T) {
            output.set = value
        }

        /** Gets the current value of an input to this block. */
        @get:JvmSynthetic
        val <T> Input<T>.get: T

        /**
         * Sets the value of an output to this block.
         *
         * The getter of this property is deprecated and should throw an UnsupportedOperationException.
         */
        @get:JvmSynthetic
        @get:Deprecated("Get not allowed.", level = DeprecationLevel.HIDDEN)
        @set:JvmSynthetic
        var <T> Output<T>.set: T
    }

    /**
     * An extended block context that also supports getting the outputs of other values in a system.
     * This should be used with caution, and should not be used in actually supported blocks.
     */
    interface ExtendedContext : Context {

        /** Gets the current value of an [output] to this block. */
        @JvmDefault
        operator fun <T> get(output: Output<T>): T = output.get

        /**
         * Gets the output of _another_ block directly, bypassing connections. Use with caution.
         */
        @get:JvmSynthetic
        val <T> Output<T>.get: T
    }

    /**
     * Called when the _entire_ system shuts down. Meant for stopping or cleanup.
     */
    open fun stop() {}

// --- other ---

    /**
     * May perform additional checks of connections of this block here. Called when a block config
     * is built.
     */
    open fun checkConfig(config: ReadOnlyBlockConfig) {}

    /**
     * Is set to true when this block exists in a built [BlockConfig].
     * After that it can not be added to another config.
     * If you do things right you don't have to worry about it.
     */
    var finalized: Boolean = false
        private set

    /**
     * Verifies that the current configuration on the given [BlockConfig] is valid (non-optional inputs will be
     * must be connected).
     */
    internal open fun finalizeConfig(config: ReadOnlyBlockConfig) {
        assert(this in config) { "Block not in the given config" }
        check(!finalized) { "Block already used in another config" }
        finalized = true
        inputs.forEach {
            if (!it.isOptional && it !in config)
                throw IllegalBlockConfigurationException("Non-optional input $it is not connected. ")
        }
        checkConfig(config)
    }

    override fun toString(): String = javaClass.simpleName.ifEmpty { "Anonymous Block" }
}

/**
 * Represents special values given to [Block]s that tap into the life of a [BlockSystem] itself, and
 * so are special input values. This is a supertype of [Block.Context].
 */
interface SystemValues {

    /** The time in seconds the last loop has taken to run. */
    val loopTime: Double
    /** The time in nanoSeconds the last loop has taken to run. */
    val loopTimeInNanos: Long
    /** The total amount of time elapsed since the block has first run. */
    val totalTime: Double
    /** The total time in nanoSeconds the last loop has taken to run. */
    val totalTimeInNanos: Long
    /** The number of the current loop run since `init`, starting from 0. */
    val loopNumber: Int
}

/**
 * Used to represent that an interface is supposed to represent a block. This is
 * only so that it can be recognized by the [BCBuilder.invoke], while being an interface.
 */
interface BlockIndicator
