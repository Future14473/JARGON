package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Context
import org.futurerobotics.jargon.blocks.Block.Processing
import org.futurerobotics.jargon.blocks.Block.Processing.*
import kotlin.reflect.KClass
import kotlin.reflect.KProperty
import kotlin.reflect.KVisibility
import kotlin.reflect.full.isSubclassOf
import kotlin.reflect.jvm.javaType

/**
 * # Blocks
 * This system was inspired by making block diagrams more literal.
 *
 * A [Block] can represent anything with a notion of _inputs_ or _outputs_: any process, calculation, value,
 * measurement, interaction, etc.
 *
 * A [Block] can have any number of inputs/outputs, including 0. Each are represented using [Block.Input] and
 * [Block.Output], and are each associated with a given class/type. The outputs of blocks can then be connected to the
 * inputs of others in a way you might see in at a block diagram.
 *
 * An arrangement of connected blocks is a [BlockArrangement], and can be created using a [BlockArrangementBuilder]
 * (and its DSL).
 *
 * ## Running blocks and Block Systems.
 *
 * A [BlockArrangement] can be passed into a [BlockSystem] that runs it, (see "Configuring blocks" below), where
 * every block will be processed in a series of _loops_. Generally"
 *
 * - [init] and [stop] are called when an entire system first starts/stops.
 * - [process] will then (maybe) be called every loop, with a [Context] interface providing a way to get and set
 * the actual values of inputs/outputs. [Context] is also a sub-interface of [SystemValues], providing other
 * values based on the system run.
 *
 * The exact way a block is processed is defined by its [processing]; see there for more details.
 *
 * There cannot be a loop of dependencies among blocks when the system is run(that do _not_ have a processing of
 * [OUT_FIRST]): if A wants values from B, but B wants values from A, there is no way to process any of the blocks.
 *
 * ## Creating a block implementation
 * Subclass a [Block], and then define inputs/outputs using [newInput]/[newOptionalInput]/[newOutput] which creates
 * [Block.Input]s and [Block.Output]s, and store these in public (final) fields/properties. These should only be
 * created during construction (in the constructor or initializer). Client code can then use these inputs/outputs to
 * link blocks together
 *
 * [PrincipalOutputBlock], and [PipeBlock] also exists for convenience with one `main` output.
 *
 * [CompositeBlock] is a block that is made of an entire subsystem of blocks.
 *
 * By default, all inputs must be connected for a block to function. One can instead create _optional_ inputs
 * using [newOptionalInput], in which case the value of `null` will be given for not connected inputs (will be
 * `nullable` in kotlin). Note that blocks can also possibly output values of `null`.
 *
 * ## Other
 *
 * There are also [SpecialBlock]s which receive special treatment from [BlockSystem].
 *
 * ## Groups
 *
 * One can create a block that contains another block system using a [CompositeBlock].
 * Alternatively, one can use the idiom of having a class that takes a [BlockArrangementBuilder] as
 * a constructor parameter, uses it to create and configure a group of blocks, and exposes [Block.Input]
 * and [Block.Output]s for the world to see.
 *
 * @see SystemValues
 * @see BlockArrangementBuilder
 *
 *
 * @property processing The [Processing] of this block.
 */
abstract class Block(val processing: Processing) {

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
     * If passed [name] is null, name will be attempted to be found via reflection.
     */
    @JvmOverloads
    protected fun <T> newInput(name: String? = null): Input<T> =
        Input(name, false)

    /**
     * Creates a new input to this block with the specified type [T], that can also be [isOptional].
     *
     * If an input is optional and not connected, the values returned will be `null`.
     *
     * If passed [name] is null, name will be attempted to be found via reflection.
     */
    @JvmOverloads
    protected fun <T> newOptionalInput(name: String? = null, isOptional: Boolean = true): Input<T?> =
        Input(name, isOptional)

    /**
     * Creates a new output to this block with the specified type [T].
     *
     * If passed [name] is null, name will be attempted to be found via reflection.
     */
    @JvmOverloads
    protected fun <T> newOutput(name: String? = null): Output<T> = Output(name)

    /** Common components of [Input] and [Output]. */
    abstract inner class AnyIO<T> internal constructor(private val _name: String?, internal val index: Int) {

        /** The [Block] that this input/output belongs to */
        val block: Block get() = this@Block

        init {
            check(!finalized) { "Block is already used in a config, cannot create new ${javaClass.simpleName}" }
        }

        private val reflectProperty: KProperty<*>? by lazy {
            ioProps.find { prop ->
                prop.call(this@Block) === this
            }
        }
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
         * This includes the type and name of this block, _if_ they can be found via reflection by looking up fields.
         */
        abstract fun name(): String

        /**
         * A name that includes:
         * - The name of the block that his belongs to.
         * - The type of this input/output (if can be found via reflection)
         * - The name of this input/output
         */
        override fun toString(): String = "${this@Block}: ${name()}"
    }

    /**
     * Represents an input to a block.
     *
     * This can be created within blocks using [newInput]/[newOptionalInput], and is used to configure blocks.
     *
     * @property isOptional if this input is optional, and will not complain when not connected.
     */
    inner class Input<T> internal constructor(name: String?, val isOptional: Boolean) : AnyIO<T>(name, numInputs) {

        init {
            inputs += this
        }

        override fun name(): String {
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
     * This can be created within blocks using [newOutput], and is used to configure blocks.
     *
     * During process, _all_ outputs have to be set to a value.
     */
    inner class Output<T> internal constructor(name: String?) : AnyIO<T>(name, numOutputs) {

        init {
            check(!finalized) { "Block is already used in a config, cannot create new output" }
            outputs += this
        }

        override fun name(): String {
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
     * - [LAZY]: only process if another block requires this block's outputs
     * - [ALWAYS]: always process
     * - [OUT_FIRST]: allow getting outputs before processing (first process will have inputs be null)
     *
     * These options allow for more dynamic behavior, as blocks may not be run every loop.
     *
     * At least one block in a component system must be _not_ [LAZY] since if everyone is lazy nobody
     * will process.
     */
    enum class Processing {

        /**
         * A block with [LAZY] processing will only [process] if another blocks requests its outputs,
         * via calling `get` on [Context] that links to this block.
         *
         * Blocks that do nothing more than process their inputs directly into output without storing information
         * should have this kind of processing.
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
     * Processes the current block. Get inputs and set outputs via the provided [Context].
     *
     * Calling [Context.get] on an input will ensure that the block that sources that input is processed first.
     *
     * _All_ outputs this block has must be set to a value when processed.
     *
     * There cannot be a loop of blocks in which
     *
     * Also, values given by [SystemValues] are supported.
     */
    abstract fun Context.process()

    /** A block context. Used to get or set block values. Also supports values from [SystemValues] */
    interface Context : SystemValues {

        /** Gets the current value of an [input] to this block. */
        operator fun <T> get(input: Input<T>): T = input.get

        /** Sets the [value] of an [output] to this block. */
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
     * This should be used with caution.
     *
     * This is meant to ease creating "quick" calculations when creating block systems.
     */
    interface ExtendedContext : Context {

        /** Gets the [output] of _another_ block directly, bypassing connections. Use with caution. */
        operator fun <T> get(output: Output<T>): T = output.get

        /** Gets the output of _another_ block directly, bypassing connections. Use with caution. */
        @get:JvmSynthetic
        val <T> Output<T>.get: T
    }

    /**
     * Called when the _entire_ system shuts down. Meant for stopping or cleanup.
     */
    open fun stop() {}

// --- other ---

    /**
     * This is called when a block arrangement is built. Custom checks of connectivity can be done here, and
     * may throw [IllegalBlockConfigurationException].
     */
    open fun checkConfig(arrangement: ReadOnlyBlockArrangement) {}

    /**
     * Is set to true when this block exists in a built [BlockArrangement].
     * After that it can not be added to another config.
     */
    var finalized: Boolean = false
        private set

    /**
     * Verifies that the current configuration on the given [BlockArrangement] is valid (non-optional inputs will be
     * must be connected).
     */
    internal open fun finalizeConfig(arrangement: ReadOnlyBlockArrangement) {
        assert(this in arrangement) { "Block not in the given config" }
        check(!finalized) { "Block already used in another config" }
        finalized = true
        inputs.forEach {
            if (!it.isOptional && it !in arrangement)
                throw IllegalBlockConfigurationException("Non-optional input $it is not connected. ")
        }
        checkConfig(arrangement)
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
    /** The time in nanoseconds the last loop has taken to run. */
    val loopTimeInNanos: Long
    /** The total amount of time in second elapsed since the system has started. */
    val totalTime: Double
    /** The total amount of time in nanoseconds elapsed since the system has started. */
    val totalTimeInNanos: Long
    /** The number of the current loop since the system has started, starting at 0. */
    val loopNumber: Int

    /**
     * If this is the first time this block is being processed this loop.
     *
     * Usually used in [Processing.OUT_FIRST].
     */
    val isFirstTime: Boolean
        get() = loopNumber == 0
}
