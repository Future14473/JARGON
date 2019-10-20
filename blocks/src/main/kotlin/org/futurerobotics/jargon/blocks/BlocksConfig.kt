package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.BaseBlocksConfig.BlockConnections
import org.futurerobotics.jargon.blocks.BlocksConfig.Input
import org.futurerobotics.jargon.blocks.BlocksConfig.Input.Companion.of
import org.futurerobotics.jargon.blocks.BlocksConfig.Input.Companion.ofUnsafeCast
import org.futurerobotics.jargon.blocks.BlocksConfig.Output
import org.futurerobotics.jargon.blocks.BlocksConfig.Output.Companion.of
import org.futurerobotics.jargon.blocks.BlocksConfig.Output.Companion.ofUnsafeCast
import org.futurerobotics.jargon.util.asUnmodifiableList
import org.futurerobotics.jargon.util.asUnmodifiableMap
import org.futurerobotics.jargon.util.let
import java.util.*

/**
 * An exception thrown when an illegal configuration is detected.
 */
class IllegalBlockConfigurationException
@JvmOverloads constructor(message: String? = null) : RuntimeException(message)


/**
 * Coordinates the connection of blocks together to build a system, using a DSL. This can then, for instance,
 * be passed on to a [BlocksSystem],
 *
 * This primarily works with [Input]s and [Output]s, which reference specific inputs and outputs of
 * blocks, usually provided by [Block]s themselves.
 *
 * See the individual functions/methods of this class for more details.
 *
 * This is an mostly abstract class instead of an interface to make use of certain inline functions.
 *
 * FOR KOTLIN, make a context with `this` as this config to make use of extension functions, for example using
 * [kotlin.apply] or [kotlin.run], or in specifically making [BlocksSystem], [buildBlocksSystem].
 *
 * @see [BaseBlocksConfig]
 * @see [BlocksSystem]
 */
abstract class BlocksConfig {

    /**
     * Returns if this block is present in this config;
     * in other words there is at least one connection to or from it within this config.
     */
    abstract fun Block.isPresent(): Boolean

    /**
     * Gets a map of blocks to [Connections] of that block.
     */
    abstract val connections: Map<Block, Connections>

    /** Verifies all the current block configurations by calling [Block.prepareAndVerify] on all blocks present. */
    abstract fun verifyConfig()

    /**
     * Connects the given [Output] _into_ the given [Input].
     *
     * An input can only be connected once; throws [IllegalStateException] if otherwise.
     */
    abstract infix fun <T> Output<T>.into(input: Input<T>)

    /**
     * Connects the given [Input] _from_ the given [Output]
     *
     * An input can only be connected once; throws [IllegalStateException] if otherwise.
     */
    open infix fun <T> Input<T>.from(output: Output<T>): Unit = output into this

    /** Connects [this] [Output] to all of the given [inputs]. */
    open fun <T> Output<T>.toAll(vararg inputs: Input<T>) {
        inputs.forEach {
            this into it
        }
    }

    /** Returns if the block's input with the given [index] is connected. */
    abstract fun Block.inputIsConnected(index: Int): Boolean

    /** Returns if the block's output with the given [index] is connected. */
    abstract fun Block.outputIsConnected(index: Int): Boolean

    /** Returns if the given [Input] is connected. */
    open fun Input<*>.isConnected(): Boolean = block.inputIsConnected(index)

    /** Returns if the given [Output] is connected. */
    open fun Output<*>.isConnected(): Boolean = block.outputIsConnected(index)

    /** Returns where the input to the block by the given [index] is connected from, or `null` if not connected. */
    abstract fun Block.sourceOfInput(index: Int): Output<*>?

    /** Gets where this [this] input is connected from, or `null` if not connected. */
    abstract fun <T> Input<T>.source(): Output<T>?

    /**
     * Creates and connects a [Monitor] from [this] [Output], and returns it.
     *
     * Note that if the block has a processing of [Block.Processing.IN_FIRST_LAZY], it will now always be processed.
     */
    open fun <T> Output<T>.monitor(): Monitor<T> = Monitor<T>().also { this into it }


    /**
     * Creates and connects a [Monitor] from the [source] of [this] [Input], and returns it.
     *
     * *** The input must first be connected in order to find the source ***
     *
     * Note that if the source has a processing of [Block.Processing.IN_FIRST_LAZY], it will now always be processed.
     */
    open fun <T> Input<T>.monitor(): Monitor<T> = source()?.monitor()
        ?: throw IllegalStateException("Input has not yet been connected, cannot deduce source")

    /**
     * Runs the given the given [operation] function on this output loop. Useful for quick operations; but use wisely.
     *
     * Note that if the source has a processing of [Block.Processing.IN_FIRST_LAZY], it will now always be processed.
     */
    inline fun <T> Output<T>.listen(crossinline operation: (T) -> Unit) {
        this into InputOnlyBlock.of(operation)
    }

    /**
     * Create a [Delay] block with the given [initialValue], connects this output into it, and returns the delay's
     * output. Can be used to break up loop.s
     */
    fun <T> Output<T>.delay(initialValue: T): Output<T> = Delay(initialValue).also { this into it }

    /**
     * Adds the given [pipeBlock], connects [this] input, and returns the pipe's output.
     *
     * Useful for quick transformations.
     */
    fun <T, R> Output<T>.pipe(pipeBlock: PipeBlock<T, R>): Output<R> = pipeBlock.also { this into it }

    /**
     * Creates a [PipeBlock] block that pipes this output through the given [transform] function, connects
     * it, and returns the pipe's output.
     *
     * Useful for quick transformations.
     */
    inline fun <T, R> Output<T>.pipe(crossinline transform: T.() -> R): Output<R> =
        PipeBlock.of(transform).also { this into it }

    /**
     * Adds the given [combineBlock], connects [this] and [other] to its first and second inputs, and returns the
     * combine's output.
     *
     * Useful for quick transformations.
     */
    fun <A, B, R> Output<A>.combine(other: Output<B>, combineBlock: CombineBlock<A, B, R>): Output<R> =
        combineBlock.also { this into it.firstInput; other into it.secondInput }

    /**
     * Creates a [CombineBlock] block that combines [this] and [other] outputs through the given [combine] function with the
     * value of [this] as receiver, and returns the combination's output.
     */
    inline fun <A, B, R> Output<A>.combine(other: Output<B>, crossinline combine: A.(B) -> R): Output<R> =
        CombineBlock.of(combine).also { this into it.firstInput; other into it.secondInput }

    /** Runs the [configuration] block on `this`, then returns it. kotlin DSL. */
    inline operator fun <T : Block> T.invoke(configuration: T.() -> Unit): T = apply(configuration)

    /**
     * Common interface for [Input] and [Output]; use those.
     */
    interface BlockIO {
        /** The block that this input/output comes from */
        val block: Block
        /** The index of this input/output. */
        val index: Int
    }

    /**
     * Represents a specific __input__ of a specific block; used in configuration in [BlocksConfig].
     *
     * Usually [Block]s themselves should provide specific ways of getting [Input]/[Output] instead of manually creating
     * one here.
     *
     * It is not recommended to create instances of this outside of from a [Block], else types cannot be enforced.
     *
     * However, if necessary, [of] and [ofUnsafeCast] are provided.
     */
    interface Input<in T> : BlockIO {
        companion object {
            /**
             * Creates a [Input] by providing values. It is stated explicitly that casts are allowed.
             * @see of
             */
            fun <T> ofUnsafeCast(block: Block, index: Int): Input<T> {
                if (index !in 0 until block.numInputs)
                    throw IndexOutOfBoundsException("Index $index not out block's number of inputs ${block.numInputs}")
                return object : Input<T> {
                    override val block: Block get() = block
                    override val index: Int get() = index
                }
            }

            /**
             * Creates a [Input] using the type specified in [Block], but type is not known at compile time.
             * @see ofUnsafeCast
             */
            fun of(block: Block, index: Int): Input<*> = ofUnsafeCast<Any>(block, index)
        }
    }

    /**
     * Represents a specific __output__ of a specific block; used in configuration in [BlocksConfig]
     *
     * Usually [Block]s themselves should provide specific ways of getting [Input]/[Output] instead of manually creating
     * one here.
     *
     * It is not recommended to create instances of this outside of from a [Block], else types cannot be enforced.
     *
     * However, if necessary, [of] and [ofUnsafeCast] are provided.
     */
    interface Output<out T> : BlockIO {
        companion object {
            /**
             * Creates a [Output] by providing values. It is stated explicitly that casts are allowed.
             * @see of
             */
            fun <T> ofUnsafeCast(block: Block, index: Int): Output<T> {
                if (index !in 0 until block.numOutputs)
                    throw IndexOutOfBoundsException("Index $index not in block's number of outputs ${block.numOutputs}")
                return object : Output<T> {
                    override val block: Block get() = block
                    override val index: Int get() = index
                }
            }

            /**
             * Creates a [Output] using the type specified in [Block], but type is not known at compile time.
             * @see ofUnsafeCast
             */
            fun of(block: Block, index: Int): Output<*> = ofUnsafeCast<Any?>(block, index)
        }
    }

    /** Represents configured connections into and out of a block. */
    interface Connections {
        /** The block that this [Connections] is representing. */
        val block: Block
        /**
         * The sources (other block's outputs) to which are connected to the inputs of this block, or `null`
         * if not connected, with the same index as the block input.
         *
         * Should be same size as [Block.numInputs],
         */
        val sources: List<Output<*>?>

        /**
         * A list of booleans that represent if the output of the current block is connected, with the same index
         * as the block output.
         *
         * Should be same size as [Block.numOutputs]
         */
        val isOutputConnected: List<Boolean>
    }
}

/**
 * A base implementation of [BlocksConfig] that creates a set of [BlockConnections], which can
 * then be further processed.
 */
open class BaseBlocksConfig : BlocksConfig() {
    private val connectionsMap: IdentityHashMap<Block, BlockConnections> = IdentityHashMap()
    override val connections: Map<Block, Connections> = connectionsMap.asUnmodifiableMap()

    /**
     * Lets every block [verifyConfig][Block.prepareAndVerify] itself.
     */
    override fun verifyConfig() {
        connections.forEach { (block) ->
            block.prepareAndVerify(this)
        }
    }

    override fun Block.isPresent(): Boolean = connectionsMap.containsKey(this)

    private val Block.connection: BlockConnections
        get() = connectionsMap.getOrPut(this) {
            BlockConnections(
                this
            )
        }
    private val BlockIO.connection: BlockConnections get() = block.connection

    override fun <T> Output<T>.into(input: Input<T>) {
        let(this.connection, input.connection) { from, into ->
            check(into._sources[input.index] == null)
            { "Input at block (${into.block}) index (${input.index}) already connected!" }
            into._sources[input.index] = this
            from._isOutputConnected[index] = true
        }
    }

    override fun Block.inputIsConnected(index: Int): Boolean = sourceOfInput(index) != null

    override fun Block.outputIsConnected(index: Int): Boolean =
        connectionsMap[this]?._isOutputConnected?.get(index) == true

    override fun Block.sourceOfInput(index: Int): Output<*>? = connectionsMap[this]?._sources?.get(index)

    @Suppress("UNCHECKED_CAST")
    override fun <T> Input<T>.source(): Output<T>? = block.sourceOfInput(index) as? Output<T>?

    @Suppress("PropertyName")
    private class BlockConnections(override val block: Block) : Connections {
        internal val _sources: Array<Output<*>?> = arrayOfNulls(block.numInputs)

        override val sources: List<Output<*>?>
            get() = _sources.asList().asUnmodifiableList()

        internal val _isOutputConnected: BooleanArray = BooleanArray(block.numOutputs)

        override val isOutputConnected: List<Boolean>
            get() = _isOutputConnected.asList().asUnmodifiableList()
    }
}
