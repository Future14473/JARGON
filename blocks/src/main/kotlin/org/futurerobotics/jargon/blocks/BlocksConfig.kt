package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.BaseBlocksConfig.BlockConnections
import org.futurerobotics.jargon.blocks.BlocksConfig.Input
import org.futurerobotics.jargon.blocks.BlocksConfig.Input.Companion.of
import org.futurerobotics.jargon.blocks.BlocksConfig.Input.Companion.ofUnsafeCast
import org.futurerobotics.jargon.blocks.BlocksConfig.Output
import org.futurerobotics.jargon.blocks.BlocksConfig.Output.Companion.of
import org.futurerobotics.jargon.blocks.BlocksConfig.Output.Companion.ofUnsafeCast
import org.futurerobotics.jargon.util.let
import java.util.*

/**
 * An exception thrown when an illegal configuration is detected.
 */
class IllegalBlockConfigurationException
@JvmOverloads constructor(message: String? = null) : RuntimeException(message)

/**
 * Coordinates the fitting together blocks together to build a system using a DSL.
 *
 * This primarily works with [Input]s and [Output]s, which reference specific inputs and outputs of
 * blocks, usually provided by [Block]s themselves.
 *
 * See the individual functions/methods of this class for more details.
 *
 * [BlocksSystemBuilder] is a subclass of this.
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
     *  Returns if this block is present in this config;
     *  in other words there is at least one connection to or from it within this config.
     */
    abstract fun Block.isPresent(): Boolean

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

    /**
     * Connects the inputs of [this] block to all the given [outputs], in order.
     *
     * May throw [ClassCastException] if types are not compatible.
     */
    @Suppress("UNCHECKED_CAST")
    @Deprecated("Non checked types; causes too many problems.")
    open fun Block.fromAll(vararg outputs: Output<*>) {
        require(outputs.size <= this.numInputs)
        { "the given number of outputs ${outputs.size} must not exceed the block's number of inputs $this.numInputs" }
        outputs.forEachIndexed { index, output ->
            output into Input.of(this, index) as Input<Any?>
        }
    }

    /**
     * Connects [this] [Output] to all of the given [inputs].
     */
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
     * Create a [Delay] block with the given [initialValue], connects this output into it, and returns the delay's
     * output.
     */
    fun <T> Output<T>.delay(initialValue: T): Output<T> = Delay(initialValue).also { this into it }

    /**
     * Adds the given [pipeBlock], connects [this] input, and returns the pipe's output.
     *
     * Useful for quick transformations.
     */
    fun <T, R> Output<T>.pipe(pipeBlock: Pipe<T, R>): Output<R> = pipeBlock.also { this into it }

    /**
     * Creates a [Pipe] block that pipes this output through the given [transform] function, connects
     * it, and returns the pipe's output.
     *
     * Useful for quick transformations.
     */
    inline fun <T, R> Output<T>.pipe(
        crossinline transform: T.() -> R
    ): Output<R> = Pipe.of(transform).also { this into it }

    /**
     * Adds the given [combineBlock], connects [this] and [second] to its first and second inputs, and returns the
     * combine's output.
     *
     * Useful for quick transformations.
     */
    fun <A, B, R> Output<A>.pipe(second: Output<B>, combineBlock: Combine<A, B, R>): Output<R> =
        combineBlock.also { this into it.first; second into it.second }

    /**
     * Creates a [Combine] block that combines [this] and [other] outputs through the given [combine] function with the
     * value of [this] as receiver, and returns the combination's output.
     */
    inline fun <A, B, R> Output<A>.combine(
        other: Output<B>,
        crossinline combine: A.(B) -> R
    ): Output<R> = Combine.of(combine).also { this into it.first; other into it.second }

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
}

/**
 * A base implementation of [BlocksConfig] that creates a set of [BlockConnections], which can
 * then be further processed.
 */
abstract class BaseBlocksConfig : BlocksConfig() {
    /** A hash map of existing [Block]s to [BlockConnections], representing its connections. */
    private val connectionsMap: IdentityHashMap<Block, BlockConnections> = IdentityHashMap()

    /** A collection of all the [BlockConnections] configured in this block so far. */
    protected val connections: Collection<BlockConnections> = connectionsMap.values

    /**
     * Holds all the connections of a [Block]; used inside the context of a [BaseBlocksConfig].
     *
     * @param block the block
     */
    class BlockConnections(val block: Block) {
        /** The sources of this block's inputs, or `null` if not (yet) connected. */
        val sources: Array<Output<*>?> = arrayOfNulls<Output<*>?>(block.numInputs)
        /** An array of booleans that indicates if the outputs have been connected yet. */
        val isOutConnected: BooleanArray = BooleanArray(block.numOutputs)
    }

    /**
     * Lets every block [verifyConfig][Block.prepareAndVerify] itself.
     */
    protected fun verifyConfig() {
        connections.forEach {
            it.block.prepareAndVerify(this)
        }
    }

    override fun Block.isPresent(): Boolean = connectionsMap.containsKey(this)

    private val Block.connection: BlockConnections get() = connectionsMap.getOrPut(this) { BlockConnections(this) }
    private val BlockIO.connection: BlockConnections get() = block.connection

    override fun <T> Output<T>.into(input: Input<T>) {
        let(this.connection, input.connection) { from, into ->
            check(into.sources[input.index] == null)
            { "Input at block (${into.block}) index (${input.index}) already connected!" }
            into.sources[input.index] = this
            from.isOutConnected[index] = true
        }
    }

    override fun Block.inputIsConnected(index: Int): Boolean = sourceOfInput(index) != null

    override fun Block.outputIsConnected(index: Int): Boolean = connectionsMap[this]?.isOutConnected?.get(index) == true

    override fun Block.sourceOfInput(index: Int): Output<*>? = connectionsMap[this]?.sources?.get(index)

    @Suppress("UNCHECKED_CAST")
    override fun <T> Input<T>.source(): Output<T>? = block.sourceOfInput(index) as? Output<T>?
}
