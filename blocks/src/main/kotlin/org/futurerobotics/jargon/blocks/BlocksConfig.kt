package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.BaseBlocksConfig.BlockConnections
import org.futurerobotics.jargon.blocks.BlocksConfig.Output.Companion.ofInfer
import org.futurerobotics.jargon.util.let
import java.util.*


/**
 * Common interface for [Input] and [Output]; use those instead.
 */
interface BlockIO {
    /** The block that this input/output comes from */
    val block: Block
    /** The input/output index of this input. */
    val index: Int
}


/**
 * An exception thrown when an illegal configuration is detected.
 */
class IllegalBlockConfigurationException
@JvmOverloads constructor(message: String? = null) : RuntimeException(message)

/**
 * Coordinates the fitting together blocks together to build a system.
 *
 * For kotlin, make a context with `this` as this config to make use of extension functions, for example using
 * [kotlin.apply] or [kotlin.run], or in specifically making block systems, [buildBlocksSystem].
 *
 * [BlocksSystemBuilder] is a subclass of this.
 *
 * This is an mostly abstract class instead of an interface to make use of certain inline functions.
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
     * An input can only be connected once; throw [IllegalStateException] if otherwise.
     *
     * The types of [Output] and [Input] will be checked for compatibility.
     */
    abstract infix fun <T> Output<T>.into(input: Input<T>)

    /**
     * Connects the given [Input] _from_ the given [Output]
     *
     * An input can only be connected once; throw [IllegalStateException] if otherwise.
     *
     * The types of [Output] and [Input] will be checked for compatibility.
     */
    open infix fun <T> Input<T>.from(output: Output<T>): Unit = output into this

    /**
     * Connects the inputs of [this] block to all the given [outputs], in order.
     *
     * May throw [ClassCastException] if types are not compatible.
     */
    @Suppress("UNCHECKED_CAST")
    open fun Block.fromAll(vararg outputs: Output<*>) {
        require(outputs.size <= this.numInputs)
        { "the given number of outputs ${outputs.size} must not exceed the block's number of inputs $this.numInputs" }
        outputs.forEachIndexed { index, output ->
            output into Input.ofAny(this, index) as Input<Any?>
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

    /**
     * Returns if the given [Input] is connected.
     */
    open fun Input<*>.isConnected(): Boolean = source() != null

    /**
     * Returns if the given [Output] is connected.
     */
    abstract fun Output<*>.isConnected(): Boolean

    /**
     * Gets where this [this] block is connected from, or `null` if not added or connected.
     */
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
     * *** The input must first be connected in order to have a source ***
     *
     * Note that if the source has a processing of [Block.Processing.IN_FIRST_LAZY], it will now always be processed.
     */
    open fun <T> Input<T>.monitor(): Monitor<T> = source()?.monitor()
        ?: throw IllegalStateException("Input has not yet been connected, cannot deduce source")

    /**
     * Creates a block that pipes this output through the given [transform] (usually using a [Pipe] block), connects
     * it, and returns the pipe's output.
     */
    inline fun <T, R> Output<T>.pipe(crossinline transform: T.() -> R): Output<R> =
        pipe(this, transform)

    /**
     * Creates this that combines [this] and [other] outputs through the given [combine] (usually using a [Combine] block)
     * and returns the combination's output.
     */
    inline fun <A, B, R> Output<A>.combine(
        other: Output<B>,
        crossinline combine: A.(B) -> R
    ): Output<R> = combine(this, other, combine)


    /**
     * Common interface for [Input] and [Output]; use those instead.
     */
    interface BlockIO {
        /** The block that this input/output comes from */
        val block: Block
        /** The input/output index of this input. */
        val index: Int
    }

    /**
     * Represents a specific __output__ of a block; used in configuration, that takes in [type]-[T] (covariant).
     *
     * This must match the type in [Block.inputTypes].
     *
     * Usually [Block]s themselves should provide specific ways of getting [Input]/[Output] instead of manually
     * creating, but it is possible using [ofInfer], which checks type upon creation.
     */
    interface Output<out T> : BlockIO {
        /** The type of this [Output]. Covariant. */
        val type: Class<out T>

        companion object {
            /** Creates a [Output] by providing values. The [typeVerify] is to confirm generics with the [block] for validity. */
            fun <T> of(block: Block, index: Int, typeVerify: Class<T>): Output<T> {
                if (index !in 0 until block.numOutputs)
                    throw IndexOutOfBoundsException("Index $index not out block's number of outputs ${block.numOutputs}")
                //out = something super T
                val type = block.outputTypes[index].let {
                    require(typeVerify.isAssignableFrom(it)) { "Given type for verification $typeVerify must be compatible with block's given type $it" }
                    @Suppress("UNCHECKED_CAST") //checked above
                    it as Class<out T>
                }
                return BasicOutput(block, index, type)
            }

            /** Creates a [Output] using reified generics; kotlin only. */
            inline fun <reified T> ofInfer(block: Block, index: Int): Output<T> = of(block, index, T::class.java)

            /** Creates a [Output] using the type specified in [Block], but type is not known at compile time */
            fun ofAny(block: Block, index: Int): Output<*> = BasicOutput(block, index, block.outputTypes[index])
        }
    }


    /**
     * Represents a specific __input__ of a block; used in configuration.
     *
     * It takes in type [T]. The `typeVerify` parameter is there to enforce that your generics match
     * the [block]'s type.
     *
     * Usually [Block]s themselves should provide specific ways of getting [Input]/[Output] instead of manually creating
     * one here.
     *
     * It is not recommended to create instances of this outside of from a [Block], else types cannot be enforced.
     */
    interface Input<in T> : BlockIO {
        /** The type of this [Input]. Contravariant. */
        val type: Class<in T>

        companion object {
            /** Creates a [Output] by providing values. The [typeVerify] is to confirm generics with the [block] for validity. */
            fun <T> of(block: Block, index: Int, typeVerify: Class<T>): Input<T> {
                if (index !in 0 until block.numInputs)
                    throw IndexOutOfBoundsException("Index $index not in block's number of inputs ${block.numInputs}")
                //in = something super T
                val actualType = block.inputTypes[index].let {
                    require(it.isAssignableFrom(typeVerify))
                    { "Given type for verification $typeVerify must be compatible with block's given type $it" }
                    @Suppress("UNCHECKED_CAST") //checked above
                    it as Class<in T>
                }
                return BasicInput(block, index, actualType)
            }

            /** Creates a [Input] using reified generics; kotlin only. */
            inline fun <reified T> ofInfer(block: Block, index: Int): Input<T> = of(block, index, T::class.java)

            /** Creates a [Input] using the type specified in [Block], but type is not known at compile time */
            fun ofAny(block: Block, index: Int): Input<*> = BasicInput(block, index, block.inputTypes[index])
        }
    }
}

private class BasicOutput<out T>(
    override val block: Block,
    override val index: Int,
    override val type: Class<out T>
) : BlocksConfig.Output<T>

private class BasicInput<in T>(
    override val block: Block,
    override val index: Int,
    override val type: Class<in T>
) : BlocksConfig.Input<T>

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
     * Holds all the connections of a [Block]; use inside context of a [BaseBlocksConfig].
     *
     * @param block the block
     */
    class BlockConnections(val block: Block) {
        /** The sources of this block's inputs, or `null` if not (yet) connected. */
        val inputSources: Array<Output<*>?> = arrayOfNulls<Output<*>?>(block.numInputs)
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
        let(this.type, input.type) { sourceType, inputType ->
            require(inputType.isAssignableFrom(sourceType))
            { "Input type $inputType is not compatible with output type $sourceType" }
        }
        let(this.connection, input.connection) { from, into ->
            check(into.inputSources[input.index] == null)
            { "Input at block (${into.block}) index (${input.index}) already connected!" }
            into.inputSources[input.index] = this
            from.isOutConnected[index] = true
        }
    }

    override fun Output<*>.isConnected(): Boolean =
        connectionsMap[block]?.isOutConnected?.get(index) == true //not null and true

    @Suppress("UNCHECKED_CAST")
    override fun <T> Input<T>.source(): Output<T>? =
        connectionsMap[block]?.inputSources?.get(index) as? Output<T>?

}
