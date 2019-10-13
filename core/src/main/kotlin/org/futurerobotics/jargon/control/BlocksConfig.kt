@file:JvmName("BlockIOKt")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.BaseBlocksConfig.BlockConnections
import java.util.*

/**
 * Coordinates the fitting together blocks together to build a system.
 *
 * For kotlin, make a context with `this` as this config to make use of extension functions, for example using
 * [kotlin.apply] or [kotlin.run], or in specifically making block systems, [buildBlocksSystem].
 *
 * [BlocksSystemBuilder] is a subclass of this.
 *
 * @see [BaseBlocksConfig]
 * @see [BlocksSystem]
 */
interface BlocksConfig {

    /**
     * Ensures this block is registered in this builder. Kinda useless.
     */
    fun Block.ensureAdded()

    /**
     * Returns if this block is added.
     */
    fun Block.isAdded(): Boolean

    /**
     * Connects the given [BlockOutput] to the given [BlockInput].
     *
     * An input can only be connected once; throw IllegalStateException if otherwise.
     *
     * Blocks should be added if not already.
     */
    infix fun <T> BlockOutput<T>.connectTo(input: BlockInput<T>)

    /**
     * Connects the given [BlockInput] to the given [BlockOutput]
     *
     * Blocks should be added if not already.
     */
    infix fun <T> BlockInput<T>.connectFrom(output: BlockOutput<T>)

    //TODO: think about the type checking of this...
    /**
     * Connects the inputs of [this] block to all the given [outputs], in order.
     */
    fun Block.connectFromAll(vararg outputs: BlockOutput<Any?>)

    /**
     * Connects [this] [BlockOutput] to all of the given [inputs].
     */
    fun <T> BlockOutput<T>.connectToAll(vararg inputs: BlockInput<T>)

    /**
     * Returns if the given [BlockInput] is connected.
     */
    fun BlockInput<*>.isConnected(): Boolean

    /**
     * Returns if the given [BlockOutput] is connected.
     */
    fun BlockOutput<*>.isConnected(): Boolean

    /**
     * Gets where this [this] block is connected from, or `null` if not added or connected.
     */
    fun <T> BlockInput<T>.source(): BlockOutput<T>?

    /**
     * Creates and connects a [Monitor] from [this] [BlockOutput], and returns it.
     *
     * Note that if the block has a processing of [Block.Processing.IN_FIRST_LAZY], it will now always be processed.
     */
    fun <T> BlockOutput<T>.monitor(): Monitor<T>

    /**
     * Creates and connects a [Monitor] from the [source] of [this] [BlockInput], and returns it.
     *
     * *** The input must first be connected in order to have a source ***
     *
     * Note that if the source has a processing of [Block.Processing.IN_FIRST_LAZY], it will now always be processed.
     */
    fun <T> BlockInput<T>.monitor(): Monitor<T>
}

/**
 * Common interface for [BlockInput] and [BlockOutput]
 */
interface BlockIO {
    /** The block that this input/output comes from */
    val block: Block
}

/**
 * Represents the input of a block, that takes in the type [T].
 *
 * Generics are for assistance in verifying types.
 *
 * It is not recommended to create instances of this outside of from a [Block].
 */
interface BlockInput<in T> : BlockIO {
    /** The input index of this input. */
    val inputIndex: Int
}

/** A basic implementation of [BlockInput]. */
class BasicBlockInput<in T>(override val block: Block, override val inputIndex: Int) : BlockInput<T>

/**
 * Represents the output of a block, that outputs the type [T].
 *
 * Generics are for assistance in verifying types.
 *
 * It is not recommended to create instances of this outside of from a [Block].
 */
interface BlockOutput<out T> : BlockIO {
    /** The output index of this output. */
    val outputIndex: Int
}

/** A basic implementation of [BlockOutput]. */
class BasicBlockOutput<out T>(override val block: Block, override val outputIndex: Int) : BlockOutput<T>

/**
 * An exception thrown when an illegal configuration is detected.
 */
class IllegalBlockConfigurationException
@JvmOverloads constructor(message: String? = null) : RuntimeException(message)


/**
 * A base implementation of [BlocksConfig] that creates a set of [BlockConnections], which can
 * then be further processed.
 */
abstract class BaseBlocksConfig : BlocksConfig {
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
        val inputSources: Array<BlockOutput<*>?> = arrayOfNulls<BlockOutput<*>?>(block.numInputs)
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

    override fun Block.ensureAdded() {
        if (this !in connectionsMap) {
            connectionsMap[this] = BlockConnections(this)
        }
    }

    override fun Block.isAdded(): Boolean = connectionsMap.containsKey(this)

    override fun <T> BlockOutput<T>.connectTo(input: BlockInput<T>) {
        val inputNode = input.connection
        val outputBlock = connection
        check(inputNode.inputSources[input.inputIndex] == null)
        { "Input at block (${input.block}) index (${input.inputIndex}) already connected!" }
        inputNode.inputSources[input.inputIndex] = this
        outputBlock.isOutConnected[outputIndex] = true
    }

    private val Block.connection: BlockConnections
        get() {
            ensureAdded()
            return connectionsMap[this]!!
        }
    private val BlockIO.connection: BlockConnections get() = block.connection


    override fun <T> BlockInput<T>.connectFrom(output: BlockOutput<T>): Unit = output connectTo this

    override fun Block.connectFromAll(vararg outputs: BlockOutput<Any?>) {
        require(outputs.size <= this.numInputs)
        { "the given number of outputs ${outputs.size} must not exceed the block's number of inputs $this.numInputs" }
        outputs.forEachIndexed { index, output ->
            output connectTo BasicBlockInput(this, index)
        }
    }

    override fun <T> BlockOutput<T>.connectToAll(vararg inputs: BlockInput<T>) {
        inputs.forEach {
            this connectTo it
        }
    }

    override fun BlockInput<*>.isConnected(): Boolean = source() != null

    override fun BlockOutput<*>.isConnected(): Boolean =
        connectionsMap[block]?.isOutConnected?.get(outputIndex) == true //not null and true

    @Suppress("UNCHECKED_CAST")
    override fun <T> BlockInput<T>.source(): BlockOutput<T>? =
        connectionsMap[block]?.inputSources?.get(inputIndex) as? BlockOutput<T>?

    override fun <T> BlockOutput<T>.monitor(): Monitor<T> = Monitor<T>().apply {
        connectTo(this)
    }

    override fun <T> BlockInput<T>.monitor(): Monitor<T> =
        source()?.monitor() ?: throw IllegalStateException("Input has not yet been connected, cannot deduce source")


}
