package org.futurerobotics.jargon.blocks.config

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.util.uncheckedCast

/** Common components of [BlockConfig] and [BCBuilder]; provides read only access to connections. */
interface ReadOnlyBlockConfig {

    /** A map of [Block]s to their [BlockConnections]; if they exist */
    val connections: Map<Block, BlockConnections>

    /** If the given [block] exists in this config */
    operator fun contains(block: Block): Boolean

    /** If the given [input] exists and is connected in this config */
    operator fun contains(input: Block.Input<*>): Boolean

    /** Gets the output source from which the given [input] is connected, or `null` if it does not exist */
    fun <T> sourceOf(input: Block.Input<T>): Block.Output<out T>?
}

/**
 * Base implementation of [ReadOnlyBlockConfig]
 */
abstract class AbstractReadOnlyBlockConfig : ReadOnlyBlockConfig {

    override fun contains(block: Block): Boolean = connections.containsKey(block)

    override fun contains(input: Block.Input<*>): Boolean = sourceOf(input) != null

    override fun <T> sourceOf(input: Block.Input<T>): Block.Output<out T>? =
        connections[input.block]?.sources?.get(input.index).uncheckedCast()
}

/**
 * Represents the connections in and out of a particular block within the context of a [BlockConfig].
 */
interface BlockConnections {

    /** The block that this [BlockConnections] is representing. */
    val block: Block
    /**
     * The sources (other block's outputs) which are connected to the inputs of this block, or `null`
     * if not connected.
     *
     * Should be same size as [Block.numInputs],
     */
    val sources: List<Block.Output<*>?>
}

/**
 * A configuration of connected blocks. Created using [BCBuilder]
 */
class BlockConfig internal constructor(override val connections: Map<Block, BlockConnections>) :
    AbstractReadOnlyBlockConfig()
