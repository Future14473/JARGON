package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.util.uncheckedCast

/** Common components of [BlockArrangement] and [BlockArrangementBuilder]; provides read only access to connections. */
interface ReadOnlyBlockArrangement {

    /** A map of [Block]s to their [BlockConnections]; if they exist */
    val connections: Map<Block, BlockConnections>

    /** If the given [block] exists in this config */
    operator fun contains(block: Block): Boolean = connections.containsKey(block)

    /** If the given [input] exists and is connected in this config */
    operator fun contains(input: Block.Input<*>): Boolean = sourceOf(input) != null

    /** Gets the output source from which the given [input] is connected, or `null` if it does not exist */
    fun <T> sourceOf(input: Block.Input<T>): Block.Output<out T>? = connections[input.block]?.sources?.get(input.index)
        .uncheckedCast()
}
/** Deprecated */
@Deprecated("Use ReadOnlyBlockArrangementInstead")
typealias ReadOnlyBlockConfig = ReadOnlyBlockArrangement

/**
 * Represents the connections in and out of a particular block within the context of a [BlockArrangement].
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


/** Deprecated */
@Deprecated("Use BlockArrangement instead")
typealias BlockConfig = BlockArrangement

/**
 * A configuration of connected blocks. Created using [BlockArrangementBuilder]
 */
class BlockArrangement internal constructor(override val connections: Map<Block, BlockConnections>) :
    ReadOnlyBlockArrangement {

    companion object {
        /** Creates a new [BlockArrangementBuilder]. */
        @JvmStatic
        fun builder(): BlockArrangementBuilder =
            BlockArrangementBuilder()
    }
}
