package org.futurerobotics.jargon.blocks

/**
 * Represents a group of connected blocks.
 *
 * This can be made with [BlockArrangementBuilder]
 */
class BlockArrangement internal constructor(internal val blocks: Set<Block>)

/**
 * Builder for a [BlockArrangement].
 *
 * At least one block out of a network of connected blocks must be [add]ed to this arrangement ([config] also adds).
 * **Make sure blocks that use [Block.ExtendedContext] ([generate], [QuickBlock]) are also added to this arrangement
 * or [Block.link]ed with blocks that are in the arrangement).
 *
 * In kotlin, use [with] or [apply] on this to scope and make use of member extension functions.
 *
 * Also, this supplies convenience functions [config] and [runBlock] for common quick configuration patterns.
 */
class BlockArrangementBuilder {

    private val blocks = HashSet<Block>()
    /** Adds blocks, and all blocks that are or will be connected to or linked to these block */
    fun add(vararg blocks: Block) {
        this.blocks += blocks
    }

    /** Adds blocks via inputs/outputs, all blocks that are or will be connected to or linked to these block */
    fun add(vararg ios: Block.AnyIO<*>) {
        ios.forEach {
            this.blocks += it.block
        }
    }

    /**
     * Adds a block, and all blocks that are connected to (or will be connected to) this block, then returns this.
     */
    fun <T : Block> T.add(): T = apply { add(this) }

    /**
     * Adds a block, and all blocks that are connected to (or will be connected to) this block, then returns this.
     */
    fun <T : Block.AnyIO<*>> T.add(): T = apply { add(this) }

    private var built = false

    /** Runs the [configuration], then [build]s the block arrangement. */
    @JvmSynthetic
    inline fun build(configuration: BlockArrangementBuilder.() -> Unit): BlockArrangement {
        apply(configuration)
        return build()
    }

    /**
     * Builds the block arrangement.
     *
     * At least one block out of a network of connected blocks must be added to this
     * [BlockArrangement].
     */
    fun build(): BlockArrangement {
        check(!built) { "Already built!" }
        built = true
        check(blocks.isNotEmpty()) {
            "No blocks in block arrangement! Make sure you 'add'ed at least one in a connected group of them."
        }
        val blocksCopy = blocks.toList()
        blocks.clear()
        blocksCopy.forEach {
            trace(it)
        }
        blocks.forEach { it.finalizeConfig() }
        return BlockArrangement(blocks)
    }

    private fun trace(block: Block) {
        if (!blocks.add(block)) return
        block.inputs.forEach {
            it.source?.let { output ->
                trace(output.block)
            }
        }
        block.outputs.forEach {
            it.connectedTo.forEach { input ->
                trace(input.block)
            }
        }
        block.linkedWith.forEach {
            trace(it)
        }
    }

    /**
     * Creates a block that that simply runs the given [action], [add]s it, and returns it.
     *
     * **Keep in mind that all referenced blocks need to be [Block.link]ed with this output or added to the same
     * [BlockArrangement].**
     *
     * [Block.ExtendedContext] is supported, so that one can get outputs of other blocks without directly connecting.
     *
     * @see generate
     */
    fun runBlock(action: Block.ExtendedContext.() -> Unit): Block = object : QuickBlock(Processing.ALWAYS) {
        override fun ExtendedContext.process() {
            action()
        }
    }.also { add(it) }

    /**
     * Runs the given [configuration] on [this], [add]s [this], then returns [this].
     *
     * Useful for easy and fluent configurations.
     */
    fun <T : Block> T.config(configuration: T.() -> Unit): T {
        configuration()
        add(this)
        return this
    }
}

