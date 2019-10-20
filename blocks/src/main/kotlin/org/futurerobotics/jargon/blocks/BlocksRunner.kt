package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.util.fillWith
import org.futurerobotics.jargon.util.fixedSizeMutableListOfNulls
import org.futurerobotics.jargon.util.replaceIf
import java.util.*
import kotlin.collections.ArrayList
import kotlin.collections.HashMap

/**
 * A highly processed block with connections directly indicated as indices, in a list.
 * **This class only makes sense in the context of a list.**
 *
 * @param sourceBlockIndices a array of the same size as the [block]'s inputs, giving the _index within a list_ of another
 *      [ListConnectedBlock] that is the source for that input. If the input is not connected, -1.
 * @param sourceOutputIndices a array of the same size as the [block]'s inputs, giving the _index of the output_ of another
 *      [ListConnectedBlock] that is the source for that input
 */
internal class IndexedBlock(
    val block: Block,
    val sourceBlockIndices: IntArray,
    val sourceOutputIndices: IntArray
)

/** Rearranges by sorting [IndexedBlock]s using the given [comparator] while keeping indices in tact. */
internal fun MutableList<IndexedBlock>.rearranged(comparator: Comparator<Block>) {
    val sorted = zip(indices).sortedWith(kotlin.Comparator { (o1), (o2) ->
        comparator.compare(o1.block, o2.block)
    })
    val toNewIndices =
        sorted.mapIndexed { newIndex, (_, oldIndex) -> oldIndex to newIndex }.associateTo(HashMap()) { it }
    toNewIndices[-1] = -1
    sorted.forEach { (block) ->
        val indices = block.sourceBlockIndices
        repeat(indices.size) {
            indices[it] = toNewIndices[indices[it]]!!
        }
    }
    clear()
    addAll(sorted.map { it.first })
}

/**
 * This takes a collection of [BlockConnections] and processes and returns an equivalent list of [IndexedBlock]s,
 * also filtering out unused blocks if they are not connected to a block that is [Block.Processing.isAlwaysProcess]
 *
 * This also verifies that there are no impossible loops within the connections.
 */
internal fun Collection<BlocksConfig.Connections>.toIndexedBlocks(): List<IndexedBlock> =
    IndexedBlocksCreator(this).result

private class IndexedBlocksCreator(
    connections: Collection<BlocksConfig.Connections>
) {
    val result: List<IndexedBlock>

    private var nodeBlocks: Map<Block, NodeBlock> =
        connections.associateTo(IdentityHashMap()) { it.block to NodeBlock(it) }

    private val BlocksConfig.BlockIO.nodeBlock
        get() = nodeBlocks[block] ?: throw IllegalArgumentException("Reference to not included block ($block)")

    init {
        traceAll()
        result = createConnectedBlocks()
    }

    /** Verifies no bad loops */
    private fun traceAll() {
        nodeBlocks.values
            .forEach {
                if (it.block.processing.isAlwaysProcess) {
                    if (it.block.processing.isOutFirst)
                        it.traceStatus = PROCESS_NOW
                    it.trace()
                }
            }
    }

    /** Traces (dfs-type thing) all the inputs at the [this]; marking used blocks and making sure there are no
     * unresolvable loops.*/
    private fun NodeBlock.trace() {
        when (traceStatus) {
            PROCESSED -> return
            NOT_PROCESSED -> if (!block.processing.isOutFirst) doTrace()
            PROCESS_NOW -> doTrace()
            PROCESSING -> { //can't get inputs if needs inputs
                if (!block.processing.isOutFirst)
                    throw IllegalBlockConfigurationException(
                        """
                            Loop at component $block. 
                            Consider:
                            - Breaking up the loop
                            - Changing to or adding a block with OUT_FIRST_ALWAYS
                            - Inserting a [Delay] block using BlocksConfig.delay
                            - Checking configuration
                            """.trimIndent()
                    )

            }
            else -> throw AssertionError()
        }
    }


    private fun NodeBlock.doTrace() {
        traceStatus = PROCESSING
        inputSources.forEach { it?.nodeBlock?.trace() }
        traceStatus = PROCESSED
    }

    private fun createConnectedBlocks(): List<IndexedBlock> {
        val usedNodes =
            nodeBlocks.values.filterTo(ArrayList(nodeBlocks.size)) { it.traceStatus == PROCESSED }
        usedNodes.forEachIndexed { index, node ->
            node.finalIndex = index
        }
        return usedNodes.map { node ->
            val sources = node.inputSources

            val sourceBlockIndices = sources.mapToIntArray { out ->
                out?.run {
                    nodeBlock.finalIndex.also {
                        assert(it != -1) { "all sources should have index" }
                    }
                } ?: -1
            }
            val sourceOutputIndices = sources.mapToIntArray { it?.index ?: -1 }

            IndexedBlock(
                block = node.block,
                sourceBlockIndices = sourceBlockIndices,
                sourceOutputIndices = sourceOutputIndices
            )
        }
    }

    private inline fun <T> List<T>.mapToIntArray(mapping: (T) -> Int): IntArray {
        return IntArray(size).also {
            this.forEachIndexed { index, t ->
                it[index] = mapping(t)
            }
        }
    }

    private inner class NodeBlock(val connections: BlocksConfig.Connections) {
        inline val block get() = connections.block
        inline val inputSources get() = connections.sources
        var traceStatus = NOT_PROCESSED
        var finalIndex = -1
    }


    companion object {
        private const val NOT_PROCESSED = 0
        private const val PROCESSING = 1
        private const val PROCESSED = 2
        private const val PROCESS_NOW = 3
    }
}


/**
 * Most people do not have to deal with this.
 *
 * Common components of [BlocksSystem] and [CompositeBlock]. Use those instead unless you know what you're doing.
 */
abstract class AbstractBlocksRunner(config: BlocksConfig) {
    private val allRunners: Array<BlockRunner>
    private val alwaysRun: Array<InFirstBlock>
    private val outFirst: Array<OutFirstBlock>

    init {
        config.verifyConfig()
        val indexedBlocks = config.connections.values.toIndexedBlocks()
        allRunners = indexedBlocks.map {
            if (it.block.processing.isOutFirst) OutFirstBlock(it)
            else InFirstBlock(it)
        }.toTypedArray()
        @Suppress("UNCHECKED_CAST")
        alwaysRun = allRunners
            .filter { it.block.processing === IN_FIRST_ALWAYS }
            .let { it as List<InFirstBlock> }
            .toTypedArray()
        outFirst = allRunners.filterIsInstance<OutFirstBlock>().toTypedArray()
    }

    /** The loop number; used to track if blocks are updated. Incremented on [processOnce] */
    protected var loopNumber: Int = -1
        private set

    /** The [SystemValues] to be given to the blocks */
    protected abstract val systemValues: SystemValues

    /** `init`s all the blocks. */
    protected open fun init() {
        loopNumber = -1
        allRunners.forEach { it.init() }
    }

    /** Process through all the blocks once, also increments [loopNumber] */
    protected fun processOnce() {
        loopNumber++
        alwaysRun.forEach { it.ensureHasOutput() }
        outFirst.forEach { it.fillInputs() }
        outFirst.forEach { it.process() }
    }

    /** stops all the blocks and does some cleanup. */
    protected open fun stop() {
        loopNumber = -1
        allRunners.forEach { it.stop() }
    }

    /** A wrapper a [block] that actually processes it. */
    private abstract inner class BlockRunner internal constructor(inner: IndexedBlock) {
        protected val sourceBlockIndices = inner.sourceBlockIndices
        protected val sourceOutputIndices = inner.sourceOutputIndices

        @JvmField
        val block: Block = inner.block

        /** Where the outputs to this block are stored. NO_OUTPUT if not polled this loop. */
        @JvmField
        protected var outputs: Array<Any?> = Array(block.numOutputs) { NO_OUTPUT }

        open fun init() {
            outputs.fill(NO_OUTPUT)
            block.init()
        }

        open fun stop() {
            outputs.fill(null)
        }

        /** Gets inputs into this block, lazy. */
        protected fun getInputLazy(index: Int): Any? = sourceBlockIndices[index].let {
            if (it == -1) null
            else allRunners[it].getOutputLazy(sourceOutputIndices[index])
        }

        /** Gets output at this index; may or may not process. */
        fun getOutputLazy(index: Int): Any? {
            ensureHasOutput()
            return outputs[index].replaceIf({ it === NO_OUTPUT }) {
                block.getOutput(index).also { outputs[index] = it }
            }
        }

        /** Ensure that this block has been processed. */
        abstract fun ensureHasOutput()
    }

    private inner class InFirstBlock(inner: IndexedBlock) : BlockRunner(inner) {
        /** The last loop number this has been processed */
        private var lastProcess: Int = -1
        /** assertion purposes only */
        private var processing = false
        private val inputs: List<Any?> = object : AbstractList<Any?>() {
            override val size: Int
                get() = block.numInputs

            override fun get(index: Int): Any? = getInputLazy(index)
        }

        override fun init() {
            super.init()
            lastProcess = -1
            processing = false
        }

        override fun ensureHasOutput() {
            if (lastProcess == loopNumber) return

            assert(!processing) { "loop in block system" }

            processing = true
            block.process(inputs, systemValues)
            outputs.fill(NO_OUTPUT)
            processing = false

            lastProcess = loopNumber
        }
    }

    private inner class OutFirstBlock(inner: IndexedBlock) : BlockRunner(inner) {
        internal val inputs = fixedSizeMutableListOfNulls<Any>(block.numInputs)
        fun fillInputs() {
            inputs.fillWith { getInputLazy(it) }
        }

        fun process() {
            block.process(inputs, systemValues)
            outputs.fill(NO_OUTPUT)
        }

        override fun ensureHasOutput() {
            //do nothing
        }
    }

    companion object {
        private val NO_OUTPUT = Any()
    }
}
