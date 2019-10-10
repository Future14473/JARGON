package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.BaseBlocksConfig.BlockConnections
import org.futurerobotics.jargon.control.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.util.fillWith
import java.util.*
import kotlin.collections.ArrayList
import kotlin.collections.HashMap

/*
 * Classes to represent the assembly of block systems.
 *
 * Originally intended so that can reuse parts for composite blocks, but that has problems with SpecialBlocks.
 */
/**
 * A highly processed block with connects directly indicated as as indexes in a list.
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

/** Rearranges by sorting [IndexedBlock]s using the given [comparator] while keeping indexes in tact. */
internal fun MutableList<IndexedBlock>.rearranged(comparator: Comparator<Block>) {
    val sorted = zip(indices).sortedWith(kotlin.Comparator { (o1), (o2) ->
        comparator.compare(o1.block, o2.block)
    })
    val toNewIndexes =
        sorted.mapIndexed { newIndex, (_, oldIndex) -> oldIndex to newIndex }.associateTo(HashMap()) { it }
    toNewIndexes[-1] = -1
    sorted.forEach { (block) ->
        val sourceBlockIndexes = block.sourceBlockIndices
        repeat(sourceBlockIndexes.size) {
            sourceBlockIndexes[it] = toNewIndexes[sourceBlockIndexes[it]]!!
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
internal fun Collection<BlockConnections>.toIndexedBlocks(): List<IndexedBlock> {
    return IndexedBlocksCreator(this).result
}

/** Creates [IndexedBlock]s */
private class IndexedBlocksCreator(
    connections: Collection<BlockConnections>
) {
    val result: List<IndexedBlock>

    private var nodeBlocks: Map<Block, NodeBlock> =
        connections.associateTo(IdentityHashMap()) { it.block to NodeBlock(it) }

    private val BlockIO.nodeBlock
        get() = nodeBlocks[block] ?: throw IllegalArgumentException("Reference to not included block ($block)")

    private val needTrace: Queue<NodeBlock> = LinkedList<NodeBlock>()

    init {
        traceAll()
        result = createConnectedBlocks()
    }

    private fun traceAll() {
        val roots = nodeBlocks.values.filter { it.block.processing.isAlwaysProcess }
        needTrace += roots
        while (needTrace.isNotEmpty()) {
            needTrace.remove().run {
                if (traceStatus == STORED)
                    traceStatus = STORED_PROCESS_NOW
                trace()
            }
        }
    }

    /** Traces (dfs-type thing) all the inputs at the [this]; marking used blocks and making sure there are no
     * unresolvable loops.*/
    private fun NodeBlock.trace(): Unit = run {
        when (traceStatus) {
            PROCESSED, STORED, STORED_PROCESSING -> return
            PROCESSING -> { //can't get inputs if needs inputs
                throw IllegalBlockConfigurationException(
                    "Loop at component ${this}. Consider making a block in " +
                            "the loop out first (or adding one to the loop), or breaking " +
                            "up the loop."
                )
            }
            STORED_PROCESS_NOW -> {
                traceStatus = STORED_PROCESSING
                inputSources.forEach { it?.nodeBlock?.trace() }
                traceStatus = PROCESSED
            }
            NOT_PROCESSED -> {
                if (block.processing === OUT_FIRST_ALWAYS) {
                    //if out first, store outputs first.
                    traceStatus = STORED
                    needTrace.add(this) //and update later.
                } else {
                    traceStatus = PROCESSING
                    inputSources.forEach { it?.nodeBlock?.trace() }
                    traceStatus = PROCESSED
                }
            }
            else -> throw AssertionError()
        }
    }

    private fun createConnectedBlocks(): List<IndexedBlock> {
        val usedNodes =
            nodeBlocks.values.filterTo(ArrayList(nodeBlocks.size)) { it.traceStatus == PROCESSED }
        usedNodes.forEachIndexed { index, node ->
            node.finalIndex = index
        }
        return usedNodes.map { node ->
            val sources = node.inputSources

            val sourceBlockIndexes = sources.mapToIntArray { out ->
                out?.nodeBlock?.finalIndex?.also {
                    assert(it != -1) { "all sources should have index" }
                } ?: -1
            }
            val sourceOutputIndexes = sources.mapToIntArray { it?.outputIndex ?: -1 }

            IndexedBlock(
                block = node.block,
                sourceBlockIndices = sourceBlockIndexes,
                sourceOutputIndices = sourceOutputIndexes
            )
        }
    }

    private inline fun <T> Array<T>.mapToIntArray(mapping: (T) -> Int): IntArray {
        return IntArray(size).also {
            this.forEachIndexed { index, t ->
                it[index] = mapping(t)
            }
        }
    }

    private inner class NodeBlock(val connections: BlockConnections) {
        inline val block get() = connections.block
        inline val inputSources get() = connections.inputSources
        var traceStatus = NOT_PROCESSED
        var finalIndex = -1
    }


    companion object {
        private const val NOT_PROCESSED = 0
        private const val PROCESSING = 1
        private const val STORED = 2
        private const val STORED_PROCESS_NOW = 3
        private const val STORED_PROCESSING = 4
        private const val PROCESSED = 5
    }
}


/**
 * Most people do not have to deal with this.
 *
 * Common components of [BlocksSystem] and [CompositeBlock]. Use those instead unless you know what you're doing.
 */
abstract class AbstractBlocksRunner(
    connections: Collection<BlockConnections>
) {
    private val allRunners: Array<BlockRunner>
    private val alwaysRun: Array<BlockRunner>

    init {
        val indexedBlocks = connections.toIndexedBlocks()
        allRunners = indexedBlocks.map {
            if (it.block.processing == OUT_FIRST_ALWAYS)
                OutFirstBlock(it)
            else
                InFirstBlock(it)
        }.toTypedArray()

        alwaysRun = allRunners.filter { it.block.processing.isAlwaysProcess }.toTypedArray()
    }

    private val updateQueue: Queue<BlockRunner> = ArrayDeque<BlockRunner>()

    /** The loop number; used to track if blocks are updated. Incremented on [processOnce]. Change with caution*/
    protected var loopNumber: Int = -1
        private set

    /** `init`s all the blocks. */
    protected open fun init() {
        loopNumber = -1
        allRunners.forEach { it.init() }
    }

    /** Process through all the blocks once, also increments [loopNumber] */
    protected fun processOnce() {
        loopNumber++
        updateQueue.addAll(alwaysRun)
        while (updateQueue.isNotEmpty()) {
            updateQueue.remove().ensureProcessed()
        }
    }

    /** stops all the blocks and does some cleanup. */
    protected open fun stop() {
        loopNumber = -1
        allRunners.forEach { it.stop() }
    }

    /**
     * A wrapper a [block] that actually processes it.
     */
    protected abstract inner class BlockRunner internal constructor(inner: IndexedBlock) {
        private val sourceBlockIndices = inner.sourceBlockIndices
        private val sourceOutputIndices = inner.sourceOutputIndices

        /** The block used */
        @JvmField
        val block: Block = inner.block
        /** The last lostNumber this has been processed */
        @JvmField
        protected var lastProcess: Int = -1
        /** The inputs to this block; calling [get] may poll other blocks. */
        @JvmField
        protected val inputs: List<Any?> = object : AbstractList<Any?>() {
            override val size: Int
                get() = block.numInputs

            override fun get(index: Int): Any? {
                return sourceBlockIndices[index].let {
                    if (it == -1) null
                    else allRunners[it].getOutputLazy(sourceOutputIndices[index])
                }
            }
        }
        /**
         * Where the outputs to this block are stored.
         */
        @JvmField
        protected var outputs: Array<Any?> = arrayOfNulls(block.numOutputs)

        /**
         * Initializes the block.
         */
        open fun init() {
            lastProcess = -1
            outputs.fill(null)
            block.init()
        }

        /** Stops and cleans up on this block. */
        fun stop() {
            lastProcess = -1
            outputs.fill(null)
        }

        /** Gets output at this index; may or may not process. */
        fun getOutputLazy(index: Int): Any? {
            ensureOutput(index)
            return outputs[index].also { assert(it !== NO_OUTPUT) }
        }

        /**
         * Ensure that this block is only processed. Called externally from queue only, and [getOutputLazy] may have
         * never been called.
         * */
        abstract fun ensureProcessed()

        /**
         * Ensure that this block has the output at the given index. Called on request from another block.
         */
        protected abstract fun ensureOutput(index: Int)

    }

    private inner class InFirstBlock(inner: IndexedBlock) : BlockRunner(inner) {

        /** assertion purposes only */
        private var processing = false

        override fun init() {
            super.init()
            processing = false
        }

        override fun ensureProcessed() {
            if (lastProcess == loopNumber) return
            //not PROCESSED
            assert(!processing) { "loop in block system" }

            processing = true //= PROCESSING
            block.process(inputs)
            processing = false

            lastProcess = loopNumber //= PROCESSED
            outputs.fill(NO_OUTPUT)
            //PROCESSED
        }

        override fun ensureOutput(index: Int) {
            ensureProcessed()
            if (outputs[index] === NO_OUTPUT) outputs[index] = block.getOutput(index, inputs)
        }
    }

    private inner class OutFirstBlock(inner: IndexedBlock) : BlockRunner(inner) {

        override fun ensureProcessed() {
            ensureOutput(0)
            block.process(inputs)
        }

        override fun ensureOutput(index: Int) {
            if (lastProcess == loopNumber) return
            //NOT_PROCESSED
            lastProcess = loopNumber
            outputs.fillWith {
                block.getOutput(it, emptyList()) //either: STORED
            }
            //PROCESSED, STORED, STORED_PROCESSING
        }
    }

    private companion object {
        private val NO_OUTPUT = Any()
    }
}
