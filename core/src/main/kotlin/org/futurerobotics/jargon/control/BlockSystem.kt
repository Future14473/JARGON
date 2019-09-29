package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.InOutOrder.OUT_FIRST
import org.futurerobotics.jargon.system.LoopSystem
import org.futurerobotics.jargon.util.asMutableList
import java.util.*

/**
 * A control system made up of several connected [Block]s.
 *
 * Use [BlocksBuilder] to create. I worked hard on the DSLs.
 */
/*
 * Blocks should be alwaysRun first; with the amount given in [numAlwaysRun]
 */
class BlockSystem internal constructor(
    connectedBlocks: List<ConnectedBlock>
) : LoopSystem {
    private val blocks: Array<WorkingBlock>

    init {
        this.blocks = connectedBlocks.map { it.toWorkingBlock() }.toTypedArray()
    }

    private val alwaysRun = this.blocks.takeWhile { it.block.processing == Block.Processing.ALWAYS }
    private val updateQueue = ArrayDeque<WorkingBlock>()
    private var loopNumber = -1

    //special blocks
    private val shutdown = getSpecial<Shutdown>()
    private val loopNumberBlock = getSpecial<LoopNumber>()
    private val loopTimeBlock = getSpecial<LoopTime>()

    override fun init() {
        loopNumber = -1
        blocks.forEach {
            it.init()
        }
    }

    override fun loop(loopTime: Double): Boolean {
        loopNumber += 1

        loopNumberBlock?.loopNumber = loopNumber
        loopTimeBlock?.loopTime = loopTime

        updateQueue.addAll(alwaysRun)
        while (updateQueue.isNotEmpty()) {
            updateQueue.remove().ensureProcessed()
        }

        return shutdown?.shouldShutdown ?: false
    }

    override fun stop() {
        blocks.forEach { it.stop() }
    }

    private fun ConnectedBlock.toWorkingBlock(): WorkingBlock = when (block.inOutOrder) {
        OUT_FIRST -> OutFirstBlock(this)
        IN_FIRST -> InFirstBlock(this)
    }

    private inline fun <reified T : SpecialBlock> getSpecial(): T? {
        return blocks.map { it.block }.filterIsInstance<T>().also { assert(it.size <= 1) }.firstOrNull()
    }


    private abstract inner class WorkingBlock(inner: ConnectedBlock) {
        @JvmField
        val block = inner.block
        @JvmField
        protected var lastProcess = -1
        @JvmField
        protected val isAlwaysProcess = block.processing == Block.Processing.ALWAYS
        @JvmField
        protected val inputs: List<Any?> = object : AbstractList<Any?>() {
            override val size: Int
                get() = block.numInputs

            override fun get(index: Int): Any? {
                val blockIndex = sourceBlockIndexes[index]
                return blocks[blockIndex].getOutputLazy(sourceOutputIndexes[index])
            }
        }
        @JvmField
        protected var outputs = arrayOfNulls<Any>(block.numOutputs).asMutableList()

        private val sourceBlockIndexes = inner.sourceBlockIndexes

        private val sourceOutputIndexes = inner.sourceOutputIndexes

        /** gets output only, not yet process. */
        fun getOutputLazy(index: Int): Any? {
            ensureHasOutput()
            return outputs[index]
        }

        abstract fun init()
        abstract fun stop()

        /** Ensure that this block is processed. Called externally from queue only.*/
        abstract fun ensureProcessed()

        /** Ensure that this block has outputs. Called on request from block.*/
        protected abstract fun ensureHasOutput()

    }

    private inner class InFirstBlock(inner: ConnectedBlock) : WorkingBlock(inner) {

        /** assertion purposes only */
        private var processing = false

        override fun init() {
            lastProcess = -1
            processing = false
            block.init(outputs)
            outputs.fill(null) //ignore.
        }

        override fun stop() {
            lastProcess = -1
            processing = false
            outputs.fill(null)
        }

        override fun ensureProcessed() {
            if (lastProcess != loopNumber) { //!PROCESSED
                assert(!processing) { "loop in block system" } //PROCESSING
                processing = true //= PROCESSING
                block.process(inputs, outputs)
                processing = false
                lastProcess = loopNumber //= PROCESSED
            }
            //PROCESSED
        }

        override fun ensureHasOutput() {
            ensureProcessed()
        }
    }

    private inner class OutFirstBlock(inner: ConnectedBlock) : WorkingBlock(inner) {
        private var storedOutputs = arrayOfNulls<Any>(block.numOutputs).asMutableList()
        private var lastStore = -1

        private fun swapOutputs() {
            val t = storedOutputs
            storedOutputs = outputs
            outputs = t
        }

        override fun init() {
            lastProcess = -1
            lastStore = -1
            outputs.fill(null)
            storedOutputs.fill(null)
            block.init(storedOutputs)
        }

        override fun stop() {
            lastProcess = -1
            lastStore = -1
            outputs.fill(null)
            storedOutputs.fill(null)
        }

        override fun ensureProcessed() {
            if (lastStore != loopNumber) {//NOT_PROCESSED
                swapOutputs()
                lastStore = loopNumber
                //STORED -> STORED_PROCESS_NOW
            }
            //STORE_PROCESS_NOW
            //STORE_PROCESSING since lastStore = cycleNumber
            storedOutputs.fill(null)
            block.process(inputs, storedOutputs)
            //PROCESSED
        }

        override fun ensureHasOutput() {
            if (lastStore != loopNumber) { //NOT_PROCESSED
                swapOutputs() //reveal stored outputs; current stored outputs is now new
                lastStore = loopNumber //= STORED
                if (!isAlwaysProcess)
                    updateQueue += this //and update later.
            }
            //PROCESSED, STORED, STORED_PROCESSING
        }
    }

}

internal class ConnectedBlock(
    val block: Block,
    val sourceBlockIndexes: IntArray,
    val sourceOutputIndexes: IntArray
)
