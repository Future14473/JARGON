package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.config.BlockConfig
import org.futurerobotics.jargon.util.fillWith
import org.futurerobotics.jargon.util.fixedSizeMutableListOfNulls
import org.futurerobotics.jargon.util.uncheckedCast

/**
 * Most people do not have to deal with this.
 *
 * Common components of [BlockSystem] and [CompositeBlock]. Use those instead unless you know what you're doing.
 */
abstract class BlockRunner(config: BlockConfig) {

    private val allRunners: Map<Block, Runner>
    private val alwaysRun: Array<InFirstRunner>
    private val outFirst: Array<OutFirstRunner>
    /** The loop number; used to track if blocks are updated. Incremented on [processOnce] */
    protected var loopNumber: Int = -1
        private set
    /** The [SystemValues] to be given to the blocks */
    protected abstract val systemValues: SystemValues

    init {
        allRunners = config.connections.keys.associateWith {
            if (it.processing === Block.Processing.OUT_FIRST) {
                OutFirstRunner(it)
            } else {
                InFirstRunner(it)
            }
        }
        allRunners.forEach { (block, runner) ->
            runner.sources.fillWith { i ->
                config.connections.getValue(block).sources[i]?.let {
                    val sourceRunner = allRunners[it.block] ?: throw IllegalArgumentException(
                        "Configuration has a block ($block) that references " +
                                "another block not in the configuration ($it.block)"
                    )
                    Source(sourceRunner, it.index)
                }
            }
        }
        alwaysRun = allRunners.values
            .filterIsInstance<InFirstRunner>()
            .filter { it.block.processing == Block.Processing.ALWAYS }
            .toTypedArray()
        outFirst = allRunners.values.filterIsInstance<OutFirstRunner>().toTypedArray()
    }

    /** `init`s all the blocks. */
    protected open fun init() {
        loopNumber = 0
        allRunners.values.forEach { it.init() }
    }

    /** Process through all the blocks once, also increments [loopNumber] */
    protected fun processOnce() {
        outFirst.forEach { it.processBlock() }
        alwaysRun.forEach { it.ensureProcessed() }
        outFirst.forEach { it.fillInputs() }
        loopNumber++
    }

    /** Stops all the blocks and does some cleanup. */
    protected open fun stop() {
        loopNumber = 0
        allRunners.values.forEach { it.stop() }
    }

    private class Source(val runner: Runner, val index: Int) {
        fun get(): Any? = runner.getOutput(index)
    }

    /** A wrapper around a [block] that runs it. @see [InFirstRunner], [OutFirstRunner] */
    private abstract inner class Runner(@JvmField val block: Block) {

        /** An array of [Runner]s, where the values given to the blocks will be polled from. */
        val sources: Array<Source?> = arrayOfNulls(block.numInputs) //NPE prone. Be wary.
        /** Where the outputs from this block are stored. */
        @JvmField
        protected val outputs: Array<Any?> = arrayOfNulls(block.numOutputs)
        private val values = Context()

        inner class Context : Block.ExtendedContext {
            override val isFirstTime: Boolean
                get() = this@BlockRunner.loopNumber == 0

            override val <T> Block.Input<T>.get: T
                get() {
                    require(this.block === this@Runner.block) {
                        "Attempted to get input from $this while current block is ${this@Runner.block}"
                    }
                    return getInput(index).uncheckedCast()
                }
            override var <T> Block.Output<T>.set: T
                @Suppress("OverridingDeprecatedMember")
                get() = throw UnsupportedOperationException("Cannot get output value via set.")
                set(value) {
                    require(block === this@Runner.block) {
                        "Attempted to set output to $this while current block is ${this@Runner.block}"
                    }
                    require(outputs[index] === NO_VALUE) {
                        "Already set value of $this to ${outputs[index]}"
                    }
                    outputs[index] = value
                }
            @Suppress("SimpleRedundantLet") //is not redundant.
            override val <T> Block.Output<T>.get: T
                get() = allRunners[block]?.let {
                    it.getOutput(index).uncheckedCast<T>()
                } ?: throw IllegalArgumentException("Attempted to get output $this for a block not in system")
            override val loopTime: Double get() = systemValues.loopTime
            override val totalTime: Double get() = systemValues.totalTime
            override val loopNumber: Int get() = systemValues.loopNumber
        }

        /** Initializes this runner, and the block. */
        open fun init() {
            outputs.fill(null)
            block.init()
        }

        /** Stops this runner, and the block. */
        open fun stop() {
            block.stop()
            outputs.fill(null)
        }

        /** Gets the input to this block, via index. No guarantees on type safety. */
        protected abstract fun getInput(index: Int): Any?

        /** Gets output from this block. May or may not process. */
        abstract fun getOutput(index: Int): Any?

        /** Processes the block. */
        fun processBlock() {
            outputs.fill(NO_VALUE)
            block.process(values)
            outputs.indexOf(NO_VALUE).let {
                check(it == -1) {
                    val output = block.outputs[it]
                    "No value given for $output when processed."
                }
            }
        }

        @Suppress("NOTHING_TO_INLINE")
        private inline fun Block.process(context: Block.Context) = context.process()
    }

    private inner class InFirstRunner(block: Block) : Runner(block) {
        /** The last loop number this has been processed */
        private var lastProcess: Int = -1
        /** Verifies no loops */
        private var processing = false

        override fun init() {
            super.init()
            lastProcess = -1
            processing = false
        }

        override fun getInput(index: Int): Any? = sources[index]?.get()

        /** Gets output from this block. */
        override fun getOutput(index: Int): Any? {
            ensureProcessed()
            return outputs[index]
        }

        fun ensureProcessed() {
            if (lastProcess == loopNumber) return

            check(!processing) { "Loop in block system" }

            processing = true
            processBlock()
            processing = false

            lastProcess = loopNumber
        }
    }

    private inner class OutFirstRunner(block: Block) : Runner(block) {
        private val inputs = fixedSizeMutableListOfNulls<Any>(block.numInputs)

        override fun init() {
            super.init()
            inputs.fill(null)
        }

        override fun getInput(index: Int): Any? = inputs[index]

        override fun getOutput(index: Int): Any? = outputs[index]

        fun fillInputs() {
            inputs.fillWith {
                sources[it]?.get()
            }
        }
    }

    private companion object {
        val NO_VALUE = Any()
    }
}