package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.OUT_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.util.mappedView
import java.util.*
import kotlin.collections.ArrayList
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
import org.futurerobotics.jargon.control.Block as RawBlock


/**
 * An exception thrown when an illegal configuration is detected.
 */
class IllegalBlockConfigurationException
@JvmOverloads constructor(message: String? = null) : RuntimeException(message)

private const val NOT_PROCESSED = 0
private const val PROCESSING = 1
private const val STORED = 2
private const val STORED_PROCESS_NOW = 3
private const val STORED_PROCESSING = 4
private const val PROCESSED = 5

/**
 * Builds a [BlockSystem] using a nice DSL, connecting inputs and outputs.
 */
@Suppress("UNCHECKED_CAST")
class BlocksBuilder {
    private val builderBlocks = ArrayList<Block>()

    /**
     * Ensures the [block] is added to the system, and returns a new or preexisting [Block] representing it.
     */
    infix fun add(block: RawBlock): Block {
        builderBlocks.forEach {
            if (it.block === block) return it
        }
        return Block(block, builderBlocks.size).also {
            builderBlocks += it
        }
    }

    /**
     * Ensures the current block is added to the system, and returns a new or preexisting [Block] representing it.
     */
    @JvmName("addExtension")
    fun RawBlock.add(): Block = add(this)

    /** returns if the given [this@isAdded] already exists in this builder. */
    fun RawBlock.isAdded(): Boolean = builderBlocks.any { it.block === this }

    /** Connects a [Input] and an [Output] together in this builder. */
    fun <T> connect(input: Input<in T>, output: Output<T>) {
        require(input.owner === this) { "Input must come from this builder!" }
        require(output.owner === this) { "Output must come from this builder!" }
        val inputBlock = input.block
        val outputBlock = output.block
        check(inputBlock.inputSources[input.ioIndex] == null) { "Input at $input already connected!" }
        inputBlock.inputSources[input.ioIndex] = output
        outputBlock.isOutConnected[output.ioIndex] = true
    }

    /**
     * Represents a connection from nothing.
     *
     * (This is a [constant] with the value null)
     */
    val none: Output<Nothing?> by lazy { null.constant() }
        @JvmName("none") get


    /**
     * Adds a [Constant] holding the given [this], and returns the constant's output.
     */
    fun <T> T.constant(): Output<T> = add(Constant(this))(0)

    /**
     * Adds and returns a [Monitor] monitoring the value of the given input.
     *
     * *** The given input must already be connected first ***
     */
    fun <T> monitor(input: Input<T>): Monitor<T> = monitor(
        input.source ?: throw IllegalStateException("Input has not been connected!")
    )

    /**
     * Adds and returns a [Monitor] monitoring the given output.
     */
    fun <T> monitor(output: Output<T>): Monitor<T> = Monitor<T>().apply {
        add(this).let {
            output connectTo it.input(0)
        }
    }

    /**
     * Combines two outputs, [output1] and [output2], though the given [CombineBlock],
     * and returns it's output.
     */
    fun <A, B, R> combine(output1: Output<A>, output2: Output<B>, combineBlock: CombineBlock<A, B, R>): Output<R> {
        require(!combineBlock.isAdded())
        { "block cannot already be added, else connectedness cannot be guaranteed" }
        add(combineBlock).let {
            it.connectAll(output1, output2)
            return it.output(0)
        }
    }

    /**
     * Combines two outputs, [output1] and [output2], though the given block,
     * which must have 2 inputs and 1 output, and returns it's output.
     */
    fun <A, B, R> combine(output1: Output<A>, output2: Output<B>, block: RawBlock): Output<R> {
        require(!block.isAdded())
        { "block cannot already be added, else connectedness cannot be guaranteed" }
        require(block.numInputs == 2) { "block ($block) must have 2 inputs, got ${block.numInputs}" }
        require(block.numOutputs == 1) { "block ($block) must have 1 output, got ${block.numOutputs}" }
        add(block).let {
            it.connectAll(output1, output2)
            return it(0)
        }
    }

    /**
     * Combines two outputs, [output1] and [output2], though the given combination function,
     * and returns it's output.
     */
    inline fun <A, B, R> combine(
        output1: Output<A>,
        output2: Output<B>,
        crossinline combination: (A, B) -> R
    ): Output<R> {
        return combine(output1, output2, CombineBlock(combination))
    }

    /**
     * The special output with the time elapsed of the previous loop in seconds; or
     * Double.NaN if this is the first loop run.
     * @see [LoopTime]
     */
    val loopTime: Output<Double> by lazy { add(LoopTime()).output<Double>() }
        @JvmName("loopTime") get

    /**
     * The special output with the number of the current loop, starting with 0.
     * @see [LoopNumber]
     */
    val loopNumber: Output<Int> by lazy { add(LoopNumber()).output<Int>() }
        @JvmName("loopNumber") get

    /**
     * The special input, that when given a value of "true", tells the system to shut down.
     */
    val shutdown: Input<Boolean?> by lazy { add(Shutdown()).input<Boolean?>() }
        @JvmName("shutdown") get

    /**
     * Validates and builds the system.
     *
     * @return the built [BlockSystem]
     */
    fun build(): BlockSystem {
        //despite what you might see, this is anything but a fun build()
        initialCheck()
        traceAll()
        return finishBuild()
    }

    //used in build.
    private var built = false
    private var nodeBlocks = builderBlocks.mappedView { it.node }
    private val needTrace: Queue<NodeBlock> = LinkedList<NodeBlock>()

    private fun initialCheck() {
        check(!built) { "Already built" }
        built = true
    }

    private fun traceAll() {
        val alwaysRun = nodeBlocks.filter { it.processing == ALWAYS }
        needTrace += alwaysRun
        while (needTrace.isNotEmpty()) {
            needTrace.remove().let {
                if (it.traceStatus == STORED)
                    it.traceStatus = STORED_PROCESS_NOW
                trace(it)
            }
        }
    }

    /** Traces (dfs-type thing) all the inputs at the [block]; marking used blocks and making sure there are no
     * unresolvable loops.*/
    private fun trace(block: NodeBlock): Unit = block.run {
        when (traceStatus) {
            PROCESSED, STORED, STORED_PROCESSING -> return
            PROCESSING -> { //can't get inputs if needs inputs
                throw IllegalBlockConfigurationException(
                    "Loop at component $block. Consider making a block in " +
                            "the loop out first (or adding one to the loop), or breaking " +
                            "up the loop."
                )
            }
            STORED_PROCESS_NOW -> {
                traceStatus = STORED_PROCESSING
                inputSources.forEach { trace(it.block.node) }
                traceStatus = PROCESSED
            }
            NOT_PROCESSED -> {
                if (inOutOrder == OUT_FIRST) {
                    //if out first, store outputs first.
                    traceStatus = STORED
                    needTrace.add(this) //and update later.
                } else {
                    traceStatus = PROCESSING
                    inputSources.forEach { trace(it.block.node) }
                    traceStatus = PROCESSED
                }
            }
            else -> throw AssertionError()
        }
    }


    private fun finishBuild(): BlockSystem {
        val usedBlocks = nodeBlocks.filterTo(ArrayList(nodeBlocks.size)) { it.traceStatus == PROCESSED }
        usedBlocks.sortBy { if (it.processing == ALWAYS) 0 else 1 }
        usedBlocks.forEachIndexed { index, block ->
            block.finalIndex = index
        }
        val configuredBlocks = usedBlocks.map { block ->
            val sources = block.inputSources
            val sourceBlockIndexes = sources.map { out ->
                out.block.node.finalIndex.also {
                    assert(it != -1) { "all input must have thing..." }
                }
            }.toIntArray()
            val sourceOutputIndexes = sources.mapToIntArray { it.ioIndex }
            ConnectedBlock(
                block = block.block,
                sourceBlockIndexes = sourceBlockIndexes,
                sourceOutputIndexes = sourceOutputIndexes
            )
        }

        return BlockSystem(configuredBlocks)
    }

    private inline fun <T> List<T>.mapToIntArray(mapping: (T) -> Int): IntArray {
        return IntArray(size).also {
            this.forEachIndexed { index, t ->
                it[index] = mapping(t)
            }
        }
    }

    /**
     * A wrapper around a [Block], belonging to a [BlocksBuilder], which represents it.
     * Use [input]/[output] to get representations of the inputs/outputs of this block, and
     * to connect them.
     *
     * @param block the block this wraps around
     */
    inner class Block internal constructor(
        val block: RawBlock,
        private val blockIndex: Int
    ) {
        private val inputs = Array(numInputs) { Input<Any?>(this, it) }
        private val outputs = Array(numOutputs) { Output<Any?>(this, it) }

        internal val inputSources = MutableList<Output<*>?>(numInputs) { null }
        internal val isOutConnected = BooleanArray(numOutputs)
        internal val owner get() = this@BlocksBuilder
        internal val node = NodeBlock(this)

        /** The number of inputs on this block */
        val numInputs: Int get() = block.numInputs
        /** The number of outputs on this block */
        val numOutputs: Int get() = block.numOutputs

        /** Returns an [Output] representation of this block's output [index]. */
        @JvmOverloads
        fun <T> output(index: Int = 0): Output<T> = outputs[index] as Output<T>

        /** Returns an [Input] representation of this block's input [index] */
        @JvmOverloads
        fun <T> input(index: Int = 0): Input<T> = inputs[index] as Input<T>

        /** Returns an [Output] representation of this block's output [index], same as [output] */
        operator fun <T> invoke(index: Int = 0): Output<T> = output(index)

        /** Returns an [Input] representation of this block's input [index], same as [input] */
        operator fun <T> get(index: Int = 0): Input<T> = input(index)

        /**
         * Connects all the inputs of this block to the given [outputs], in order. All inputs
         * must be given.
         */
        fun connectAll(vararg outputs: Output<Any?>) {
            require(outputs.size <= numInputs)
            { "Given number of outputs ($outputs.size) be <= input size ($.numInputs}" }
            outputs.forEachIndexed { inputIndex, output ->
                connect(input(inputIndex), output)
            }
        }
    }

    /** Common functionality of both [Input] or an [Output]. */
    abstract inner class IOIndicator protected constructor(
        internal val block: Block,
        internal val ioIndex: Int
    ) {
        internal val owner get() = this@BlocksBuilder
        /**
         * If this input/output has been connected or not
         */
        abstract val isConnected: Boolean
    }

    /**
     * Represents the Input of a [Block]
     */
    inner class Input<T> internal constructor(
        block: Block,
        inputIndex: Int
    ) : IOIndicator(block, inputIndex) {

        /** If this input has been connected or not. */
        override val isConnected: Boolean get() = source != null

        /**
         * Gets the [Output] that this input is connected from; `null` if not yet connected.
         */
        val source: Output<T>?
            get() = block.inputSources[ioIndex] as Output<T>?

        /** Connects this input from the given [output] */
        infix fun connectFrom(output: Output<T>) {
            connect(this, output)
        }

        /**
         * Creates, adds, and returns a [Monitor] monitoring this input.
         *
         * ***Input must be connected first.***
         */
        fun monitor(): Monitor<T> = monitor(this)

        override fun toString(): String {
            return "$block:in[$ioIndex]"
        }
    }

    /**
     * Represents the Output of a [Block]
     */
    inner class Output<out T> internal constructor(
        block: Block,
        outputIndex: Int
    ) : IOIndicator(block, outputIndex) {

        /**
         * If this output has been connected or not
         */
        override val isConnected: Boolean get() = block.isOutConnected[ioIndex]

        /** Connects this output to the given [input] */
        infix fun connectTo(input: Input<in T>) {
            connect(input, this)
        }

        /** Connects this output to all of the given [inputs] */
        fun connectToAll(vararg inputs: Input<in T>) {
            inputs.forEach { this connectTo it }
        }

        /**
         * Adds and connects a [Monitor] monitoring this output.
         */
        fun monitor(): Monitor<T> = monitor(this)

        /**
         * First pipes this output through the given [block] (which must be 1 input, 1 output),
         * and returns the output of that pipe.
         */
        fun <R> pipe(block: PipeBlock<in T, out R>): Output<R> {
            require(!block.isAdded())
            { "block cannot already be added, else pipe connectedness cannot be guaranteed" }
            add(block).let {
                this connectTo it[0]
                return it(0)
            }
        }

        /**
         * First pipes this output through the given [piping] function (which must be 1 input, 1 output),
         * and returns the output of that pipe.
         */
        inline fun <R> pipe(crossinline piping: (T) -> R): Output<R> {
            return pipe(PipeBlock(piping))
        }

        override fun toString(): String {
            return "$block:out($ioIndex)"
        }
    }

    internal inner class NodeBlock constructor(inner: Block) {
        val block = inner.block
        val inOutOrder = block.inOutOrder
        val processing = block.processing

        val inputSources = inner.inputSources.mappedView { it ?: none }
        var traceStatus: Int = 0

        var finalIndex = -1

        override fun toString(): String {
            return block.toString()
        }
    }

}

/**
 * DSL to build a block system.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun buildBlockSystem(configuration: BlocksBuilder.() -> Unit): BlockSystem {
    contract {
        callsInPlace(configuration, InvocationKind.EXACTLY_ONCE)
    }
    return BlocksBuilder().run {
        configuration()
        build()
    }
}