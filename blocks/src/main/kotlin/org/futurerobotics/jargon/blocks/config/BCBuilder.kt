package org.futurerobotics.jargon.blocks.config

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Input
import org.futurerobotics.jargon.blocks.Block.Output
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.QuickBlock
import org.futurerobotics.jargon.blocks.SingleInputBlock
import org.futurerobotics.jargon.blocks.functional.Constant
import org.futurerobotics.jargon.blocks.functional.Delay
import org.futurerobotics.jargon.blocks.functional.Monitor
import org.futurerobotics.jargon.blocks.functional.Recording
import org.futurerobotics.jargon.util.asUnmodifiableList
import org.futurerobotics.jargon.util.asUnmodifiableMap
import java.util.*

/** DSL marker for [BCBuilder] */
@DslMarker
annotation class BCBuilderDsl

/**
 * A builder for a [BlockConfig], using a nice dsl.
 *
 * For kotlin, you should use the scoping function [with] or [apply] on this to configure, as then
 * several member extension functions can be used.
 *
 * This primarily works by using [connect], [from], or [into] to connect the inputs and outputs of blocks.
 * One can also:
 * - create common blocks using [constant], [recording], [monitor], [delay].
 * - can create quick transformations using [pipe] or [generate]
 * - can create their own, one time use multi input block using [quickBlock].
 *
 * Kotlin people can extend functionality by creating extension functions on [BCBuilder].
 */
@BCBuilderDsl
class BCBuilder : AbstractReadOnlyBlockConfig() {

    private var built: Boolean = false
    private val _connections: MutableMap<Block, BlockConnectionsImpl> = IdentityHashMap()
    override val connections: Map<Block, BlockConnections>
        get() = _connections.asUnmodifiableMap()
    private val Block.connections: BlockConnectionsImpl
        get() = _connections.getOrPut(this) {
            check(!built) { "BlockConfig already built; cannot add block $this" }
            BlockConnectionsImpl(this)
        }

    private class BlockConnectionsImpl(override val block: Block) : BlockConnections {
        val internalSources = arrayOfNulls<Output<*>>(block.numInputs)

        override val sources: List<Output<*>?>
            get() = internalSources.asList().asUnmodifiableList()
    }

    /**
     * Adds the given [block] to the system, if not already.
     *
     * This is for blocks that aren't connected to any other block.
     */
    fun add(block: Block) {
        block.connections
    }

    /**
     * Connects the given [input] and [output] together. Blocks will be added if necessary.
     */
    fun <T> connect(input: Input<in T>, output: Output<out T>) {
        check(!built) { "BlockConfig already built; cannot connect input $input and output $output" }
        val inputCon = input.block.connections
        add(output.block)
        val previousSource = inputCon.internalSources[input.index]
        check(previousSource == null) { "Input $input already connected to $previousSource; cannot connect to $output" }

        inputCon.internalSources[input.index] = output
    }

    /** Connects this input _from_ the given [output]. Adds blocks to the config if necessary. */
    infix fun <T> Input<in T>.from(output: Output<out T>) {
        connect(this, output)
    }

    /** Connects this output _into_ the given [input]. Adds blocks to the config if necessary.*/
    infix fun <T> Output<T>.into(input: Input<in T>) {
        input from this
    }

    /** Connects this output _into all_ of the given [inputs]. Adds blocks to the config if necessary.*/
    fun <T> Output<T>.intoAll(vararg inputs: Input<in T>) {
        inputs.forEach { it from this }
    }

    /**
     * Builds the [BlockConfig].
     *
     * After building, any included blocks can no longer be added to any other [BlockConfig], and this builder
     * can no longer be modified.
     */
    fun build(): BlockConfig {
        check(!built) { "Already built!" }
        built = true
        //really enforce no changing; only reading.
        val delegate = object : ReadOnlyBlockConfig by this {}
        connections.keys.forEach { it.finalizeConfig(delegate) }
        //after built is true, can no longer mutate, so can directly use unmodifiable map
        return BlockConfig(connections)
    }

    /**
     * Builds a [BlockConfig], by running the given [configuration], then calling [build].
     */
    @BCBuilderDsl
    inline fun build(configuration: BCBuilder.() -> Unit): BlockConfig {
        configuration()
        return build()
    }

    /**
     * Gets a [Output] representing where this [Input] is connected _from_, or `null` if not yet connected.
     *
     * This is often useful to use any of the [Output] utility functions instead on the inputs to a particular
     * block.
     */
    fun <T> Input<T>.source(): Output<out T>? = sourceOf(this)

    //utilities

    /** Runs the given [configuration] block on [this] block, and returns it. */
    @BCBuilderDsl
    inline operator fun <T : Block> T.invoke(configuration: T.() -> Unit): T = apply(configuration)

    /**
     * Creates a new [PipeBlock] with the given [piping], connects this output to it, and returns the piping's
     * output. Useful for quick transformations. For instance:
     *
     * in kotlin:
     * ```kotlin
     * with(builder){
     *  val pose: BCBuilder.Output<Pose2d> = SomethingThat().outputsPose()
     *  val x = a.pipe { it.x } //pipes to x value of pose
     * }
     * ```
     * in java:
     * ```java
     * BCBuilder.Output<Pose2d> pose = builder.get(SomethingThat().outputsPose);
     * BCBuilder.Output<Double> x = pose.pipe(p->p.x); //pipes to x value of pose
     *
     * ```
     * @see listen
     * @see generate
     */
    @BCBuilderDsl
    inline fun <T, R> Output<T>.pipe(crossinline piping: Block.Context.(T) -> R): Output<R> =
        pipe(PipeBlock.with(piping))

    /**
     * Connects this output into the given [pipeBlock]'s input, and returns the [pipeBlock]'s output.
     *
     * Useful for quick transformations using already existing [pipeBlock]s.
     * @see listen
     * @see generate
     */
    fun <T, R> Output<T>.pipe(pipeBlock: PipeBlock<T, R>): Output<R> {
        this into pipeBlock.input
        return pipeBlock.output
    }

    /**
     * Creates and connects a [Delay] block to this output, and returns the delay's output.
     *
     * Useful for breaking up loops when it's ok for a block to delayed value.
     */
    fun <T> Output<T>.delay(initialValue: T): Output<T> = Delay(
        initialValue
    )() { input from this@delay }.output

    /**
     * Creates and connects a [Monitor] block from this output.
     * @see recording
     */
    fun <T> Output<T>.monitor(): Monitor<T> = Monitor<T>().also { this into it.input }

    /**
     * Creates and connects a [Recording] block from this output.
     * @see monitor
     */
    fun <T> Output<T>.recording(): Recording<T> = Recording<T>().also { this into it.input }

    /**
     * Creates a new block that runs the given [func] every loop, given values from this output.
     * @see pipe
     */
    inline fun <T> Output<T>.listen(crossinline func: Block.Context.(T) -> Unit) {
        object : SingleInputBlock<T>(Processing.ALWAYS) {
            override fun Context.process(input: T) {
                func(input)
            }

            override fun toString(): String = "listen Block"
        }.let { this into it.input }
    }

    /**
     * Creates a [Constant] with the given [value], and returns the output (which will always be the constant
     * value).
     */
    fun <T> constant(value: T): Output<T> = Constant(value).output

    /**
     * Creates a block that creates values using the given [producer] function.
     *
     * [Block.ExtendedContext] is supported, so that one can get outputs of other blocks.
     *
     * Keep in mind that all referenced blocks need to be added to the system.
     *
     * @see runBlock
     */
    @JvmOverloads
    fun <T> generate(
        processing: Block.Processing = Block.Processing.LAZY,
        producer: Block.ExtendedContext.() -> T
    ): Output<T> {
        val block = quickBlock(processing)
        val output = block.newOutput<T>()
        block.setProcess {
            output.set = producer()
        }
        return output
    }

    /**
     * Creates a block that that simply runs the given [block].
     *
     * [Block.ExtendedContext] is supported to get outputs of other blocks.
     *
     * @see generate
     */
    fun runBlock(block: Block.ExtendedContext.() -> Unit) {
        add(QuickBlock(Block.Processing.ALWAYS, block))
    }

    /**
     * Creates a [QuickBlock], for one place use blocks.
     *
     * Outputs can then be defined using [QuickBlock.newOutput].
     * Then, [QuickBlock.setProcess] should be called to set the behavior of the process function.
     * [Block.ExtendedContext] is supported to get outputs of other blocks.
     *
     * As a very quick example:
     *
     * In kotlin:
     * ```kotlin
     * val config = BCBuilder.build {
     *    val a = constant(1)
     *    val b = constant(2)
     *
     *    val adderAndDiff = quickBlock()
     *    val sum = adderAndDiff.newOutput<Int>("sum")
     *    val diff = adderAndDiff.newOutput<Int>("diff")
     *    adderAndDiff.setProcess {
     *      sum.set = a.get + b.get
     *      diff.set = a.get - b.get
     *    }
     *
     * }
     * ```
     *
     * In java (use java 10 type inference!!! works on android too!!!)
     * ``` java
     *  var bd = new BCBuilder();
     *  var a = bd.constant(1);
     *  var b = bd.constant(2);
     *
     *  var adderAndDiff = bd.quickBlock();
     *  var sum = adderAndDiff.newOutput<Integer>("sum");
     *  var diff = adderAndDiff.newOutput<Integer>("diff");
     *  adderAndDiff.setProcess(ctx -> {
     *     ctx.set(sum, ctx.get(a) + ctx.get(b));
     *     ctx.set(diff, ctx.get(a) - ctx.get(b));
     *  return Unit.INSTANCE;
     *  });
     *  var config = bd.build();
     *
     * ```
     * The default processing is [Block.Processing.LAZY]
     * @see generate
     */
    @JvmOverloads
    fun quickBlock(processing: Block.Processing = Block.Processing.LAZY) = QuickBlock(processing, null)
}

