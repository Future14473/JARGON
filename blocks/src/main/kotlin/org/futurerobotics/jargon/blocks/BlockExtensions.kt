@file:JvmName("Blocks")

package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.functional.Constant
import org.futurerobotics.jargon.blocks.functional.Delay
import org.futurerobotics.jargon.blocks.functional.Monitor
import org.futurerobotics.jargon.blocks.functional.Recording

/**
 * Creates and connects a [Recording] block from this output.
 * @see monitor
 */
fun <T> Block.Output<T>.recording(): Recording<T> = Recording<T>().also { this into it.input }

/**
 * Creates and connects a [Monitor] block from this output.
 * @see recording
 */
fun <T> Block.Output<T>.monitor(): Monitor<T> = Monitor<T>().also { this into it.input }

/**
 * Creates and connects a [Delay] block to this output, and returns the delay's output.
 *
 * Useful for breaking up loops when it's ok for a block to delayed value.
 */
fun <T> Block.Output<T>.delay(initialValue: T): Block.Output<T> =
    Delay(initialValue).apply { input from this@delay }.output

/**
 * Connects this output into the given [pipeBlock]'s input, and returns the [pipeBlock]'s output.
 *
 * Useful for quick transformations using already existing [pipeBlock]s.
 * @see listen
 * @see generate
 */
fun <T, R> Block.Output<T>.pipe(pipeBlock: PipeBlock<T, R>): Block.Output<R> {
    this into pipeBlock.input
    return pipeBlock.output
}

/**
 * Creates a new [PipeBlock] with the given [piping], connects this output to it, and returns the piping's
 * output. Useful for quick transformations. For instance:
 *
 * in kotlin:
 * ```kotlin
 *  val pose: Block.Output<Pose2d> = SomethingThat().outputsPose()
 *  val x = pose.pipe { it.x } //pipes to x value of pose
 *
 * ```
 * in java:
 * ```java
 * Block.Output<Pose2d> pose = builder.get(SomethingThat().outputsPose);
 * Block.Output<Double> x = builder.pipe<<(pose, (ctx, p)->p.x); //pipes to x value of pose
 *
 * ```
 * @see listen
 * @see generate
 */
inline fun <T, R> Block.Output<T>.pipe(crossinline piping: Block.Context.(T) -> R): Block.Output<R> =
    pipe(PipeBlock.with(piping))

/**
 * Creates a [Constant] with the given [value], and returns the output (which will always be the constant
 * value).
 */
fun <T> constant(value: T): Block.Output<T> = Constant(value).output

/**
 * Creates a block that creates values using the given [producer] function.
 *
 * **Keep in mind that all referenced blocks need to be /[Block.link]ed with this output or added to the same system.**
 *
 * [Block.ExtendedContext] is supported, so that one can get outputs of other blocks without directly connecting.
 */
@JvmOverloads
fun <T> generate(
    processing: Block.Processing = Block.Processing.LAZY,
    producer: Block.ExtendedContext.() -> T
): Block.Output<T> = object : QuickBlock(processing) {
    val output = newOutput<T>()
    override fun ExtendedContext.process() {
        output.set = producer()
    }
}.output

/**
 * Creates a new block that runs the given [action] every loop, passing in values from [this] output.
 * @see generate
 * @see pipe
 */
inline fun <T> Block.Output<T>.listen(crossinline action: Block.Context.(T) -> Unit) {
    object : Block(Processing.ALWAYS) {
        val input = newInput<T>()
        override fun Context.process() {
            action(input.get)
        }

        override fun toString(): String = "listen"
    }.let { this into it.input }
}
