package org.futurerobotics.jargon.blocks

/**
 * A helper to create a quick block when creating block systems, that:
 *
 * - Supporting creating inputs/outputs that directly. connect to existing outputs/inputs via [inputTo] and [inputFrom]
 * - Getting outputs from other blocks directly via [Block.ExtendedContext]
 * @property name the name of the block as reported by toString
 */
abstract class QuickBlock @JvmOverloads constructor(
    processing: Processing,
    var name: String = "QuickBlock"
) : Block(processing) {

    final override fun Context.process() {
        (this as? ExtendedContext)?.process() ?: error("Process function not set.")
    }

    /**
     * [process] but with [Block.ExtendedContext]
     */
    protected abstract fun ExtendedContext.process()

    /** Adds a new output to a [QuickBlock] that also is connected from this input. */
    @JvmOverloads
    fun <T> inputTo(input: Input<in T>, name: String? = null): Output<T> =
        Output<T>(name ?: input.name ?: "??", input.type)
            .also { input from it }

    /** Adds a new input to a [QuickBlock] that also is connected from this output. */
    @JvmOverloads
    fun <T> inputFrom(output: Output<out T>, name: String? = null): Input<T> =
        Input<T>(name ?: output.name ?: "??", output.type, false)
            .also { output into it }

    override fun toString(): String = name
}
