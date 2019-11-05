package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.config.BCBuilder

/**
 * A block which is allowed to get the values of other blocks using [Block.ExtendedContext].
 *
 * This is mainly used within [BCBuilder].
 *
 * As such, it has no input support, but additional outputs can be defined _publicly_.
 *
 * Intended for quick implementations only when it may be inconvenient otherwise. Use with caution.
 */
class QuickBlock internal constructor(
    override val processing: Processing,
    private var run: (ExtendedContext.() -> Unit)?
) : Block() {

    override fun Context.process() {
        (this as? ExtendedContext)?.process() ?: error("Given context is not an Extended Context")
    }

    /**
     * Adds a new output to this block, with the given [name].
     *
     * If given [name] is null, may show up as ?? (name cannot be found via reflection)
     */
    @JvmOverloads
    fun <T> newOutput(name: String? = null): Output<T> = Output(name)

    /**
     * Sets the [run] function of this [QuickBlock].
     */
    fun setProcess(run: ExtendedContext.() -> Unit) {
        if (this.run != null) error("Run function already initialized")
        this.run = run
    }

    private fun ExtendedContext.process() {
        run?.invoke(this) ?: error("Run function not initialized.")
    }
}
