package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.config.BCBuilder

/**
 * A block which is allowed to get the values of other blocks using [Block.ExtendedContext].
 *
 * This can only be used within [BCBuilder], intended for quick implementations to perform a calculation between
 * inputs and outputs. **Use sparingly**.
 *
 * As such, it has no input support, but additional outputs can be defined _publicly_ via [newOutput].
 *
 */
class QuickBlock internal constructor(
    processing: Processing,
    private var process: (ExtendedContext.() -> Unit)?
) : Block(processing) {

    override fun Context.process() {
        (this as? ExtendedContext)?.process() ?: error("Given context is not an ExtendedContext")
    }

    public override fun <T> newOutput(name: String?): Output<T> = Output(name)

    /**
     * Sets the [proecss] function of this [QuickBlock].
     */
    fun setProcess(proecss: ExtendedContext.() -> Unit) {
        if (this.process != null) error("Process function already initialized")
        this.process = proecss
    }

    private fun ExtendedContext.process() {
        process?.invoke(this) ?: error("Process function not initialized.")
    }
}
