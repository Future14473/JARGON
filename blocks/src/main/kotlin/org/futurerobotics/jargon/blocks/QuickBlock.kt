package org.futurerobotics.jargon.blocks

/**
 * A block which is allowed to get the values of other blocks using [Block.ExtendedContext], as such, it has no
 * input support, but additional outputs can be defined _publicly_ via [newOutput].
 *
 *
 * The [process] function is set separately.
 *
 * This can only be created using [BlockArrangementBuilder], intended for quick calculations when designing
 * block systems.
 *
 * @property name the name of the block as reported by toString
 */
class QuickBlock internal constructor(
    processing: Processing,
    private var process: (ExtendedContext.() -> Unit)?,
    var name: String = "QuickBlock"
) : Block(processing) {

    override fun Context.process() {
        (this as? ExtendedContext)?.process() ?: error("Given context is not an ExtendedContext")
    }

    /** Creates a new output to this [QuickBlock] with the given [name] and type [T]. */
    fun <T> makeNewOutput(name: String): Output<T> = newOutput(name)

    /**
     * Sets the [process] function of this [QuickBlock].
     */
    fun setProcess(process: ExtendedContext.() -> Unit) {
        if (this.process != null) error("Process function already initialized")
        this.process = process
    }

    private fun ExtendedContext.process() {
        process?.invoke(this) ?: error("Process function not initialized.")
    }

    override fun toString(): String = name
}
