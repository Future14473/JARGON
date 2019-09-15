package org.futurerobotics.temporaryname

/**
 * Expressive debugging utilities.
 */
@Suppress("ConstantConditionIf", "NOTHING_TO_INLINE", "unused")
object Debug {

    /**
     * If debug print should be enabled.
     */
    var printDebug: Boolean = true

    /** Used to indicate breakpoint from code. For best use, set your debugger to breakpoint at this function. */
    @JvmStatic
    inline fun breakpoint() {
    }

    /**
     * breakpoints if [b] is true.
     */
    @JvmStatic
    inline fun breakIf(b: Boolean) {
        if (b) breakpoint()
    }

    /**
     * breakpoints if [b] is false.
     */
    @JvmStatic
    inline fun breakUnless(b: Boolean) {
        if (!b) breakpoint()
    }

    /** Println's to console only if [printDebug] is true. */
    @JvmStatic
    inline fun println(message: Any? = "") {
        if (printDebug) kotlin.io.println(message)
    }

    /** Prints to console only if [printDebug] is true. */
    @JvmStatic
    inline fun print(message: Any? = "") {
        if (printDebug) kotlin.io.print(message)
    }
}
