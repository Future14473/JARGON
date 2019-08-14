
package org.futurerobotics.temporaryname

@Suppress("ConstantConditionIf", "NOTHING_TO_INLINE", "unused")
object Debug {
    /** Used to activate or deactivate debugging utils. */
    const val printDebug: Boolean = true

    /** Used to indicate breakpoint from code. For best use, set your debugger to breakpoint at this function. */
    @JvmStatic
    inline fun breakpoint() {
    }

    /** If the condition [b] is false, causes [breakpoint] */
    @JvmStatic
    inline fun breakUnless(b: Boolean) {
        if (!b) breakpoint()
    }

    /** If the condition [b] is True, causes [breakpoint] */
    @JvmStatic
    inline fun breakIf(b: Boolean) {
        if (b) breakpoint()
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
