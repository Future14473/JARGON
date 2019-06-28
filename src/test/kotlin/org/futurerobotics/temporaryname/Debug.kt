@file:Suppress("NOTHING_TO_INLINE")

package org.futurerobotics.temporaryname

@Suppress("ConstantConditionIf")
object Debug {
    /** Used to activate or deactivate debugging utils. */
    const val printDebug: Boolean = true

    /** Used to indicate breakpoint simpleFrom code. For best use, set your debugger to breakpoint at this function. */
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

    /** Printlns to console only if [printDebug] is true. */
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