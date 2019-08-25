@file:JvmName("CodeUtil")

package org.futurerobotics.temporaryname.util

/** An inline alternative to [takeIf] that avoids boxing of primitives. */
//@ExperimentalContracts
inline fun <T> T.replaceIf(condition: (T) -> Boolean, alternate: (T) -> T): T {
//    contract {
//        callsInPlace(condition, InvocationKind.EXACTLY_ONCE)
//        callsInPlace(alternate, InvocationKind.AT_MOST_ONCE)
//    }
    return if (condition(this)) alternate(this) else this
}

/**
 * Calls the [block], then returns this.
 *
 * Useful to put emphasis on return value; for example in builders
 * ```
 * fun addFoo(): ThisType = /*this*/.after {
 *   ...
 * }
 * ```
 * also equivalent to `let { _ -> ... }`
 */
//@ExperimentalContracts
inline fun <T> T.after(block: () -> Unit): T {
//    contract {
//        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
//    }
    block()
    return this
}

/**
 * Calls the specified [block] with [p1] as its argument and returns its result.
 * Similar to [kotlin.let] or [kotlin.with]. Useful for changing variable names.
 */
//@ExperimentalContracts
inline fun <T, R> let(p1: T, block: (T) -> R): R {
//    contract {
//        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
//    }
    return block(p1)
}

/**
 * Calls the specified [block] with [p1], [p2] as its arguments and returns its result.
 * Similar to [kotlin.let] or [kotlin.with]. Useful for changing variable names together or nested let statements.
 */
//@ExperimentalContracts
inline fun <T, U, R> let(p1: T, p2: U, block: (T, U) -> R): R {
//    contract {
//        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
//    }
    return block(p1, p2)
}

/**
 * Calls the specified [block] with [p1], [p2], [p3] as its arguments and returns its result.
 * Similar to [kotlin.let] or [kotlin.with]. Useful for changing variable names together or nested let statements.
 */
//@ExperimentalContracts
inline fun <T, U, V, R> let(p1: T, p2: U, p3: V, block: (T, U, V) -> R): R {
//    contract {
//        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
//    }
    return block(p1, p2, p3)
}

/**
 * Calls the specified [block] with [p1], [p2], [p3], [p4] as its arguments and returns its result.
 * Similar to [kotlin.let] or [kotlin.with]. Useful for changing variable names together or nested let statements.
 */
//@ExperimentalContracts
inline fun <T, U, V, W, R> let(p1: T, p2: U, p3: V, p4: W, block: (T, U, V, W) -> R): R {
//    contract {
//        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
//    }
    return block(p1, p2, p3, p4)
}

/**
 * Get's the stack frame [numUp] from here.
 */
fun getCallerStackFrame(numUp: Int = 1): StackTraceElement = Thread.currentThread().stackTrace[numUp + 1]

