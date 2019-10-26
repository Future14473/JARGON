@file:JvmName("KotlinOnlyCodeUtil")

package org.futurerobotics.jargon.util

import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/** An inline alternative to [takeUnless] that avoids boxing of primitives. */
@UseExperimental(ExperimentalContracts::class)
inline fun <T> T.replaceIf(predicate: (T) -> Boolean, alternate: (T) -> T): T {
    contract {
        callsInPlace(predicate, InvocationKind.EXACTLY_ONCE)
        callsInPlace(alternate, InvocationKind.AT_MOST_ONCE)
    }
    return if (predicate(this)) alternate(this) else this
}

/**
 * Calls the specified [block] with [p1] as its argument and returns its result.
 * Similar to [kotlin.let]. Useful for changing variable names.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T, R> let(p1: T, block: (T) -> R): R {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }
    return block(p1)
}

/**
 * Calls the specified [block] with [p1], [p2] as its arguments and returns its result.
 * Similar to [kotlin.let]. Useful for changing variable names together or avoiding nested let statements.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T1, T2, R> let(p1: T1, p2: T2, block: (T1, T2) -> R): R {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }
    return block(p1, p2)
}

/**
 * Calls the specified [block] with [p1], [p2], [p3] as its arguments and returns its result.
 * Similar to [kotlin.let]. Useful for changing variable names together or avoiding nested let statements.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T1, T2, T3, R> let(p1: T1, p2: T2, p3: T3, block: (T1, T2, T3) -> R): R {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }
    return block(p1, p2, p3)
}

/**
 * Calls the specified [block] with [p1], [p2], [p3], [p4] as its arguments and returns its result.
 * Similar to [kotlin.let]. Useful for changing variable names together or avoiding nested let statements.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T1, T2, T3, T4, R> let(p1: T1, p2: T2, p3: T3, p4: T4, block: (T1, T2, T3, T4) -> R): R {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }
    return block(p1, p2, p3, p4)
}

/** Casts this to type [T] unchecked, but with type inference too. Use with caution. */
@Suppress("UNCHECKED_CAST", "NOTHING_TO_INLINE")
inline fun <T> Any?.unsafeCast(): T = this as T
