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

/** If [condition] is true, returns [alternate], else returns [this]. */
@UseExperimental(ExperimentalContracts::class)
inline fun <T> T.replaceIf(condition: Boolean, alternate: (T) -> T): T {
    contract {
        callsInPlace(alternate, InvocationKind.AT_MOST_ONCE)
    }
    return if (condition) alternate(this) else this
}

/** Casts this to type [T] unchecked, with type inference. Use with caution. */
@Suppress("UNCHECKED_CAST", "NOTHING_TO_INLINE")
inline fun <T> Any?.uncheckedCast(): T = this as T

/**
 * Runs the block, then returns [this] as [S].
 *
 * Intended for use in builder methods that return itself.
 *
 * If [this] is a final/non-open class, type inference is smart enough to put the return
 * type as this. However one must be careful when doing explicit types as it will cast
 * to any type you give it.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T, S : T> T.builder(block: () -> Unit): S {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }
    block()
    return this.uncheckedCast()
}
