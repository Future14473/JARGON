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

/** Casts this to type [T] unchecked, but with type inference too. Use with caution. */
@Suppress("UNCHECKED_CAST", "NOTHING_TO_INLINE")
inline fun <T> Any?.uncheckedCast(): T = this as T
