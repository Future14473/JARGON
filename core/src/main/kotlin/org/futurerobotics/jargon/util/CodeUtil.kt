package org.futurerobotics.jargon.util

import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/**
 * Runs the block, then returns [this] as the type [S].
 *
 * Intended for use in builder methods that return itself (self type).
 *
 * If [this] is a final/non-open class, type inference is smart enough to make the return type the same type
 * as [this]. However one must be careful when doing explicit types as it will cast to any type you give it.
 */
@OptIn(ExperimentalContracts::class)
inline fun <T, S : T> T.builder(block: () -> Unit): S {
    contract {
        callsInPlace(block, InvocationKind.EXACTLY_ONCE)
    }
    block()
    @Suppress("UNCHECKED_CAST")
    return this as S
}
