package org.futurerobotics.jargon.pathing

import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.contract

/**
 * Returns a list of the result of the [mapping] function called on all possible pairs of elements from the lists
 */
@OptIn(ExperimentalContracts::class)
inline fun <A, B, R> mapAllPairs(listA: List<A>, listB: List<B>, mapping: (A, B) -> R): List<R> {
    contract {
        callsInPlace(mapping)
    }
    if (listA.isEmpty() || listB.isEmpty()) return emptyList()
    val result = ArrayList<R>(listA.size * listB.size)
    listA.forEach { a ->
        listB.forEach { b ->
            result += mapping(a, b)
        }
    }
    return result
}


