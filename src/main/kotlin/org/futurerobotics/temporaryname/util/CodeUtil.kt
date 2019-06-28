@file:JvmName("CodeUtil")

package org.futurerobotics.temporaryname.util

/** An inline alternative to [takeIf] that avoids boxing of primitives. */
inline fun <T> T.replaceIf(condition: (T) -> Boolean, alternate: (T) -> T): T =
    if (condition(this)) alternate(this) else this

/**
 * Get's the stack frame [numUp] from here.
 */
fun getCallerStackFrame(numUp: Int = 1): StackTraceElement = Thread.currentThread().stackTrace[numUp + 1]


/** Checks if the values of this iterator is sorted, inlined. */
fun <T> Iterable<T>.isSorted(): Boolean where T : Comparable<T> = inlineIsSortedBy { it }

/** Checks if the values of this iterator is sorted by [which], inlined. */
inline fun <T, V : Comparable<V>> Iterable<T>.inlineIsSortedBy(which: (T) -> V): Boolean = iterator().let {
    var prev = if (it.hasNext()) which(it.next()) else return true
    while (it.hasNext()) {
        val cur = which(it.next())
        if (cur < prev) return false
        prev = cur
    }
    return true
}

/**
 * Returns a list of all possible a-b pairs
 */
fun <A, B> allPairs(a: List<A>, b: List<B>): List<Pair<A, B>> {
    if (a.isEmpty() || b.isEmpty()) return emptyList()
    val result = ArrayList<Pair<A, B>>(a.size * b.size)
    a.forEach { a ->
        b.forEach { b ->
            result += a to b
        }
    }
    return result
}