package org.futurerobotics.temporaryname.util

/**
 * Creates a new list where the elements are viewed as a [mapping] of the original list.
 */
inline fun <T, R> List<T>.mappedView(crossinline mapping: (T) -> R): List<R> = object : AbstractList<R>() {
    override val size: Int
        get() = this@mappedView.size

    override fun get(index: Int): R {
        return mapping(this@mappedView[index])
    }
}

/**
 * Maps all values of this [MutableList] through the [mapping] function, replacing the values
 * with their results.
 */
inline fun <T> MutableList<T>.localMap(mapping: (T) -> T) {
    val iterator = listIterator()
    while (iterator.hasNext()) {
        iterator.set(mapping(iterator.next()))
    }
}

/** @return true if the values given by this iterable are sorted. */
fun <T> Iterable<T>.isSorted(): Boolean where T : Comparable<T> = isSortedBy { it }

/** @return true if the values of this iterator is sorted by [which], inlined. */
inline fun <T, V : Comparable<V>> Iterable<T>.isSortedBy(which: (T) -> V): Boolean = iterator().let {
    var prev = if (it.hasNext()) which(it.next()) else return true
    while (it.hasNext()) {
        val cur = which(it.next())
        if (cur < prev) return false
        prev = cur
    }
    return true
}

/**
 * Runs forEach on each of the iterables [p1], [p2], zipped.
 */
inline fun <T, V> forEachZipped(p1: Iterable<T>, p2: Iterable<V>, block: (T, V) -> Unit) {
    let(p1.iterator(), p2.iterator()) { it1, it2 ->
        while (it1.hasNext() && it2.hasNext()) {
            block(it1.next(), it2.next())
        }
    }
}

/**
 * Runs forEach on each of the iterables [p1], [p2], zipped, and indexed.
 */
inline fun <T, V> forEachZippedIndexed(p1: Iterable<T>, p2: Iterable<V>, block: (index: Int, T, V) -> Unit) {
    var i = 0
    let(p1.iterator(), p2.iterator()) { it1, it2 ->
        while (it1.hasNext() && it2.hasNext()) {
            block(i++, it1.next(), it2.next())
            if (i < 0) throw ArithmeticException("Index overflow")
        }
    }
}

/**
 * Runs forEach on each of the iterables [this], [p2], zipped.
 */
inline fun <T, V> Iterable<T>.zipForEach(p2: Iterable<V>, block: (T, V) -> Unit): Unit =
    forEachZipped(this, p2, block)

/**
 * Runs forEach on each of the iterables [this], [p2], zipped.
 */
inline fun <T, V> Iterable<T>.zipForEachIndexed(p2: Iterable<V>, block: (index: Int, T, V) -> Unit): Unit =
    forEachZippedIndexed(this, p2, block)

/**
 * Returns a list of all possible pairs of elements from the lists.
 */
fun <A, B> allPairs(listA: List<A>, listB: List<B>): List<Pair<A, B>> {
    if (listA.isEmpty() || listB.isEmpty()) return emptyList()
    val result = ArrayList<Pair<A, B>>(listA.size * listB.size)
    listA.forEach { a ->
        listB.forEach { b ->
            result += a to b
        }
    }
    return result
}

/**
 * Returns a list of the result of the [mapping] function called on all possible pairs of elements from the lists
 */
inline fun <A, B, R> allPairs(listA: List<A>, listB: List<B>, mapping: (A, B) -> R): List<R> {
    if (listA.isEmpty() || listB.isEmpty()) return emptyList()
    val result = ArrayList<R>(listA.size * listB.size)
    listA.forEach { a ->
        listB.forEach { b ->
            result += mapping(a, b)
        }
    }
    return result
}

/**
 * Runs the [block] on all possible pairs of elements from the lists.
 */
//@ExperimentalContracts
inline fun <A, B> onAllPairs(listA: List<A>, listB: List<B>, block: (A, B) -> Unit) {
//    contract {
//        callsInPlace(block, InvocationKind.UNKNOWN)
//    }
    listA.forEach { a ->
        listB.forEach { b ->
            block(a, b)
        }
    }
}

