package org.futurerobotics.jargon.util


/** Fills an array using the given [generator]. */
inline fun <T> Array<T>.fillWith(generator: (Int) -> T) {
    repeat(size) {
        this[it] = generator(it)
    }
}

inline fun <T> MutableList<T>.fillWith(generator: (Int) -> T) {
    val iterator = listIterator()
    var index = 0
    while (iterator.hasNext()) {
        iterator.next()
        iterator.set(generator(index++))
    }
}

/**
 * Returns a [MutableList] that wraps the original array.
 */
fun <T> Array<T>.asMutableList(): MutableList<T> {
    return asList() as MutableList<T>
}

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
 * Iterates over all
 */
inline fun <T> List<T>.forEachReversed(action: (T) -> Unit) {
    val listIt = listIterator(size)
    while (listIt.hasPrevious()) {
        action(listIt.previous())
    }
}

/**
 * Runs forEach on each of the iterables [p1], [p2], zipped.
 */
inline fun <T, V> forEachZipped(p1: Iterable<T>, p2: Iterable<V>, action: (T, V) -> Unit) {
    let(p1.iterator(), p2.iterator()) { it1, it2 ->
        while (it1.hasNext() && it2.hasNext()) {
            action(it1.next(), it2.next())
        }
    }
}

/**
 * Runs forEach on each of the iterables [p1], [p2], zipped, and indexed.
 */
inline fun <T, V> forEachZippedIndexed(p1: Iterable<T>, p2: Iterable<V>, action: (index: Int, T, V) -> Unit) {
    var i = 0
    let(p1.iterator(), p2.iterator()) { it1, it2 ->
        while (it1.hasNext() && it2.hasNext()) {
            if (i < 0) throw ArithmeticException("Index overflow")
            action(i++, it1.next(), it2.next())
        }
    }
}

/**
 * Runs forEach on each of the iterables [this], [p2], zipped.
 */
inline fun <T, V> Iterable<T>.zipForEach(p2: Iterable<V>, action: (T, V) -> Unit): Unit =
    forEachZipped(this, p2, action)

/**
 * Runs forEach on each of the iterables [this], [p2], zipped.
 */
inline fun <T, V> Iterable<T>.zipForEachIndexed(p2: Iterable<V>, action: (index: Int, T, V) -> Unit): Unit =
    forEachZippedIndexed(this, p2, action)

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
 * Runs the [action] on all possible pairs of elements from the lists.
 */
//@ExperimentalContracts
inline fun <A, B> onAllPairs(listA: List<A>, listB: List<B>, action: (A, B) -> Unit) {
//    contract {
//        callsInPlace(block, InvocationKind.UNKNOWN)
//    }
    listA.forEach { a ->
        listB.forEach { b ->
            action(a, b)
        }
    }
}

