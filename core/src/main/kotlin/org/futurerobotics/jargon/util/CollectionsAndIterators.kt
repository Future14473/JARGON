@file:JvmName("MoreCollections")

package org.futurerobotics.jargon.util

import java.util.*
import kotlin.collections.ArrayList
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.contract

/** Fills an array using the given [generator], given indexes. */
inline fun <T> Array<T>.fillWith(generator: (Int) -> T) {
    repeat(size) {
        this[it] = generator(it)
    }
}

/** Fills a mutable list using the given [generator], given indexes. */
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
fun <T> Array<T>.asMutableList(): MutableList<T> = asList() as MutableList<T>

/**
 * Creates a mutable list with a fixed [size], filling using [init].
 */
inline fun <reified T> fixedSizeMutableList(size: Int, init: (Int) -> T): MutableList<T> {
    @Suppress("UNCHECKED_CAST")
    return (arrayOfNulls<T>(size).apply { fillWith(init) } as Array<T>).asMutableList()
}

/**
 * Creates a mutable list with a fixed [size], initializing with nulls.
 */
inline fun <reified T> fixedSizeMutableListOfNulls(size: Int): MutableList<T?> = arrayOfNulls<T>(size).asMutableList()

/** Wraps this list with [Collections.unmodifiableList]. */
fun <T> List<T>.asUnmodifiableList(): List<T> = Collections.unmodifiableList(this)

/** Wraps this map with [Collections.unmodifiableMap]. */
fun <T, R> Map<T, R>.asUnmodifiableMap(): Map<T, R> = Collections.unmodifiableMap(this)

/** Wraps this set with [Collections.unmodifiableSet]. */
fun <T> Set<T>.asUnmodifiableSet(): Set<T> = Collections.unmodifiableSet(this)

/** Wraps this collection with [Collections.unmodifiableCollection] */
fun <T> Collection<T>.asUnmodifiableCollection(): Collection<T> = Collections.unmodifiableCollection(this)

/**
 * Until kotlinx.immutableCollections becomes stable, turns this list into an effectively immutable list
 * by wrapping a copy of this list in [asUnmodifiableList]. Call sparingly.
 */
fun <T> List<T>.toImmutableList(): List<T> = toList().asUnmodifiableList()

/**
 * Replaces the values of this [MutableList] through the [mapping] function to itself, replacing values.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T> MutableList<T>.mapToSelf(mapping: (T) -> T) {
    contract {
        callsInPlace(mapping)
    }
    val iterator = listIterator()
    while (iterator.hasNext()) {
        iterator.set(mapping(iterator.next()))
    }
}

/** @return true if the values given by this iterable is sorted. */
fun <T> Iterable<T>.isSorted(): Boolean where T : Comparable<T> = isSortedBy { it }

/** @return true if the values of this iterator is sorted with values given by the [selector], inlined. */
@UseExperimental(ExperimentalContracts::class)
inline fun <T, V : Comparable<V>> Iterable<T>.isSortedBy(selector: (T) -> V): Boolean {
    contract {
        callsInPlace(selector)
    }
    val iterator = iterator()
    var prev = if (iterator.hasNext()) selector(iterator.next()) else return true
    while (iterator.hasNext()) {
        val cur = selector(iterator.next())
        if (cur < prev) return false
        prev = cur
    }
    return true
}

/** [forEach], but in reverse direction. */
@UseExperimental(ExperimentalContracts::class)
inline fun <T> List<T>.forEachReversed(action: (T) -> Unit) {
    contract {
        callsInPlace(action)
    }
    val iterator = listIterator(size)
    while (iterator.hasPrevious()) {
        action(iterator.previous())
    }
}

/**
 * Runs forEach on each of the iterables [this], [p2], zipped.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T, V> Iterable<T>.zipForEachIndexed(p2: Iterable<V>, action: (index: Int, T, V) -> Unit) {
    contract {
        callsInPlace(action)
    }
    var i = 0
    val it1 = this.iterator()
    val it2 = p2.iterator()
    while (it1.hasNext() && it2.hasNext()) {
        if (i < 0) throw ArithmeticException("Index overflow")
        action(i++, it1.next(), it2.next())
    }
}

/**
 * Runs forEach on each of the iterables [this], [p2], zipped.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <T, V> Iterable<T>.zipForEach(p2: Iterable<V>, action: (T, V) -> Unit) {
    contract {
        callsInPlace(action)
    }
    val it1 = this.iterator()
    val it2 = p2.iterator()
    while (it1.hasNext() && it2.hasNext()) {
        action(it1.next(), it2.next())
    }
}

/**
 * Returns a list of all possible pairs of elements from the given lists.
 */
fun <A, B> mapAllPairs(list1: List<A>, list2: List<B>): List<Pair<A, B>> {
    if (list1.isEmpty() || list2.isEmpty()) return emptyList()
    val result = ArrayList<Pair<A, B>>(list1.size * list2.size)
    list1.forEach { a ->
        list2.forEach { b ->
            result += a to b
        }
    }
    return result
}

/**
 * Returns a list of the result of the [mapping] function called on all possible pairs of elements from the lists
 */
@UseExperimental(ExperimentalContracts::class)
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

/**
 * Runs the [action] on all possible pairs of elements from the lists.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun <A, B> onAllPairs(listA: List<A>, listB: List<B>, action: (A, B) -> Unit) {
    contract {
        callsInPlace(action)
    }
    listA.forEach { a ->
        listB.forEach { b ->
            action(a, b)
        }
    }
}

