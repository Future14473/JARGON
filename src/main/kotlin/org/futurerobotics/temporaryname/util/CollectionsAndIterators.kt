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
        val next = iterator.next()
        iterator.set(mapping(next))
    }
}
