package org.futurerobotics.jargon.util

/**
 * A list which only has one value repeated.
 */
private class RepeatedList<T>(override val size: Int, private val value: T) : AbstractList<T>() {
    override fun contains(element: T): Boolean = element == value

    override fun containsAll(elements: Collection<T>): Boolean = elements.all { it == value }

    override fun get(index: Int): T {
        if (index !in 0..size) throw IndexOutOfBoundsException()
        return value
    }

    override fun indexOf(element: T): Int = if (element == value) 0 else -1

    override fun isEmpty(): Boolean = false

    override fun lastIndexOf(element: T): Int = if (element == value) size - 1 else -1

    override fun subList(fromIndex: Int, toIndex: Int): List<T> {
        if (fromIndex < 0 || toIndex > size) throw IndexOutOfBoundsException("fromIndex: $fromIndex, toIndex: $toIndex, size: $size")
        require(fromIndex <= toIndex) { "fromIndex: $fromIndex > toIndex: $toIndex" }
        return repeatedList(toIndex - fromIndex, value)
    }


}

/**
 * Returns a list whose elements are only the [value] repeated [size] times.
 */
fun <T> repeatedList(size: Int, value: T): List<T> {
    require(size >= 0) { "Size ($size) <= 0" }
    return when (size) {
        0 -> emptyList()
        else -> RepeatedList(size, value)
    }
}