package org.futurerobotics.jargon.pathing.builder

/** Utility used by path builder. May be made public someday. */
internal class Sweeper<out T : Any>(private val list: List<T>) : Iterator<T> {

    /** Index that will be returned by next. */
    private var index = 0

    /** What was _just_ returned by next. */
    val current get() = list[index - 1]

    override fun hasNext() = index < list.size

    override fun next(): T = list[index++]

    /** Sees next or `null` if none */
    fun tryNext() = if (hasNext()) next() else null

    /** Sees next element without advancing, or `null` if none */
    fun peek() = if (hasNext()) list[index] else null

    inline fun takeWhile(crossinline predicate: (T) -> Boolean): List<T> {
        val list = ArrayList<T>()
        while (
            peek().let { it != null && predicate(it) }
        ) list += next()
        return list
    }
}
