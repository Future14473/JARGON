package org.futurerobotics.jargon.linalg

/**
 * Returns this [Vec] as a list.
 */
fun Vec.asList(): List<Double> = object : AbstractList<Double>() {
    override val size: Int
        get() = dimension

    override fun get(index: Int): Double = this@asList[index]
}

/**
 * Returns this [Vec] as a mutable list.
 */
fun Vec.asMutableList(): MutableList<Double> = object : AbstractMutableList<Double>() {
    override val size: Int get() = dimension

    override fun get(index: Int): Double = this@asMutableList[index]

    override fun set(index: Int, element: Double): Double {
        return this@asMutableList[index].also {
            this@asMutableList[index] = element
        }
    }

    override fun removeAt(index: Int): Double {
        throw UnsupportedOperationException("Vec list")
    }

    override fun add(index: Int, element: Double) {
        throw UnsupportedOperationException("Vec list")
    }

}

/**
 * Converts this [vec] to a list.
 */
fun Vec.toList(): List<Double> = toMutableList()

/**
 * Converts this [vec] to a mutableList.
 */
fun Vec.toMutableList(): MutableList<Double> = MutableList(dimension) { get(it) }