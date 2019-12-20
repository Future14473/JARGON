package org.futurerobotics.jargon.linalg

/**
 * Returns this [Vec] as a list. Changes in the vector will be reflected in this list.
 */
fun Vec.asList(): List<Double> = object : AbstractList<Double>(), RandomAccess {
    override val size: Int
        get() = dimension

    override fun get(index: Int): Double = this@asList[index]
}

/**
 * Returns this [Vec] as a mutable list. Changes in the list will be reflected in the vector, and vice-versa.
 */
fun Vec.asMutableList(): MutableList<Double> = object : AbstractMutableList<Double>(), RandomAccess {
    override val size: Int get() = dimension

    override fun get(index: Int): Double = this@asMutableList[index]

    override fun set(index: Int, element: Double): Double = this@asMutableList[index].also {
        this@asMutableList[index] = element
    }

    override fun removeAt(index: Int): Double {
        throw UnsupportedOperationException("Vec list")
    }

    override fun add(index: Int, element: Double) {
        throw UnsupportedOperationException("Vec list")
    }
}

/**
 * Converts this vector to a list.
 */
fun Vec.toList(): List<Double> = toMutableList()

/**
 * Converts this vector to a mutable list.
 */
fun Vec.toMutableList(): MutableList<Double> = MutableList(size) { get(it) }

/** Converts this list of doubles to a [Vec]. */
fun List<Double>.toVec(): Vec = createVec(this)

/** Converts this array of doubles to a [Vec]. */
fun DoubleArray.toVec(): Vec = createVec(this, true)
