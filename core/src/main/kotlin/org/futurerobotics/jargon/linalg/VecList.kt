package org.futurerobotics.jargon.linalg
/*
 * Operations from converting between vectors and lists of doubles.
 *
 * Operations with vectors and double arrays already exist.
 */
/**
 * Returns this [vecFrom] as a list. Changes in the vector will be reflected in this list.
 */
fun Vec.asList(): List<Double> = object : AbstractList<Double>(), RandomAccess {
    override val size: Int
        get() = dimension

    override fun get(index: Int): Double = this@asList[index]
}

/**
 * Returns this [vecFrom] as a mutable list. Changes in the list will be reflected in the vector, and vice-versa.
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

/** Converts this list of doubles to a vector. */
fun List<Double>.toVec(): Vec = vecFrom(toDoubleArray(), false)
