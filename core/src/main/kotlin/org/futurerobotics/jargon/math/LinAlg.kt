package org.futurerobotics.jargon.math

import koma.extensions.*
import koma.matrix.Matrix
import koma.zeros

/** Shorthand for [Matrix]<Double> */
typealias Mat = Matrix<Double>

/**
 * Makes a diagonal matrix with the supplied [elements]
 */
fun createDiag(vararg elements: Double): Matrix<Double> {
    val out = zeros(elements.size, elements.size)
    elements.forEachIndexed { i, d ->
        out[i, i] = d
    }
    return out
}

/**
 * Makes a diagonal matrix with the supplied [elements]
 */
fun createDiag(elements: List<Double>): Matrix<Double> {
    val out = zeros(elements.size, elements.size)
    elements.forEachIndexed { i, d ->
        out[i, i] = d
    }
    return out
}

/**
 * Makes a matrix with the given [elements]
 */
fun create(elements: List<Double>): Matrix<Double> {
    val out = zeros(1, elements.size)
    elements.forEachIndexed { index, d ->
        out[index] = d
    }
    return out
}

/**
 * Sets the first [elements].size elements of [this] matrix to the supplied [elements]
 */
fun Matrix<Double>.setTo(elements: List<Double>): Matrix<Double> {
    require(this.size >= elements.size) { "Matrix needs at least as many elements as supplied" }
    elements.forEachIndexed { i, d ->
        this[i] = d
    }
    return this
}

/**
 * Returns true if this is a vector.
 */
fun Matrix<*>.isVector(): Boolean = this.numCols() == 1 || this.numRows() == 1

/**
 * The total of elements in this matrix.
 */
fun Matrix<*>.numElements(): Int = numCols() * numRows()