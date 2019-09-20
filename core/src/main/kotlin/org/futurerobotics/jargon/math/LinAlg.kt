@file:JvmName("LinAlg")

package org.futurerobotics.jargon.math

import koma.extensions.set
import koma.mat
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
 * Returns true if this is a square; i.e. numCols == numRows
 */
fun Matrix<*>.isSquare(): Boolean = numCols() == numRows()

/**
 * Returns true if this Matrix is the same size as the [other] matrix.
 */
infix fun Matrix<*>.sameSizeAs(other: Matrix<*>): Boolean =
    this.numRows() == other.numRows() && this.numCols() == other.numCols()

/**
 * DSL for concatenating matrices
 */
object MatConcat {
    /**
     * DSL using [], similar to [mat]
     */
    operator fun get(vararg ts: Any): Matrix<Double> {
        val numStops = ts.count { it is Pair<*, *> }
        val numRows = numStops + 1
        val numElements = ts.count() + numStops
        val numCols = numElements / numRows
        if (numRows * numCols != numElements) throwNotEven()
        val arr = Array(numRows) { arrayOfNulls<Any>(numCols) }
        var curRow = 0
        var curCol = 0
        for (element in ts) {
            if (curCol >= numCols) throwNotEven()
            when (element) {
                is Matrix<*>, is Number -> {
                    arr[curRow][curCol] = element
                    curCol++
                }
                is Pair<*, *> -> {
                    arr[curRow][curCol] = element.first
                    arr[curRow + 1][0] = element.second
                    curRow++
                    curCol = 1
                }
                else -> throw IllegalArgumentException("Invalid value given to builder.")
            }
        }
        return concat(arr)
    }

    /**
     * Flattens a 2d array of [Matrix]es or Numbers, concatenating them into a single Matrix.
     */
    fun concat(arr: Array<Array<Any?>>): Matrix<Double> {
        if (arr.isEmpty()) return zeros(0, 0)
        val cols = arr.first().map { it.numCols() }
        val rows = arr.map { it.first().numRows() }
        val outRows = rows.sum()
        val outCols = cols.sum()
        val out = zeros(outRows, outCols)
        var currentRow = 0
        arr.forEachIndexed { rowInd, row ->
            val requiredRows = rows[rowInd]
            var currentCol = 0
            row.forEachIndexed { colInd, it ->
                val requiredCols = cols[colInd]
                if (it.numCols() != requiredCols || it.numRows() != requiredRows) throwNotEven()
                @Suppress("UNCHECKED_CAST")
                when (it) {
                    is Matrix<*> ->
                        out[currentRow until (currentRow + rows[rowInd]),
                                currentCol until (currentCol + cols[colInd])] = it as Matrix<Double>
                    is Number ->
                        out[currentRow, currentCol] = it.toDouble()
                    else -> throwInvalidValue()
                }
                currentCol += requiredCols
            }
            currentRow += requiredRows
        }
        return out
    }

    private fun Any?.numCols(): Int = when (this) {
        is Matrix<*> -> this.numCols()
        is Number -> 1
        else -> throwInvalidValue()
    }

    private fun Any?.numRows(): Int = when (this) {
        is Matrix<*> -> this.numRows()
        is Number -> 1
        else -> throwInvalidValue()
    }

    private fun throwInvalidValue(): Nothing = throw IllegalArgumentException("Invalid value given to builder.")

    private fun throwNotEven(): Nothing = throw IllegalArgumentException("Even rows/cols not given")

}

