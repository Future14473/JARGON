@file:JvmName("LinAlg")

package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.RealMatrix
import org.hipparchus.linear.RealVector

/** Shorthand for a [RealMatrix] */
typealias Mat = RealMatrix

/** Shorthand for a [RealVector]. */
typealias Vec = RealVector

private val dummyArr = pureDiag(0.0)

/**
 * Functions for concatenating matrices
 */
object MatConcat {
    /**
     * DSL using [] to concatenate matrices (or numbers which represent single elements), similar to [mat].
     *
     * Recommended use in kotlin only.
     */
    operator fun get(vararg elements: Any): Mat {
        val numStops = elements.count { it is Pair<*, *> }
        val numElements = elements.size + numStops
        val numRows = numStops + 1
        val numCols = numElements / numRows
        if (numRows * numCols != numElements) throwNotEven()
        val arr = Array(numRows) { Array(numCols) { dummyArr } }
        var curRow = 0
        var curCol = 0
        fun Any.convertToMat(): Mat = when (this) {
            is Mat -> this
            is Number -> pureDiag(toDouble())
            else -> throwInvalidValue()
        }
        elements.forEach { element ->
            if (curCol >= numCols) throwNotEven() //if stop late
            when (element) {
                is Pair<*, *> -> {
                    if (curCol != numCols - 1) throwNotEven() //if stop early
                    arr[curRow][curCol] = element.first?.convertToMat() ?: throwInvalidValue()
                    arr[curRow + 1][0] = element.second?.convertToMat() ?: throwInvalidValue()
                    curRow++
                    curCol = 1
                }
                else -> {
                    arr[curRow][curCol] = element.convertToMat()
                    curCol++
                }
            }
        }

        return concat(arr)
    }


    /**
     * Takes a 2d array of [Mat]rices with compatible sizes, concatenating them into a single Matrix.
     */
    @JvmStatic
    fun concat(arr: Array<out Array<out Mat>>): Mat {
        if (arr.isEmpty()) return zeros(0, 0)
        val cols = arr.first().map { it.cols }
        val rows = arr.map { it.first().rows }
        val outRows = rows.sum()
        val outCols = cols.sum()
        val out = zeros(outRows, outCols)
        var currentRow = 0
        arr.forEachIndexed { rowInd, row ->
            val requiredRows = rows[rowInd]
            var currentCol = 0
            row.forEachIndexed { colInd, it ->
                val requiredCols = cols[colInd]
                if (it.cols != requiredCols || it.rows != requiredRows) throwNotEven()
                out[currentRow, currentCol] = it
                currentCol += requiredCols
            }
            currentRow += requiredRows
        }
        return out
    }

    /** Common operation of concatenating 4 matrices of compatible size into a 2x2 pattern */
    @JvmStatic
    fun concat2x2(m11: Mat, m12: Mat, m21: Mat, m22: Mat): Mat {
        require(m11.rows == m12.rows) { "Size must match" }
        require(m21.rows == m22.rows) { "Size must match" }
        require(m11.cols == m21.cols) { "Size must match" }
        require(m12.cols == m22.cols) { "Size must match" }
        val row = m11.rows
        val col = m11.cols
        return zeros(row + m21.rows, col + m12.cols).apply {
            this[0, 0] = m11
            this[row, 0] = m21
            this[0, col] = m12
            this[row, col] = m22
        }
    }


    /**
     * Common operation of concatenates 4 _square_ matrices of equal size into a single matrix.
     *
     * This is also _dynamic_:
     * If at least one matrix is supplied and so the size can be determined, use the value 0 to indicate a zero
     * matrix and the value 1 to indicate an identity matrix.
     * */
    @JvmStatic
    fun square2x2(m11: Any, m12: Any, m21: Any, m22: Any): Mat {
        var size: Int = -1
        val all = arrayOf(m11, m12, m21, m22)
        all.forEach {
            when (it) {
                is Mat -> {
                    require(it.isSquare) { "Matrix given must be square" }
                    if (size == -1) size = it.rows
                    else require(size == it.rows) { "All matrices give must be same size" }
                }
                !is Number -> throwInvalidValue()
            }
        }
        require(size != -1) { "At least one matrix must be given" }
        fun convertToMat(any: Any, size: Int): Mat = when (any) {
            is Mat -> any
            !is Number -> throwInvalidValue()
            0 -> pureZeroSquare(size)
            1 -> pureEye(size)
            else -> throw IllegalArgumentException(
                "Number value given must be 0, for zero matrix, or 1, for identity matrix"
            )
        }
        return zeros(size * 2, size * 2).apply {
            this[0, 0] = convertToMat(m11, size)
            this[size, 0] = convertToMat(m12, size)
            this[0, size] = convertToMat(m21, size)
            this[size, size] = convertToMat(m22, size)
        }
    }


    private fun throwInvalidValue(): Nothing = throw IllegalArgumentException("Invalid value given")

    private fun throwNotEven(): Nothing = throw IllegalArgumentException("Even rows/cols not given")

}

