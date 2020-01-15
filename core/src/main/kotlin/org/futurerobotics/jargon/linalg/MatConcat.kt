package org.futurerobotics.jargon.linalg

import org.futurerobotics.jargon.util.uncheckedCast

/**
 * Concatenates a matrix using the given matrices.
 *
 * Matrices should be separated by commas, and rows separated using the
 * infix function [to].
 *
 * For example:
 * ```kotlin
 * concat(
 *  m1, m2, m3 to
 *  m4, m5, m6
 * )
 * ```
 *
 * In java, use [createMat] instead with a 2d array literal.
 */
@JvmSynthetic
fun concat(vararg elements: Any): Mat {
    lateinit var arr: Array<Array<Mat>>
    varargEndToArr<Mat>(
        elements,
        { rows, cols ->
            arr = Array(rows) {
                arrayOfNulls<Mat>(cols).uncheckedCast<Array<Mat>>()
            }
        },
        { r, c, e -> arr[r][c] = e }
    )
    return concat(arr)
}

/**
 * Concatenates a 2d array of [Mat]rices with compatible sizes into a single matrix.
 */
fun concat(arr: Array<out Array<out Mat>>): Mat {
    if (arr.isEmpty()) return zeroMat(0, 0)
    val colSizes = arr.first().map { it.cols }
    val rowSizes = arr.map { it.first().rows }
    val rowResult = rowSizes.sum()
    val colResult = colSizes.sum()
    val result = zeroMat(rowResult, colResult)
    var curRow = 0
    arr.forEachIndexed { arrRowI, arrRow ->
        require(arrRow.size == colSizes.size) { "Even rows must be given" }
        val curMatRows = rowSizes[arrRowI]
        var currentCol = 0
        arrRow.forEachIndexed { arrColI, it ->
            val curMatCols = colSizes[arrColI]
            if (it.cols != curMatCols || it.rows != curMatRows) throw IllegalArgumentException("Even rows/cols not given")
            result[curRow, currentCol] = it
            currentCol += curMatCols
        }
        curRow += curMatRows
    }
    return result
}

/** Concatenates two matrices on top of each other, in a column. */
fun concatCol(m1: Mat, m2: Mat): Mat {
    require(m1.cols == m2.cols) { "All matrices must have sane number of columns." }
    val rows = m1.rows + m2.rows
    return zeroMat(rows, m1.cols).apply {
        this[0, 0] = m1
        this[m1.rows, 0] = m2
    }
}

/** Concatenates matrices on top of each other, in a column. */
fun concatCol(vararg mats: Mat): Mat {
    require(mats.isNotEmpty()) { "At least one matrix must be provided." }
    val cols = mats[0].cols
    mats.forEach {
        require(it.cols == cols) { "All matrices must have sane number of columns." }
    }
    val rows = mats.sumBy { it.rows }
    return zeroMat(rows, cols).apply {
        var curRow = 0
        mats.forEach {
            this[curRow, 0] = it
            curRow += it.rows
        }
    }
}

/** Concatenates two matrices side by side, in a row. */
fun concatRow(m1: Mat, m2: Mat): Mat {
    require(m1.rows == m2.rows) { "Matrices must have same number of columns." }
    val cols = m1.cols + m2.cols
    return zeroMat(m1.rows, cols).apply {
        this[0, 0] = m1
        this[0, m1.cols] = m2
    }
}

/** Concatenates matrices side by side, in a row. */
fun concatRow(vararg mats: Mat): Mat {
    require(mats.isNotEmpty()) { "At least one matrix must be provided." }
    val rows = mats[0].rows
    mats.forEach {
        require(it.rows == rows) { "All matrices must have sane number of rows." }
    }
    val cols = mats.sumBy { it.cols }
    return zeroMat(rows, cols).apply {
        var curCol = 0
        mats.forEach {
            this[0, curCol] = it
            curCol += it.cols
        }
    }
}

/** Concatenating 4 matrices of compatible sizes in a 2x2 pattern. */
fun concat2x2(m11: Mat, m12: Mat, m21: Mat, m22: Mat): Mat {
    require(m11.rows == m12.rows) { "Size must match" }
    require(m21.rows == m22.rows) { "Size must match" }
    require(m11.cols == m21.cols) { "Size must match" }
    require(m12.cols == m22.cols) { "Size must match" }
    val row = m11.rows
    val col = m11.cols
    return zeroMat(row + m21.rows, col + m12.cols).apply {
        this[0, 0] = m11
        this[row, 0] = m21
        this[0, col] = m12
        this[row, col] = m22
    }
}

/** [concatDynamic] for a 2x2 matrix. */
fun concat2x2dynamic(m11: Any, m12: Any, m21: Any, m22: Any): Mat =
    concatDynamic(arrayOf(arrayOf(m11, m12), arrayOf(m21, m22)))

/**
 * Combination of [concat] with vararg and [concatDynamic]
 */
@JvmSynthetic
fun concatDynamic(vararg elements: Any): Mat {
    lateinit var arr: Array<Array<Any>>
    varargEndToArr<Any>(
        elements,
        { rows, cols ->
            arr = Array(rows) {
                arrayOfNulls<Any>(cols).uncheckedCast<Array<Any>>()
            }
        },
        { r, c, e -> arr[r][c] = e }
    )
    return concatDynamic(arr)
}

/**
 * Concatenating matrices into a single matrix, where sizes of some matrices can be inferred if there is enough
 * information.
 *
 * **Also,** a value of 0 substitutes in a zero matrix, while a value of 1 substitutes an identity
 * matrix. The sizes of these matrices can be inferred from other matrices provided.
 *
 * If only one dimension of the resulting matrix is known, the matrix is assume to be square, and the other
 * dimensions will then attempt to be inferred.
 */
fun concatDynamic(arr: Array<out Array<out Any>>): Mat {
    val rowSizes = IntArray(arr.size)
    val colSizes = IntArray(arr[0].size)

    arr.forEachIndexed { rowI, row ->
        require(row.size == colSizes.size) { "Uneven matrix array given." }
        row.forEachIndexed { colI, it ->
            when (it) {
                is Mat -> {
                    if (rowSizes[rowI] == 0) rowSizes[rowI] = it.rows
                    else require(it.rows == rowSizes[rowI]) { "Row size at ${rowI + 1} must match" }
                    if (colSizes[colI] == 0) colSizes[colI] = it.cols
                    else require(it.cols == colSizes[colI]) { "Col size at ${colI + 1} must match" }
                }
                !is Number -> throw IllegalArgumentException("Invalid value given")
                0, 1 -> Unit
                else -> throw IllegalArgumentException("Invalid value given")
            }
        }
    }
    arr.forEachIndexed { rowI, row ->
        row.forEachIndexed inner@{ colI, it ->
            if (it != 1) return@inner
            if (rowSizes[rowI] != 0) { //row size known
                if (colSizes[colI] != 0) { //col size known
                    require(rowSizes[rowI] == colSizes[colI]) { "Size around identity matrix must be square." }
                } else { //col size unknown
                    colSizes[colI] = rowSizes[rowI]
                }
            } else { //row size unknown
                if (colSizes[colI] != 0) { //col size known
                    rowSizes[rowI] = colSizes[colI]
                } else { //bot unknown
                    require(it != 0) { "Not enough information to deduce size of identity matrix" }
                }
            }
        }
    }
    val unknownRowsCount = rowSizes.count { it == -1 }
    val unknownColsCount = rowSizes.count { it == -1 }
    require(unknownRowsCount <= 1 && unknownColsCount <= 1) { "Not enough information to deduce size of matrix" }
    var outRows = rowSizes.sum() //including 0s
    var outCols = colSizes.sum() //including 0s
    repeat(rowSizes.size) { rowI ->
        repeat(colSizes.size) { colI ->
            if (rowSizes[rowI] == 0) {
                require(unknownColsCount == 0) {
                    "Not enough information to deduce size of zero matrix, even if " +
                            "assuming resulting matrix is square."
                }
                rowSizes[rowI] = outCols - outRows
                outRows = outCols
            }
            if (colSizes[colI] == 0) {
                require(unknownRowsCount == 0) {
                    "Not enough information to deduce size of zero matrix, even if " +
                            "assuming resulting matrix is square."
                }
                colSizes[colI] = outRows - outCols
                outCols = outRows
            }
        }
    }
    val out = zeroMat(outRows, outCols)
    var currentRow = 0
    arr.forEachIndexed { rowInd, row ->
        val requiredRows = rowSizes[rowInd]
        var currentCol = 0
        row.forEachIndexed { colInd, it ->
            val requiredCols = colSizes[colInd]
            if (it != 0)
                out[currentRow, currentCol] = convertToMat(it, requiredRows, requiredCols)
            currentCol += requiredCols
        }
        currentRow += requiredRows
    }
    return out
}

private fun convertToMat(any: Any, rows: Int, cols: Int): Mat = when (any) {
    is Mat -> any
    1 -> {
        assert(rows == cols) { "Rows must equal cols for identity matrix" }
        idenMat(rows)
    }
    else -> throw AssertionError()
}
