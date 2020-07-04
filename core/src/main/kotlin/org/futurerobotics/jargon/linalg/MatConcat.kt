package org.futurerobotics.jargon.linalg

/**
 * Concatenates matrices, using a similar syntax as [matOf].
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
 * In java, use [concat] instead with a 2d array literal of matrices.
 */
@JvmSynthetic
fun concat(vararg elements: Any): Mat {
    val mats = varargBuilderToArray<Mat, Array<Array<Mat>>>(
        elements,
        { rows, cols ->
            Array(rows) {
                @Suppress("UNCHECKED_CAST")
                arrayOfNulls<Mat>(cols) as Array<Mat>
            }
        },
        { r, c, e -> this[r][c] = e }
    )
    return concat(mats)
}

/**
 * Concatenates a 2d array of matrices with compatible sizes into a single matrix.
 */
fun concat(arr: Array<out Array<out Mat>>): Mat {
    require(arr.isNotEmpty()) { "Non empty array must be given" }
    val colSizes = arr.first().map { it.cols }
    val rowSizes = arr.map { it.first().rows }

    val resultRows = rowSizes.sum()
    val resultCols = colSizes.sum()
    val result = Mat(resultRows, resultCols)

    var curRow = 0
    arr.forEachIndexed { arrRowInd, arrRow ->
        require(arrRow.size == colSizes.size) { "Array has uneven rows" }
        val curMatRows = rowSizes[arrRowInd]
        var curCol = 0
        arrRow.forEachIndexed { arrCollInd, mat ->
            val curMatCols = colSizes[arrCollInd]
            require(mat.cols == curMatCols && mat.rows == curMatRows)
            { "Matrices do not have compatible dimensions to concatenate" }
            result[curRow, curCol] = mat
            curCol += curMatCols
        }
        curRow += curMatRows
    }

    return result
}

/** Concatenates two matrices on top of each other, in a column. */
fun concatCol(m1: Mat, m2: Mat): Mat {
    require(m1.cols == m2.cols) { "All matrices must have same number of columns." }
    val rows = m1.rows + m2.rows
    return Mat(rows, m1.cols).apply {
        this[0, 0] = m1
        this[m1.rows, 0] = m2
    }
}

/** Concatenates matrices on top of each other, in a column. */
fun concatCol(vararg mats: Mat): Mat {
    require(mats.isNotEmpty()) { "At least one matrix must be provided." }
    val cols = mats[0].cols
    mats.forEach {
        require(it.cols == cols) { "All matrices must have same number of columns." }
    }
    val resultRows = mats.sumBy { it.rows }
    return Mat(resultRows, cols).apply {
        var curRow = 0
        mats.forEach {
            this[curRow, 0] = it
            curRow += it.rows
        }
    }
}

/** Concatenates two matrices side by side, in a row. */
fun concatRow(m1: Mat, m2: Mat): Mat {
    require(m1.rows == m2.rows) { "Matrices must have same number of rows." }
    val cols = m1.cols + m2.cols
    return Mat(m1.rows, cols).apply {
        this[0, 0] = m1
        this[0, m1.cols] = m2
    }
}

/** Concatenates matrices side by side, in a row. */
fun concatRow(vararg mats: Mat): Mat {
    require(mats.isNotEmpty()) { "At least one matrix must be provided." }
    val rows = mats[0].rows
    mats.forEach {
        require(it.rows == rows) { "All matrices must have same number of rows." }
    }
    val cols = mats.sumBy { it.cols }
    return Mat(rows, cols).apply {
        var curCol = 0
        mats.forEach {
            this[0, curCol] = it
            curCol += it.cols
        }
    }
}

/** Concatenating 4 matrices of compatible sizes in a 2x2 pattern. */
fun concat2x2(m11: Mat, m12: Mat, m21: Mat, m22: Mat): Mat {
    require(
        m11.rows == m12.rows &&
            m21.rows == m22.rows &&
            m11.cols == m21.cols &&
            m12.cols == m22.cols
    ) { "Matrices do not have compatible dimensions to concatenate" }
    val mrow = m11.rows
    val mcol = m11.cols
    return Mat(mrow + m21.rows, mcol + m12.cols).apply {
        this[0, 0] = m11
        this[mrow, 0] = m21
        this[0, mcol] = m12
        this[mrow, mcol] = m22
    }
}

/** [concatDynamic] for a 2x2 matrix. */
fun concat2x2dynamic(m11: Any, m12: Any, m21: Any, m22: Any): Mat =
    concatDynamic(arrayOf(arrayOf(m11, m12), arrayOf(m21, m22)))

/**
 * Concatenating matrices into a single matrix, but 0's and 1's can be substituted as zero matrices and identity
 * matrices respectively instead of matrices if there is enough information from other matrices to infer their size.
 *
 * This uses the same syntax as ([concat] with vararg).
 *
 * Also, if only one dimension of the resulting matrix is known, then the resulting matrix is assumed to be square
 * and have their dimensions matched.
 */
@JvmSynthetic
fun concatDynamic(vararg elements: Any): Mat {
    val array = varargBuilderToArray<Any, Array<Array<Any>>>(
        elements,
        { rows, cols ->
            Array(rows) {
                @Suppress("UNCHECKED_CAST")
                arrayOfNulls<Any>(cols) as Array<Any>
            }
        },
        { r, c, e -> this[r][c] = e }
    )
    return concatDynamic(array)
}

/**
 * Concatenating matrices into a single matrix, but additionally:
 * - 0's and 1's can be substituted as zero matrices and identity matrices respectively
 * - The above can only be done if there is enough information from other matrices to infer their sizes.
 * - If only one dimension of the resulting matrix can be inferred, then the resulting matrix is assumed to be a square
 *  matrix.
 */
fun concatDynamic(arr: Array<out Array<out Any>>): Mat {
    val rowSizes = IntArray(arr.size)
    val colSizes = IntArray(arr[0].size)

    arr.forEachIndexed { rowI, row ->
        require(row.size == colSizes.size) { "Uneven array of matrices given." }
        row.forEachIndexed { colI, it ->
            when {
                it is Mat -> {
                    //known row size
                    if (rowSizes[rowI] == 0) rowSizes[rowI] = it.rows
                    else require(it.rows == rowSizes[rowI]) { "Row size at ${rowI + 1} must match" }
                    //known col size
                    if (colSizes[colI] == 0) colSizes[colI] = it.cols
                    else require(it.cols == colSizes[colI]) { "Col size at ${colI + 1} must match" }
                }
                it is Int && it == 0 || it == 1 -> Unit
                else -> throw IllegalArgumentException("Invalid value &it given")
            }
        }
    }
    //make sure identity matrices are square, and infer other dimensions with it
    arr.forEachIndexed { rowI, row ->
        row.forEachIndexed inner@{ colI, it ->
            if (it != 1) return@inner
            if (rowSizes[rowI] != 0) { //row size known
                if (colSizes[colI] != 0) { //col size known
                    require(rowSizes[rowI] == colSizes[colI]) { "Identity matrix must be square. " }
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
    val unknownRows = rowSizes.count { it == 0 }
    val unknownCols = colSizes.count { it == 0 }
    require(unknownRows <= 1 && unknownCols <= 1) { "Not enough information to deduce size of matrix" }
    var outRows = rowSizes.sum() //including 0s
    var outCols = colSizes.sum() //including 0s
    repeat(rowSizes.size) { rowI ->
        repeat(colSizes.size) { colI ->
            if (rowSizes[rowI] == 0) {
                require(unknownCols == 0) {
                    "Not enough information to deduce size of zero matrix, even if " +
                        "assuming resulting matrix is square."
                }
                rowSizes[rowI] = outCols - outRows
                outRows = outCols
            }
            if (colSizes[colI] == 0) {
                require(unknownRows == 0) {
                    "Not enough information to deduce size of zero matrix, even if " +
                        "assuming resulting matrix is square."
                }
                colSizes[colI] = outRows - outCols
                outCols = outRows
            }
        }
    }
    val result = Mat(outRows, outCols)
    var currentRow = 0
    arr.forEachIndexed { rowInd, row ->
        val requiredRows = rowSizes[rowInd]
        var currentCol = 0
        row.forEachIndexed { colInd, it ->
            val requiredCols = colSizes[colInd]
            when (it) {
                is Mat -> {
                    result[currentRow, currentCol] = it
                }
                1 -> {
                    assert(requiredRows == requiredCols)
                    result[currentRow, currentCol] = idenMat(requiredRows)
                }
//              0 -> { }
            }
            currentCol += requiredCols
        }
        currentRow += requiredRows
    }
    return result
}

