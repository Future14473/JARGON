package org.futurerobotics.jargon.linalg

/**
 * Functions for concatenating matrices
 */
object MatConcat {

    private val dummyMat = zeroMat(1, 1)
    /**
     * DSL using [] to concatenate matrices (or numbers that represent single elements), similar to [Mat].
     *
     * Use `[to]` to indicate end of a row
     *
     * Recommended use in kotlin only.
     */
    operator fun get(vararg elements: Any): Mat {
        val numStops = elements.count { it is Pair<*, *> }
        val numElements = elements.size + numStops
        val numRows = numStops + 1
        val numCols = numElements / numRows
        if (numRows * numCols != numElements) throwNotEven()
        val arr = Array(numRows) { Array(numCols) { dummyMat } }
        var curRow = 0
        var curCol = 0
        fun Any.convertToMat(): Mat = when (this) {
            is Mat -> this
            is Number -> diagMat(toDouble())
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
        if (arr.isEmpty()) return zeroMat(0, 0)
        val cols = arr.first().map { it.cols }
        val rows = arr.map { it.first().rows }
        val outRows = rows.sum()
        val outCols = cols.sum()
        val out = zeroMat(outRows, outCols)
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
        return zeroMat(row + m21.rows, col + m12.cols).apply {
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
     * matrix and the value 1 to indicate an identity matrix. (must be Int).
     * */
    @JvmStatic
    fun dynamic2x2Square(m11: Any, m12: Any, m21: Any, m22: Any): Mat {
        val all = arrayOf(m11, m12, m21, m22)
        val rows = intArrayOf(-1, -1)
        val cols = intArrayOf(-1, -1)
        all.forEachIndexed { i, it ->
            when (it) {
                is Mat -> {
                    val rowI = i / 2
                    val colI = i % 2
                    if (rows[rowI] == -1) rows[rowI] = it.rows
                    else require(it.rows == rows[rowI]) { "Row size at ${rowI + 1} must match" }
                    if (cols[colI] == -1) cols[colI] = it.cols
                    else require(it.cols == cols[colI]) { "Col size at ${colI + 1} must match" }
                }
                !is Number -> throwInvalidValue()
                0, 1 -> Unit
                else -> throwInvalidValue()
            }
        }
        all.forEachIndexed { i, it ->
            if (it != 1) return@forEachIndexed
            val rowI = i / 2
            val colI = i % 2
            if (rows[rowI] != -1) rows[rowI].let {
                if (cols[colI] == -1) cols[colI] = it
                else require(cols[colI] == it) { "Size around identity matrix must be square" }
            }
            //rows[rowI] == -1
            else cols[colI].let {
                require(it != -1) { "not enough information to deduce size of identity matrix" }
                rows[rowI] = it
            }
        }
        val rowSum = if (rows.none { it == -1 }) rows.sum() else -1
        val colSum = if (cols.none { it == -1 }) cols.sum() else -1
        all.forEachIndexed { i, it ->
            if (it != 0) return@forEachIndexed
            val rowI = i / 2
            val colI = i % 2
            if (rows[rowI] == -1) {
                require(colSum != -1) { "Not enough information to deduce size of zero matrix, even if assuming square" }
                rows[rowI] = colSum - rows[rowI.flip]
            }
            //if ^ happened and succeeded, this cannot happen.
            if (cols[colI] == -1) {
                require(rowSum != -1) { "Not enough information to deduce size of zero matrix, even if assuming square" }
                cols[colI] = rowSum - cols[colI.flip]
            }
        }
        require(rows.none { it == -1 }) { "Not enough information to deduce # of rows" }
        require(cols.none { it == -1 }) { "Not enough information to deduce # of cols" }
        return zeroMat(rows.sum(), cols.sum()).apply {
            this[0, 0] = convertToMat(m11, rows[0], cols[0])
            this[0, cols[0]] = convertToMat(m12, rows[0], cols[1])
            this[rows[0], 0] = convertToMat(m21, rows[1], cols[0])
            this[rows[0], cols[0]] = convertToMat(m22, rows[1], cols[1])
        }
    }

    private inline val Int.flip get() = if (this != 0) 0 else 1
    private fun convertToMat(any: Any, rows: Int, cols: Int): Mat = when (any) {
        is Mat -> any
        0 -> zeroMat(rows, cols)
        1 -> {
            assert(rows == cols) { "Rows must equal cols for identity matrix" }
            idenMat(rows)
        }
        else -> throw AssertionError()
    }

    private fun throwInvalidValue(): Nothing = throw IllegalArgumentException("Invalid value given")

    private fun throwNotEven(): Nothing = throw IllegalArgumentException("Even rows/cols not given")
}
