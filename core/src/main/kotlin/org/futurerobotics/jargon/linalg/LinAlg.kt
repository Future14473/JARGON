@file:JvmName("LinAlg")

package org.futurerobotics.jargon.linalg

import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector

/** Shorthand for RealMatrix */
typealias Mat = RealMatrix

typealias Vec = RealVector
//
//object MatConcat {
//    /**
//     * DSL using [], similar to [mat]
//     */
//    operator fun get(vararg ts: Any): Matrix<Double> {
//        val numStops = ts.count { it is Pair<*, *> }
//        val numRows = numStops + 1
//        val numElements = ts.count() + numStops
//        val numCols = numElements / numRows
//        if (numRows * numCols != numElements) throwNotEven()
//        val arr = Array(numRows) { arrayOfNulls<Any>(numCols) }
//        var curRow = 0
//        var curCol = 0
//        for (element in ts) {
//            if (curCol >= numCols) throwNotEven()
//            when (element) {
//                is AnyMatrix, is Number -> {
//                    arr[curRow][curCol] = element
//                    curCol++
//                }
//                is Pair<*, *> -> {
//                    arr[curRow][curCol] = element.first
//                    arr[curRow + 1][0] = element.second
//                    curRow++
//                    curCol = 1
//                }
//                else -> throw IllegalArgumentException("Invalid value given to builder.")
//            }
//        }
//        return concat(arr)
//    }
//
//    /**
//     * Flattens a 2d array of [Matrix]es or Numbers, concatenating them into a single Matrix.
//     */
//    fun concat(arr: Array<Array<Any?>>): Matrix<Double> {
//        if (arr.isEmpty()) return zeros(0, 0)
//        val cols = arr.first().map { it.numCols() }
//        val rows = arr.map { it.first().numRows() }
//        val outRows = rows.sum()
//        val outCols = cols.sum()
//        val out = zeros(outRows, outCols)
//        var currentRow = 0
//        arr.forEachIndexed { rowInd, row ->
//            val requiredRows = rows[rowInd]
//            var currentCol = 0
//            row.forEachIndexed { colInd, it ->
//                val requiredCols = cols[colInd]
//                if (it.numCols() != requiredCols || it.numRows() != requiredRows) throwNotEven()
//                @Suppress("UNCHECKED_CAST")
//                when (it) {
//                    is Matrix<*> ->
//                        out[currentRow until (currentRow + rows[rowInd]),
//                                currentCol until (currentCol + cols[colInd])] = it as Matrix<Double>
//                    is Number ->
//                        out[currentRow, currentCol] = it.toDouble()
//                    else -> throwInvalidValue()
//                }
//                currentCol += requiredCols
//            }
//            currentRow += requiredRows
//        }
//        return out
//    }
//
//    private fun Any?.numCols(): Int = when (this) {
//        is Matrix<*> -> this.numCols()
//        is Number -> 1
//        else -> throwInvalidValue()
//    }
//
//    private fun Any?.numRows(): Int = when (this) {
//        is Matrix<*> -> this.numRows()
//        is Number -> 1
//        else -> throwInvalidValue()
//    }
//
//    private fun throwInvalidValue(): Nothing = throw IllegalArgumentException("Invalid value given to builder.")
//
//    private fun throwNotEven(): Nothing = throw IllegalArgumentException("Even rows/cols not given")
//
//}
//
