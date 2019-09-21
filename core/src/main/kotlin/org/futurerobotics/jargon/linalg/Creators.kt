package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.ArrayRealVector
import org.hipparchus.linear.DiagonalMatrix
import org.hipparchus.linear.MatrixUtils


/**
 * Creates a matrix with zeros given the [numRows] and [numCols]
 */
fun zeros(numRows: Int, numCols: Int): Mat = MatrixUtils.createRealMatrix(numRows, numCols)

fun create(data: Array<DoubleArray>): Mat = MatrixUtils.createRealMatrix(data)

fun createVec(v: DoubleArray): Vec = ArrayRealVector(v, true)
fun createVec(v: List<Double>): Vec = ArrayRealVector(v.toDoubleArray(), false)
@JvmName("createVecVararg")
fun createVec(vararg v: Double): Vec = ArrayRealVector(v, false)

fun zeroVec(size: Int) = ArrayRealVector(size)

fun diag(v: DoubleArray): Mat = DiagonalMatrix(v)
fun diag(v: List<Double>): Mat = DiagonalMatrix(v.toDoubleArray(), false)
@JvmName("createDiagVararg")
fun diag(vararg v: Double): Mat = DiagonalMatrix(v, false)


fun eye(size: Int): Mat = MatrixUtils.createRealIdentityMatrix(size)

fun pureEye(size: Int): Mat = DiagonalMatrix(DoubleArray(size) { 1.0 })

object mat {
    operator fun get(vararg values: Any): Mat {
        val stops = values.count { it is Pair<*, *> }
        val items = values.count() + stops
        val rows = stops + 1
        val cols = items / rows
        require(rows * cols == items) { "Even rows/cols not given" }
        val out = zeros(rows, cols)
        var curRow = 0
        var curCol = 0
        values.forEach {
            when (it) {
                is Number -> {
                    require(curCol < cols) { "Even rows/cols not given" }
                    out[curRow, curCol] = it.toDouble()
                    curCol++
                }
                is Pair<*, *> -> {
                    val (a, b) = it
                    require(a is Number && b is Number) { "Invalid value given" }
                    require(curRow < rows) { "Even rows/cols not given" }
                    out[curRow, curCol] = a.toDouble()
                    curRow++
                    out[curRow, 0] = b.toDouble()
                    curCol = 1
                }
                else -> require(false) { "Invalid value given" }
            }
        }
        return out
    }
}

infix fun Number.end(other: Number) = this to other